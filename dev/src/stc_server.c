// stc_server.c
//
// STC runtime daemon (TCP CMD/RSP <-> CANopen PD4-C) + UDP unicast telemetry.
//
// Architecture:
//  - TCP thread (main): receive CMD, parse/validate, perform fast side-effects (state/manualpos),
//    and enqueue discrete actions for CAN worker.
//  - CAN worker thread: owns pd4c_ctx_t, executes queued commands, runs safety watchdog,
//    polls TPDOs and updates shared telemetry.
//  - Telemetry threads: publish TLMH @100Hz and TLML @10Hz over UDP unicast.
//
/* Build example:
 *   gcc -std=c11 -O2 -Wall -Wextra -pthread \
 *       stc_icd.c pd4c.c stc_server.c -o stc_server -lm
 */

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif
#define _POSIX_C_SOURCE 200809L

#include "stc_icd.h"
#include "stc_config.h"
#include "stc_axis_cfg_map.h"
#include "stc_diag.h"
#include "stc_model.h"
#include "stc_actions.h"
#include "stc_version.h"
#include "stc_limits.h"
#include "stc_state.h"
#include "stc_faults.h"
#include "stc_log.h"

#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <poll.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <limits.h>
#include <stdatomic.h>
#include <inttypes.h>

#include "motion_backend.h"

#define STC_DIAG_LATCH_MS 2000ULL

/* ----------------------------- time helpers ----------------------------- */

static uint64_t mono_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static uint64_t mono_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static uint64_t wall_ms(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}

static uint64_t elapsed_ms_nonnegative(uint64_t now_ms, uint64_t last_ms) {
  if (now_ms < last_ms) {
    return 0;
  }
  return now_ms - last_ms;
}

static _Atomic sig_atomic_t g_stop = 0;
static _Atomic int g_listener_fd = -1;
static _Atomic int g_client_fd = -1;

static void sleep_until(struct timespec* next, long period_ns) {
  next->tv_nsec += period_ns;
  while (next->tv_nsec >= 1000000000L) {
    next->tv_nsec -= 1000000000L;
    next->tv_sec += 1;
  }

  for (;;) {
    int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL);
    if (rc == 0) {
      return;
    }
    if (rc == EINTR) {
      if (atomic_load(&g_stop)) {
        return;
      }
      continue;
    }

    STC_LOG_WARN("clock_nanosleep(TIMER_ABSTIME) failed: rc=%d (%s)", rc, strerror(rc));
    return;
  }
}

static uint64_t abs_diff_u64(uint64_t a, uint64_t b) {
  return (a >= b) ? (a - b) : (b - a);
}

static inline uint8_t stc_state_to_wire(stc_state_t s) {
  return (s == STC_STATE_FAIL) ? 0xFFu : (uint8_t)s;
}

/* TLML motor status byte (per canonical axis order):
 * bit 0: axis configured/enabled
 * bit 1: axis available/connected
 * bit 2: axis fault
 * bit 3: axis stale
 * bit 4: command saturated recently
 * bit 5: moving or speed-streaming
 * bit 6: degraded/warning
 * bit 7: reserved
 */
static uint8_t build_axis_health_byte(int enabled,
                                      int available,
                                      int fault,
                                      int stale,
                                      int saturated,
                                      int moving_or_streaming,
                                      int degraded) {
  uint8_t flags = 0;
  if (enabled) flags |= (1u << 0);
  if (available) flags |= (1u << 1);
  if (fault) flags |= (1u << 2);
  if (stale) flags |= (1u << 3);
  if (saturated) flags |= (1u << 4);
  if (moving_or_streaming) flags |= (1u << 5);
  if (degraded) flags |= (1u << 6);
  return flags;
}

static const char* stc_state_to_string(stc_state_t state) {
  switch (state) {
    case STC_STATE_INIT: return "INIT";
    case STC_STATE_INIT_DONE: return "INIT_DONE";
    case STC_STATE_SAFE: return "SAFE";
    case STC_STATE_OPER: return "OPER";
    case STC_STATE_SELF_LEVELLING: return "SELF_LEVELLING";
    case STC_STATE_STORE: return "STORE";
    case STC_STATE_IBIT_RUNNING: return "IBIT_RUNNING";
    case STC_STATE_FAIL: return "FAIL";
    default: return "UNKNOWN";
  }
}

/* ----------------------------- networking helpers ----------------------------- */

static int send_all(int fd, const uint8_t* buf, size_t len) {
  size_t off = 0;
  while (off < len) {
    ssize_t w = send(fd, buf + off, len - off, 0);
    if (w < 0) {
      if (errno == EINTR) continue;
      return -1;
    }
    off += (size_t)w;
  }
  return 0;
}

static int recv_exact(int fd, uint8_t* buf, size_t len) {
  size_t off = 0;
  while (off < len) {
    ssize_t r = recv(fd, buf + off, len - off, 0);
    if (r == 0) return 0;  // closed
    if (r < 0) {
      if (errno == EINTR) {
        if (atomic_load(&g_stop)) return -1;
        continue;
      }
      return -1;
    }
    off += (size_t)r;
  }
  return 1;
}

static int open_udp_tx(void) {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return -1;

  return fd;
}

static int udp_send_to(int fd, const char* dest_ip, int port, const uint8_t* buf, size_t len) {
  struct sockaddr_in dst;
  memset(&dst, 0, sizeof(dst));
  dst.sin_family = AF_INET;
  dst.sin_port = htons((uint16_t)port);
  if (inet_pton(AF_INET, dest_ip, &dst.sin_addr) != 1) return -1;

  ssize_t s = sendto(fd, buf, len, 0, (struct sockaddr*)&dst, sizeof(dst));
  return (s == (ssize_t)len) ? 0 : -1;
}

/* ----------------------------- shared state ----------------------------- */

typedef struct {
  uint64_t sample_count;
  uint64_t deadline_miss_count;
  uint64_t no_destination_count;
  uint64_t send_error_count;
  uint64_t packets_sent_count;
  uint64_t expected_wake_ns;
  uint64_t actual_wake_ns;
  uint64_t send_complete_ns;
  uint64_t current_sec_samples;
  uint64_t current_sec_jitter_sum_ns;
  uint64_t current_sec_jitter_min_ns;
  uint64_t current_sec_jitter_max_ns;
  uint64_t current_sec_idx;
  uint64_t last_sec_samples;
  uint64_t last_sec_jitter_min_ns;
  uint64_t last_sec_jitter_max_ns;
  uint64_t last_sec_jitter_avg_ns;
} tx_timing_stats_t;

typedef struct {
  uint64_t samples;
  uint64_t invalid_samples;
  uint64_t tcp_recv_ns;
  uint64_t action_queued_ns;
  uint64_t exec_start_ns;
  uint64_t tcp_to_queue_avg_ns;
  uint64_t queue_to_exec_avg_ns;
  uint64_t end_to_end_avg_ns;
} cmd_timing_stats_t;

typedef struct shared_data {
  // --- per-axis config/telemetry/stream (axis0 = turret az active in step3)
  axis_cfg_t cfg[STC_AXIS_COUNT];
  uint8_t axis_node_id[STC_AXIS_COUNT];
  axis_telem_t telem[STC_AXIS_COUNT];
  axis_stream_t stream[STC_AXIS_COUNT];
  bool saturated[STC_AXIS_COUNT];
  uint64_t saturated_until_ms[STC_AXIS_COUNT];
  double storage_deg[STC_AXIS_COUNT];
  uint8_t diag_cat[STC_AXIS_COUNT];
  uint8_t diag_subcat[STC_AXIS_COUNT];

  int turret_az_axis;
  int turret_el_axis;
  int sut_count;
  int sut_az_axis[STC_MAX_SUT];
  int sut_el_axis[STC_MAX_SUT];
  uint8_t sut_laser_on[STC_MAX_SUT];
  int mirror_count;
  int mirror_axis[STC_MAX_MIRROR];

  int speed_watchdog_ms;
  int speed_stream_period_ms;
  double position_tolerance_deg;
  int position_timeout_ms;
  int position_poll_ms;
  int ds402_timeout_ms;
  int pp_sdo_diag;              // enable SDO snapshot before PP move
  uint32_t can_timeout_ms;
  uint32_t startup_feedback_grace_ms;
  int strict_stale_policy;

  // --- software state / navigation
  stc_state_t st_state;
  uint8_t posmode;              // from POSMODE command

  int has_manualpos;
  double lat, lon, alt, heading;

  // --- latest motor telemetry (from pd4c_poll)
  uint8_t hb_state;
  int have_hb;

  uint16_t emcy_code;
  uint8_t emcy_reg;
  int have_emcy;

  uint64_t ts_tpdo1_ms;
  uint64_t ts_tpdo2_ms;
  uint64_t ts_tpdo3_ms;
  uint64_t ts_hb_ms;
  uint64_t ts_emcy_ms;

  // --- runtime flags
  int can_ok;
  int num_axes;
  uint8_t axis_available[STC_AXIS_COUNT];
  uint8_t axis_expected[STC_AXIS_COUNT];
  uint8_t can_discovered_nodes[16];
  uint8_t can_connected_devices;
  uint8_t can_discovered_devices;
  uint8_t backend_mode;
  uint8_t transport_status;
  uint8_t supports_native_a2_speed;
  uint8_t supports_native_absolute_position;
  uint16_t can_bitrate_kbps;
  uint16_t can_restart_ms;
  uint8_t can_scan_max_node;
  uint64_t last_rx_ms[STC_AXIS_COUNT];
  uint8_t axis_fault_active[STC_AXIS_COUNT];
  uint8_t axis_enabled[STC_AXIS_COUNT];
  uint8_t fail_stop_pending;
  uint32_t fail_stop_axes_mask;

  // --- telemetry seq + uptime
  uint16_t seq_h;
  uint16_t seq_l;
  uint64_t boot_ms;
  uint64_t server_start_ms;

  // --- observability
  tx_timing_stats_t tlmh_timing;
  tx_timing_stats_t tlml_timing;
  cmd_timing_stats_t cmd_timing;
  uint64_t action_queue_depth_peak;
  uint64_t obs_last_log_ms;

  // --- active CU telemetry destination (authoritative while connected)
  struct in_addr active_cu_addr;
  int active_cu_connected;

  // --- controller-id acceptance policy
  int controller_id_strict_legacy;

  // --- IBIT tracking
  int ibit_pending;
  uint64_t ibit_due_ms;

  // --- SELFLEV tracking
  int selflev_pending;
  uint64_t selflev_due_ms;
} shared_data_t;

typedef struct {
  pthread_mutex_t mtx;
  shared_data_t d;
} shared_t;

static void tx_timing_update(tx_timing_stats_t* stats,
                             uint64_t expected_wake_ns,
                             uint64_t actual_wake_ns,
                             uint64_t send_complete_ns,
                             long period_ns) {
  if (!stats) return;
  const uint64_t jitter_ns = abs_diff_u64(actual_wake_ns, expected_wake_ns);
  const uint64_t sec_idx = actual_wake_ns / 1000000000ULL;

  stats->sample_count++;
  stats->expected_wake_ns = expected_wake_ns;
  stats->actual_wake_ns = actual_wake_ns;
  stats->send_complete_ns = send_complete_ns;
  if (period_ns > 0 && actual_wake_ns > expected_wake_ns &&
      (actual_wake_ns - expected_wake_ns) > (uint64_t)period_ns) {
    stats->deadline_miss_count++;
  }

  if (stats->current_sec_idx != sec_idx) {
    if (stats->current_sec_samples > 0) {
      stats->last_sec_samples = stats->current_sec_samples;
      stats->last_sec_jitter_min_ns = stats->current_sec_jitter_min_ns;
      stats->last_sec_jitter_max_ns = stats->current_sec_jitter_max_ns;
      stats->last_sec_jitter_avg_ns = stats->current_sec_jitter_sum_ns / stats->current_sec_samples;
    }
    stats->current_sec_idx = sec_idx;
    stats->current_sec_samples = 0;
    stats->current_sec_jitter_sum_ns = 0;
    stats->current_sec_jitter_min_ns = UINT64_MAX;
    stats->current_sec_jitter_max_ns = 0;
  }

  stats->current_sec_samples++;
  stats->current_sec_jitter_sum_ns += jitter_ns;
  if (jitter_ns < stats->current_sec_jitter_min_ns) stats->current_sec_jitter_min_ns = jitter_ns;
  if (jitter_ns > stats->current_sec_jitter_max_ns) stats->current_sec_jitter_max_ns = jitter_ns;
}

static void cmd_timing_update(cmd_timing_stats_t* stats,
                              uint64_t tcp_recv_ns,
                              uint64_t action_queued_ns,
                              uint64_t exec_start_ns) {
  if (!stats) return;
  if (!(tcp_recv_ns > 0 &&
        action_queued_ns >= tcp_recv_ns &&
        exec_start_ns >= action_queued_ns)) {
    stats->invalid_samples++;
    return;
  }

  const uint64_t tcp_to_queue_ns = (action_queued_ns >= tcp_recv_ns) ? (action_queued_ns - tcp_recv_ns) : 0;
  const uint64_t queue_to_exec_ns = (exec_start_ns >= action_queued_ns) ? (exec_start_ns - action_queued_ns) : 0;
  const uint64_t end_to_end_ns = (exec_start_ns >= tcp_recv_ns) ? (exec_start_ns - tcp_recv_ns) : 0;

  stats->samples++;
  stats->tcp_recv_ns = tcp_recv_ns;
  stats->action_queued_ns = action_queued_ns;
  stats->exec_start_ns = exec_start_ns;
  stats->tcp_to_queue_avg_ns =
    (uint64_t)((((__uint128_t)stats->tcp_to_queue_avg_ns * (stats->samples - 1)) + tcp_to_queue_ns) / stats->samples);
  stats->queue_to_exec_avg_ns =
    (uint64_t)((((__uint128_t)stats->queue_to_exec_avg_ns * (stats->samples - 1)) + queue_to_exec_ns) / stats->samples);
  stats->end_to_end_avg_ns =
    (uint64_t)((((__uint128_t)stats->end_to_end_avg_ns * (stats->samples - 1)) + end_to_end_ns) / stats->samples);
}

static void observability_emit_periodic_log(shared_t* sh, uint64_t now_ms);
static int sut_index_from_laser_id(const shared_data_t* sh, uint8_t laser_id);
static int command_targets_unmapped_axis(const shared_data_t* sh, const stc_icd_cmd_t* cmd);

/* ----------------------------- signal handling ----------------------------- */

static void on_sig(int s) {
  (void)s;
  atomic_store(&g_stop, 1);
  int listener_fd = atomic_load(&g_listener_fd);
  if (listener_fd >= 0) {
    close(listener_fd);
    atomic_store(&g_listener_fd, -1);
  }
  int client_fd = atomic_load(&g_client_fd);
  if (client_fd >= 0) {
    close(client_fd);
    atomic_store(&g_client_fd, -1);
  }
}

/* ----------------------------- ICD validation ----------------------------- */

static int is_allowed_controller(uint8_t cid, const shared_data_t* sh) {
  const int strict_legacy = (sh != NULL) ? (sh->controller_id_strict_legacy != 0) : 0;

  /*
   * Effective policy for CMD.controller_id:
   *  - Compatibility mode (default): accept 0x01 to support legacy/bridged producers,
   *    plus legacy ICD IDs {11, 13, 15, 21}.
   *  - Strict legacy mode: accept only the ICD legacy IDs {11, 13, 15, 21}.
   *
   * This keeps GUI/tool producers and server-side validation aligned, while allowing
   * integrators to tighten validation through config when required.
   */
  if (cid == 11 || cid == 13 || cid == 15 || cid == 21) return 1;
  if (!strict_legacy && cid == 0x01) return 1;
  return 0;
}

static stc_nack_err_t validate_cmd_fast(const stc_icd_cfg_t* cfg,
                                       const shared_data_t* sh,
                                       const stc_icd_cmd_t* c) {
  // magic
  if (!(c->magic[0] == cfg->cmd_magic[0] && c->magic[1] == cfg->cmd_magic[1] &&
        c->magic[2] == cfg->cmd_magic[2] && c->magic[3] == cfg->cmd_magic[3])) {
    return STC_ERR_PROTOCOL_MISMATCH;
  }
  if (c->version != cfg->protocol_version_default) return STC_ERR_PROTOCOL_MISMATCH;
  if (!is_allowed_controller(c->controller_id, sh)) return STC_ERR_CLIENT_UNKNOWN;

  // known cmd_id set
  switch (c->cmd_id) {
    case STC_CMD_CU_STATE:
    case STC_CMD_SELFLEV:
    case STC_CMD_IBIT:
    case STC_CMD_STORAGE:
    case STC_CMD_LASER:
    case STC_CMD_TURRET:
    case STC_CMD_MIRROR:
    case STC_CMD_SUT:
    case STC_CMD_RESET:
    case STC_CMD_DELLOG:
    case STC_CMD_FACTORY:
    case STC_CMD_SELFTEST:
    case STC_CMD_SHUTDOWN:
    case STC_CMD_CONFIG:
    case STC_CMD_MANUALPOS:
    case STC_CMD_POSMODE:
    case STC_CMD_REQ_SW_VERSION:
    case STC_CMD_REQ_HW_INFO:
    case STC_CMD_REQ_WORK_TIME:
      break;
    default:
      return STC_ERR_COMMAND_NOT_IMPLEMENTED;
  }

  // payload decoding must be consistent (for known payload-bearing commands)
  if (c->payload_size > 0 && c->payload_decoded == 0) {
    return STC_ERR_SYNTAX_ERROR;
  }

  if (c->cmd_id == STC_CMD_TURRET) {
    if (c->u.turret.motion_type > STC_MOTION_SPEED) return STC_ERR_VALUE_OUT_OF_RANGE;
  }
  if (c->cmd_id == STC_CMD_SUT) {
    if (c->u.sut.motion_type > STC_MOTION_SPEED) return STC_ERR_VALUE_OUT_OF_RANGE;
    if (c->u.sut.id != 0xFF && c->u.sut.id < 1) return STC_ERR_VALUE_OUT_OF_RANGE;
  }
  if (c->cmd_id == STC_CMD_MIRROR) {
    if (c->u.mirror.motion_type > STC_MOTION_SPEED) return STC_ERR_VALUE_OUT_OF_RANGE;
    if (c->u.mirror.mirror_id != 0xFF && c->u.mirror.mirror_id < 1) return STC_ERR_VALUE_OUT_OF_RANGE;
  }

  /* Resolve target mapping before state gating so unmapped bench targets NACK
   * as NOT_CONFIGURED regardless of SAFE/OPER state. */
  if (c->payload_decoded &&
      (c->cmd_id == STC_CMD_TURRET || c->cmd_id == STC_CMD_SUT || c->cmd_id == STC_CMD_MIRROR) &&
      command_targets_unmapped_axis(sh, c)) {
    return STC_ERR_NOT_CONFIGURED;
  }
  if (c->cmd_id == STC_CMD_LASER) {
    if (sut_index_from_laser_id(sh, c->u.laser.id) < 0) return STC_ERR_VALUE_OUT_OF_RANGE;
    if (c->u.laser.on_off > 1) return STC_ERR_VALUE_OUT_OF_RANGE;
  }
  if (c->cmd_id == STC_CMD_MANUALPOS || c->cmd_id == STC_CMD_POSMODE) {
    if (sh->st_state != STC_STATE_SAFE && sh->st_state != STC_STATE_OPER) {
      return STC_ERR_NOT_ALLOWED_STATE;
    }
  }

  return STC_ERR_NO_ERROR;
}


static int shared_transition_locked(shared_t* sh, stc_event_t evt) {
  const stc_state_t prev = sh->d.st_state;
  stc_state_t next = sh->d.st_state;
  if (stc_state_transition(sh->d.st_state, evt, &next) != 0) return -1;
  sh->d.st_state = next;
  if (prev != next) {
    STC_LOG_INFO("state transition: %s -> %s",
                 stc_state_to_string(prev),
                 stc_state_to_string(next));
  }
  if (next == STC_STATE_FAIL && prev != STC_STATE_FAIL) {
    uint32_t mask = 0;
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      if (sh->d.axis_expected[axis] ||
          sh->d.axis_available[axis] ||
          sh->d.axis_enabled[axis] ||
          sh->d.stream[axis].enabled) {
        mask |= (1u << axis);
      }
    }
    sh->d.fail_stop_pending = 1;
    sh->d.fail_stop_axes_mask = mask;
  }
  if (evt == EVT_ENTER_SAFE) {
    sh->d.ibit_pending = 0;
    sh->d.ibit_due_ms = 0;
    sh->d.selflev_pending = 0;
    sh->d.selflev_due_ms = 0;
  }
  if (!stc_state_can_stream(sh->d.st_state)) {
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      sh->d.stream[axis].enabled = 0;
    }
  }
  return 0;
}

static int shared_start_ibit_locked(shared_t* sh, uint64_t now_ms) {
  if (shared_transition_locked(sh, EVT_IBIT) != 0) return -1;
  sh->d.ibit_pending = 1;
  sh->d.ibit_due_ms = now_ms + 100;
  return 0;
}

static stc_event_t ibit_completion_event_locked(const shared_data_t* d) {
  int expected_axes = 0;
  int available_axes = 0;
  int faulted_axes = 0;

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!d->axis_expected[axis]) continue;
    expected_axes++;
    if (d->axis_available[axis]) available_axes++;
    if (d->axis_fault_active[axis]) faulted_axes++;
  }

  const int pass =
    (expected_axes > 0) &&
    d->can_ok &&
    (available_axes == expected_axes) &&
    (faulted_axes == 0);

  stc_diag_set_ibit_result(pass ? true : false);
  return run_ibit_sequence((shared_data_t*)d) ? EVT_IBIT_COMPLETE_OK : EVT_IBIT_COMPLETE_FAIL;
}

static stc_event_t selflev_completion_event_locked(const shared_data_t* d) {
  if (!d) return EVT_ENTER_SAFE;
  if (!d->can_ok) return EVT_ENTER_SAFE;
  return EVT_SELF_LEV_DONE;
}

static void mark_axis_saturated(shared_data_t* sh, int axis, int limited) {
  if (!limited) return;
  if (axis < 0 || axis >= (int)STC_AXIS_COUNT) return;
  sh->saturated[axis] = true;
  sh->saturated_until_ms[axis] = mono_ms() + STC_DIAG_LATCH_MS;
}


static void log_axis_abs_saturation_if_needed(int axis,
                                              double requested_abs,
                                              double clamped_abs,
                                              const axis_cfg_t* cfg,
                                              int was_limited) {
  if (!was_limited || !cfg) return;
  STC_LOG_WARN("axis%d command saturated: requested_abs=%.3f clamped_abs=%.3f min=%.3f max=%.3f",
               axis,
               requested_abs,
               clamped_abs,
               cfg->az_min_deg,
               cfg->az_max_deg);
}

static void log_axis_final_status(shared_t* sh, int axis) {
  if (!sh) return;
  if (axis < 0 || axis >= (int)STC_AXIS_COUNT) return;
  pthread_mutex_lock(&sh->mtx);
  const axis_telem_t telem = sh->d.telem[axis];
  const int stale = (sh->d.can_timeout_ms > 0 &&
                     elapsed_ms_nonnegative(mono_ms(), sh->d.last_rx_ms[axis]) > sh->d.can_timeout_ms) ? 1 : 0;
  pthread_mutex_unlock(&sh->mtx);
  STC_LOG_INFO("axis%d status pos=%.3f deg vel=%.3f deg/s brake=%u err_flags=0x%04X stale=%d",
               axis,
               telem.pos_deg,
               telem.vel_deg_s,
               0u,
               (unsigned)(telem.status_word & 0xFFFFu),
               stale);
}

static int axis_in_runtime_range(const shared_data_t* sh, int axis) {
  if (!sh) return 0;
  if (axis < 0 || axis >= (int)STC_AXIS_COUNT) return 0;
  return axis < sh->num_axes;
}

static int axis_is_operational(const shared_data_t* sh, int axis) {
  if (!axis_in_runtime_range(sh, axis)) return 0;
  if (!sh->axis_expected[axis]) return 0;
  if (!sh->axis_available[axis]) return 0;
  if (sh->axis_fault_active[axis]) return 0;
  return 1;
}

static int sut_index_from_laser_id(const shared_data_t* sh, uint8_t laser_id) {
  if (!sh) return -1;
  if (laser_id < 1) return -1;
  const int idx = (int)laser_id - 1;
  if (idx < 0 || idx >= sh->sut_count) return -1;
  return idx;
}

static uint32_t turret_axes_mask(const shared_data_t* sh) {
  uint32_t mask = 0;
  if (sh->turret_az_axis >= 0 && sh->turret_az_axis < (int)STC_AXIS_COUNT) mask |= (1u << sh->turret_az_axis);
  if (sh->turret_el_axis >= 0 && sh->turret_el_axis < (int)STC_AXIS_COUNT) mask |= (1u << sh->turret_el_axis);
  return mask;
}

static uint32_t sut_axes_mask(const shared_data_t* sh, uint8_t id) {
  uint32_t mask = 0;
  if (id == 0xFF) {
    for (int i = 0; i < sh->sut_count; i++) {
      int az = sh->sut_az_axis[i];
      int el = sh->sut_el_axis[i];
      if (axis_is_operational(sh, az)) mask |= (1u << az);
      if (axis_is_operational(sh, el)) mask |= (1u << el);
    }
    return mask;
  }

  int idx = (int)id - 1;
  if (idx < 0 || idx >= sh->sut_count) return 0;
  if (sh->sut_az_axis[idx] >= 0 && sh->sut_az_axis[idx] < (int)STC_AXIS_COUNT) mask |= (1u << sh->sut_az_axis[idx]);
  if (sh->sut_el_axis[idx] >= 0 && sh->sut_el_axis[idx] < (int)STC_AXIS_COUNT) mask |= (1u << sh->sut_el_axis[idx]);
  return mask;
}

static uint32_t mirror_axes_mask(const shared_data_t* sh, uint8_t id) {
  uint32_t mask = 0;
  if (id == 0xFF) {
    for (int i = 0; i < sh->mirror_count; i++) {
      int axis = sh->mirror_axis[i];
      if (axis_is_operational(sh, axis)) mask |= (1u << axis);
    }
    return mask;
  }

  int idx = (int)id - 1;
  if (idx < 0 || idx >= sh->mirror_count) return 0;
  if (sh->mirror_axis[idx] >= 0 && sh->mirror_axis[idx] < (int)STC_AXIS_COUNT) mask |= (1u << sh->mirror_axis[idx]);
  return mask;
}

static int command_targets_unmapped_axis(const shared_data_t* sh, const stc_icd_cmd_t* cmd) {
  if (cmd->cmd_id == STC_CMD_TURRET) {
    return (sh->turret_az_axis < 0 || sh->turret_el_axis < 0);
  }
  if (cmd->cmd_id == STC_CMD_SUT) {
    if (cmd->u.sut.id == 0xFF) {
      for (int i = 0; i < sh->sut_count; i++) {
        if (sh->sut_az_axis[i] < 0 || sh->sut_el_axis[i] < 0) return 1;
      }
      return 0;
    }
    const int idx = (int)cmd->u.sut.id - 1;
    if (idx < 0 || idx >= sh->sut_count) return 0;
    return (sh->sut_az_axis[idx] < 0 || sh->sut_el_axis[idx] < 0);
  }
  if (cmd->cmd_id == STC_CMD_MIRROR) {
    if (cmd->u.mirror.mirror_id == 0xFF) {
      for (int i = 0; i < sh->mirror_count; i++) {
        if (sh->mirror_axis[i] < 0) return 1;
      }
      return 0;
    }
    const int idx = (int)cmd->u.mirror.mirror_id - 1;
    if (idx < 0 || idx >= sh->mirror_count) return 0;
    return (sh->mirror_axis[idx] < 0);
  }
  return 0;
}

static void print_cmd(const stc_icd_cmd_t* c) {
  STC_LOG_DEBUG("CMD: ver=%u ctrl=%u cmd_id=0x%04X seq=%u ts=%llu payload=%u",
         c->version, c->controller_id, c->cmd_id, c->seq,
         (unsigned long long)c->timestamp_ms, c->payload_size);

  if (!c->payload_decoded) {
    if (c->payload_size) STC_LOG_DEBUG("  payload: (raw %u bytes)", c->payload_size);
    return;
  }
  switch (c->cmd_id) {
    case STC_CMD_CU_STATE:
      STC_LOG_DEBUG("  CU_STATE=%u", c->u.cu_state.state);
      break;
    case STC_CMD_SELFLEV:
      STC_LOG_DEBUG("  SELFLEV start_stop=%u", c->u.selflev.v);
      break;
    case STC_CMD_IBIT:
      STC_LOG_DEBUG("  IBIT start_stop=%u", c->u.ibit.v);
      break;
    case STC_CMD_STORAGE:
      STC_LOG_DEBUG("  STORAGE");
      break;
    case STC_CMD_LASER:
      STC_LOG_DEBUG("  LASER id=%u on_off=%u", c->u.laser.id, c->u.laser.on_off);
      break;
    case STC_CMD_TURRET:
      STC_LOG_DEBUG("  TURRET motion=%u az=%f el=%f", c->u.turret.motion_type, c->u.turret.az, c->u.turret.el);
      break;
    case STC_CMD_MIRROR:
      STC_LOG_DEBUG("  MIRROR id=%u motion=%u elev=%f", c->u.mirror.mirror_id, c->u.mirror.motion_type, c->u.mirror.elevation);
      break;
    case STC_CMD_SUT:
      STC_LOG_DEBUG("  SUT id=%u motion=%u az=%f el=%f", c->u.sut.id, c->u.sut.motion_type, c->u.sut.az, c->u.sut.el);
      break;
    case STC_CMD_MANUALPOS:
      STC_LOG_DEBUG("  MANUALPOS lat=%lf lon=%lf alt=%lf heading=%lf (encoded_as_double=%d)",
             c->u.manualpos.lat, c->u.manualpos.lon, c->u.manualpos.alt, c->u.manualpos.heading,
             c->u.manualpos.encoded_as_double);
      break;
    case STC_CMD_POSMODE:
      STC_LOG_DEBUG("  POSMODE mode=%u", c->u.posmode.v);
      break;
    case STC_CMD_REQ_SW_VERSION:
      STC_LOG_DEBUG("  REQ_SW_VERSION");
      break;
    case STC_CMD_RESET:
      STC_LOG_DEBUG("  RESET");
      break;
    case STC_CMD_DELLOG:
      STC_LOG_DEBUG("  DELLOG");
      break;
    case STC_CMD_FACTORY:
      STC_LOG_DEBUG("  FACTORY");
      break;
    case STC_CMD_SELFTEST:
      STC_LOG_DEBUG("  SELFTEST");
      break;
    case STC_CMD_SHUTDOWN:
      STC_LOG_DEBUG("  SHUTDOWN");
      break;
    default:
      STC_LOG_DEBUG("  (decoded)");
      break;
  }
}

/* ----------------------------- CAN worker ----------------------------- */


typedef struct {
  int rt_enabled;
  int rt_strict;
  int sched_policy;
  int can_worker_priority;
  int tlmh_tx_priority;
  int tlml_tx_priority;
} runtime_sched_t;

typedef struct {
  int ok;
  int err;
  int requested_priority;
} rt_probe_result_t;

static const char* sched_policy_name(int policy) {
  switch (policy) {
    case SCHED_FIFO: return "SCHED_FIFO";
    case SCHED_RR: return "SCHED_RR";
    case SCHED_OTHER: return "SCHED_OTHER";
    default: return "SCHED_UNKNOWN";
  }
}

static int parse_sched_policy_cli(const char* value, int* out_policy) {
  if (!value || !out_policy) return -1;
  if (!strcmp(value, "fifo")) {
    *out_policy = SCHED_FIFO;
  } else if (!strcmp(value, "rr")) {
    *out_policy = SCHED_RR;
  } else if (!strcmp(value, "other")) {
    *out_policy = SCHED_OTHER;
  } else {
    return -1;
  }
  return 0;
}

static int clamp_sched_priority(int policy, int requested_priority) {
  int max_prio = sched_get_priority_max(policy);
  int min_prio = sched_get_priority_min(policy);
  if (max_prio < min_prio) return requested_priority;
  if (requested_priority < min_prio) return min_prio;
  if (requested_priority > max_prio) return max_prio;
  return requested_priority;
}

static void log_rt_remediation(const char* binary_hint) {
  const char* bin = (binary_hint && binary_hint[0]) ? binary_hint : "<binary>";
  STC_LOG_WARN("RT privilege remediation options:");
  STC_LOG_WARN("  1) grant CAP_SYS_NICE: sudo setcap cap_sys_nice=ep %s", bin);
  STC_LOG_WARN("  2) systemd unit: AmbientCapabilities=CAP_SYS_NICE and LimitRTPRIO=<priority>");
  STC_LOG_WARN("  3) run as root where local policy allows");
}

static rt_probe_result_t probe_rt_privilege(int policy, int requested_priority) {
  rt_probe_result_t r;
  memset(&r, 0, sizeof(r));
  r.ok = 0;
  r.err = EINVAL;
  r.requested_priority = clamp_sched_priority(policy, requested_priority);

  int old_policy = 0;
  struct sched_param old_sp;
  memset(&old_sp, 0, sizeof(old_sp));
  int rc = pthread_getschedparam(pthread_self(), &old_policy, &old_sp);
  if (rc != 0) {
    r.err = rc;
    return r;
  }

  struct sched_param probe_sp;
  memset(&probe_sp, 0, sizeof(probe_sp));
  probe_sp.sched_priority = r.requested_priority;
  rc = pthread_setschedparam(pthread_self(), policy, &probe_sp);
  if (rc != 0) {
    r.err = rc;
    return r;
  }

  rc = pthread_setschedparam(pthread_self(), old_policy, &old_sp);
  if (rc != 0) {
    r.err = rc;
    return r;
  }

  r.ok = 1;
  r.err = 0;
  return r;
}

static int enforce_rt_startup_policy(runtime_sched_t* sched_cfg,
                                     const char* binary_hint) {
  if (!sched_cfg) return 0;
  if (!sched_cfg->rt_enabled) {
    STC_LOG_INFO("RT scheduling disabled by config.");
    return 0;
  }
  if (sched_cfg->sched_policy == SCHED_OTHER) {
    STC_LOG_INFO("RT startup probe skipped: requested policy=%s", sched_policy_name(sched_cfg->sched_policy));
    return 0;
  }

  int highest_requested = sched_cfg->can_worker_priority;
  if (sched_cfg->tlmh_tx_priority > highest_requested) highest_requested = sched_cfg->tlmh_tx_priority;
  if (sched_cfg->tlml_tx_priority > highest_requested) highest_requested = sched_cfg->tlml_tx_priority;

  rt_probe_result_t probe = probe_rt_privilege(sched_cfg->sched_policy, highest_requested);
  if (probe.ok) {
    STC_LOG_INFO("RT startup probe succeeded: policy=%s priority=%d", sched_policy_name(sched_cfg->sched_policy), probe.requested_priority);
    return 0;
  }

  STC_LOG_ERROR("RT startup probe failed: policy=%s priority=%d err=%s",
                sched_policy_name(sched_cfg->sched_policy),
                probe.requested_priority,
                strerror(probe.err));
  log_rt_remediation(binary_hint);

  if (sched_cfg->rt_strict) {
    STC_LOG_ERROR("strict RT mode enabled (--rt-strict=1): refusing to start without RT privileges");
    return -1;
  }

  STC_LOG_WARN("non-strict RT mode (--rt-strict=0): continuing with RT disabled for deterministic startup behavior");
  sched_cfg->rt_enabled = 0;
  return 0;
}

static const char* resolve_binary_hint(char* out, size_t out_len, const char* argv0) {
  if (!out || out_len == 0) return NULL;
  out[0] = '\0';
  if (!argv0 || !argv0[0]) return NULL;
  if (realpath(argv0, out)) return out;
  snprintf(out, out_len, "%s", argv0);
  return out;
}

static int apply_thread_sched_attr(pthread_attr_t* attr,
                                   int policy,
                                   int priority,
                                   const char* thread_name) {
  int rc = pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
  if (rc != 0) {
    STC_LOG_ERROR("%s: pthread_attr_setinheritsched(PTHREAD_EXPLICIT_SCHED) failed: %s", thread_name, strerror(rc));
    return rc;
  }

  rc = pthread_attr_setschedpolicy(attr, policy);
  if (rc != 0) {
    STC_LOG_ERROR("%s: pthread_attr_setschedpolicy(%s) failed: %s", thread_name, sched_policy_name(policy), strerror(rc));
    return rc;
  }

  struct sched_param sp;
  memset(&sp, 0, sizeof(sp));
  sp.sched_priority = priority;
  rc = pthread_attr_setschedparam(attr, &sp);
  if (rc != 0) {
    STC_LOG_ERROR("%s: pthread_attr_setschedparam(priority=%d) failed: %s", thread_name, priority, strerror(rc));
    return rc;
  }

  return 0;
}

static void log_thread_effective_sched(const char* thread_name, pthread_t thread) {
  int policy = 0;
  struct sched_param sp;
  memset(&sp, 0, sizeof(sp));
  int rc = pthread_getschedparam(thread, &policy, &sp);
  if (rc != 0) {
    STC_LOG_WARN("%s: pthread_getschedparam failed: %s", thread_name, strerror(rc));
    return;
  }
  STC_LOG_INFO("%s effective scheduling: policy=%s priority=%d", thread_name, sched_policy_name(policy), sp.sched_priority);
}

static int create_thread_with_sched(pthread_t* thread,
                                    const char* thread_name,
                                    void* (*thread_fn)(void*),
                                    void* arg,
                                    const runtime_sched_t* sched_cfg,
                                    int priority) {
  if (!thread || !thread_name || !thread_fn || !sched_cfg) return EINVAL;

  if (!sched_cfg->rt_enabled) {
    int rc = pthread_create(thread, NULL, thread_fn, arg);
    if (rc == 0) {
      STC_LOG_INFO("%s scheduling request: RT disabled, using default pthread attrs", thread_name);
      log_thread_effective_sched(thread_name, *thread);
    }
    return rc;
  }

  pthread_attr_t attr;
  int rc = pthread_attr_init(&attr);
  if (rc != 0) return rc;

  int requested_prio = clamp_sched_priority(sched_cfg->sched_policy, priority);

  STC_LOG_INFO("%s scheduling request: policy=%s priority=%d strict_rt=%d", thread_name, sched_policy_name(sched_cfg->sched_policy), requested_prio, sched_cfg->rt_strict);

  rc = apply_thread_sched_attr(&attr, sched_cfg->sched_policy, requested_prio, thread_name);
  if (rc != 0) {
    pthread_attr_destroy(&attr);
    return rc;
  }

  rc = pthread_create(thread, &attr, thread_fn, arg);
  pthread_attr_destroy(&attr);
  if (rc == 0) {
    log_thread_effective_sched(thread_name, *thread);
    return 0;
  }

  if ((rc == EPERM || rc == EINVAL) && !sched_cfg->rt_strict) {
    STC_LOG_WARN("%s RT thread create failed (%s, policy=%s priority=%d); falling back to SCHED_OTHER",
                 thread_name,
                 strerror(rc),
                 sched_policy_name(sched_cfg->sched_policy),
                 requested_prio);
    pthread_attr_t fb_attr;
    int fb_rc = pthread_attr_init(&fb_attr);
    if (fb_rc != 0) return fb_rc;

    fb_rc = apply_thread_sched_attr(&fb_attr, SCHED_OTHER, 0, thread_name);
    if (fb_rc != 0) {
      pthread_attr_destroy(&fb_attr);
      return rc;
    }

    fb_rc = pthread_create(thread, &fb_attr, thread_fn, arg);
    pthread_attr_destroy(&fb_attr);
    if (fb_rc == 0) {
      log_thread_effective_sched(thread_name, *thread);
      return 0;
    }
    return fb_rc;
  }

  return rc;
}

typedef struct {
  const char* can_if;
  int can_bitrate;
  int can_restart_ms;
  int can_scan_max_node;
  const char* backend_name;
  int primary_node_id;
  int primary_tx_id;
  int primary_rx_id;
  int primary_motion_tx_id;
  int primary_motion_rx_id;
  double travel_min_deg;
  double travel_max_deg;
  double profile_max_speed_deg_s;
  double profile_max_accel_deg_s2;
  int telemetry_interval_ms;
  int experimental_native_speed_mode;
  shared_t* sh;
  actionq_t* q;
} can_worker_arg_t;



#ifdef STC_TEST_HOOKS
_Atomic uint64_t g_can_worker_action_dispatch_iterations = 0;
#endif

static void* can_worker_thread(void* arg) {
  can_worker_arg_t* a = (can_worker_arg_t*)arg;
  const uint64_t maintenance_period_ms = 10;
  uint64_t speed_stream_period_ms = 50;
  uint64_t next_maintenance_ms = mono_ms();

  motion_backend_cfg_t backend_cfg;
  memset(&backend_cfg, 0, sizeof(backend_cfg));
  backend_cfg.can_if = a->can_if;
  snprintf(backend_cfg.backend, sizeof(backend_cfg.backend), "%s", a->backend_name);
  backend_cfg.primary_node_id = (uint8_t)a->primary_node_id;
  backend_cfg.primary_tx_id = (uint32_t)a->primary_tx_id;
  backend_cfg.primary_rx_id = (uint32_t)a->primary_rx_id;
  backend_cfg.primary_motion_tx_id = (uint32_t)a->primary_motion_tx_id;
  backend_cfg.primary_motion_rx_id = (uint32_t)a->primary_motion_rx_id;
  backend_cfg.travel_min_deg = a->travel_min_deg;
  backend_cfg.travel_max_deg = a->travel_max_deg;
  backend_cfg.profile_max_speed_deg_s = a->profile_max_speed_deg_s;
  backend_cfg.profile_max_accel_deg_s2 = a->profile_max_accel_deg_s2;
  backend_cfg.telemetry_interval_ms = a->telemetry_interval_ms;
  backend_cfg.experimental_native_speed_mode = a->experimental_native_speed_mode;
  backend_cfg.can_bitrate = a->can_bitrate;
  backend_cfg.can_restart_ms = a->can_restart_ms;
  backend_cfg.can_scan_max_node = a->can_scan_max_node;

  pthread_mutex_lock(&a->sh->mtx);
  backend_cfg.speed_watchdog_ms = a->sh->d.speed_watchdog_ms;
  backend_cfg.position_tolerance_deg = a->sh->d.position_tolerance_deg;
  backend_cfg.position_timeout_ms = a->sh->d.position_timeout_ms;
  backend_cfg.position_poll_ms = a->sh->d.position_poll_ms;
  backend_cfg.ds402_timeout_ms = a->sh->d.ds402_timeout_ms;
  backend_cfg.pp_sdo_diag = a->sh->d.pp_sdo_diag;
  backend_cfg.can_timeout_ms = a->sh->d.can_timeout_ms;
  memcpy(backend_cfg.axis_cfg, a->sh->d.cfg, sizeof(backend_cfg.axis_cfg));
  memcpy(backend_cfg.axis_node_id, a->sh->d.axis_node_id, sizeof(backend_cfg.axis_node_id));
  pthread_mutex_unlock(&a->sh->mtx);

  motion_backend_t* backend = NULL;
  if (motion_backend_start(&backend_cfg, &backend) != 0 || !backend) {
    STC_LOG_ERROR("motion_backend_start failed");
    return NULL;
  }

  (void)motion_backend_discover(backend);

  int axis_enabled[STC_AXIS_COUNT] = {0};
  uint8_t speed_stream_active[STC_AXIS_COUNT] = {0};
  double speed_stream_target_deg_s[STC_AXIS_COUNT] = {0.0};
  uint64_t next_speed_send_ms[STC_AXIS_COUNT] = {0};
  uint64_t speed_stream_start_ms[STC_AXIS_COUNT] = {0};
  uint32_t speed_stream_tx_count[STC_AXIS_COUNT] = {0};
  uint32_t speed_stream_fail_count[STC_AXIS_COUNT] = {0};
  uint32_t speed_stream_consecutive_failures[STC_AXIS_COUNT] = {0};
  uint64_t speed_stream_last_fail_log_ms[STC_AXIS_COUNT] = {0};

  while (!atomic_load(&g_stop)) {
    uint64_t now = mono_ms();

    if (now < next_maintenance_ms) {
      usleep(1000);
      continue;
    }

    motion_snapshot_t snap;
    if (motion_backend_get_snapshot(backend, &snap) == 0) {
      pthread_mutex_lock(&a->sh->mtx);
      a->sh->d.can_ok = snap.can_ok;
      a->sh->d.can_connected_devices = snap.can_connected_devices;
      a->sh->d.can_discovered_devices = snap.can_discovered_devices;
      a->sh->d.backend_mode = snap.backend_mode;
      a->sh->d.transport_status = snap.transport_status;
      a->sh->d.supports_native_a2_speed = snap.capabilities.supports_native_a2_speed;
      a->sh->d.supports_native_absolute_position = snap.capabilities.supports_native_absolute_position;
      memcpy(a->sh->d.can_discovered_nodes, snap.can_discovered_nodes, sizeof(a->sh->d.can_discovered_nodes));
      for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
        a->sh->d.axis_available[axis] = snap.axis_available[axis];
        uint64_t rx_ms = snap.last_rx_ms[axis];
        if (rx_ms == 0 && a->sh->d.axis_expected[axis]) {
          /*
           * During startup some backends can report "connected/available"
           * before they publish the first feedback timestamp. Treat the
           * current snapshot time as fresh telemetry so CBIT stale detection
           * does not force an immediate FAIL right after successful PBIT.
           */
          rx_ms = now;
        }
        a->sh->d.last_rx_ms[axis] = rx_ms;
        axis_telem_update(&a->sh->d.telem[axis],
                          &a->sh->d.cfg[axis],
                          snap.pos_raw[axis],
                          snap.vel_raw[axis],
                          snap.status_word[axis],
                          snap.mode_display[axis],
                          now);
      }
      a->sh->d.hb_state = snap.hb_state;
      a->sh->d.have_hb = snap.have_hb;
      a->sh->d.have_emcy = snap.have_emcy;
      a->sh->d.emcy_code = snap.emcy_code;
      a->sh->d.emcy_reg = snap.emcy_reg;
      a->sh->d.ts_tpdo1_ms = snap.ts_tpdo1_ms;
      a->sh->d.ts_tpdo2_ms = snap.ts_tpdo2_ms;
      a->sh->d.ts_tpdo3_ms = snap.ts_tpdo3_ms;
      a->sh->d.ts_hb_ms = snap.ts_hb_ms;
      a->sh->d.ts_emcy_ms = snap.ts_emcy_ms;
      pthread_mutex_unlock(&a->sh->mtx);
    }

    pthread_mutex_lock(&a->sh->mtx);
    if (a->sh->d.ibit_pending && now >= a->sh->d.ibit_due_ms) {
      const stc_event_t ibit_evt = ibit_completion_event_locked(&a->sh->d);
      if (shared_transition_locked(a->sh, ibit_evt) == 0) {
        STC_LOG_INFO("IBIT completion event published: event=%d state=%d",
                     (int)ibit_evt,
                     (int)a->sh->d.st_state);
      } else {
        STC_LOG_ERROR("IBIT completion transition rejected: state=%d event=%d",
                      (int)a->sh->d.st_state,
                      (int)ibit_evt);
      }
      a->sh->d.ibit_pending = 0;
      a->sh->d.ibit_due_ms = 0;
    }
    if (a->sh->d.selflev_pending && now >= a->sh->d.selflev_due_ms) {
      const stc_event_t selflev_evt = selflev_completion_event_locked(&a->sh->d);
      if (shared_transition_locked(a->sh, selflev_evt) == 0) {
        STC_LOG_INFO("SELFLEV completion event published: event=%d state=%d",
                     (int)selflev_evt,
                     (int)a->sh->d.st_state);
      } else {
        STC_LOG_ERROR("SELFLEV completion transition rejected: state=%d event=%d",
                      (int)a->sh->d.st_state,
                      (int)selflev_evt);
      }
      a->sh->d.selflev_pending = 0;
      a->sh->d.selflev_due_ms = 0;
    }

    int critical_axis_faults = 0;
    int critical_axis_stale = 0;
    int critical_hb = 0;
    int critical_emcy = 0;
    int any_expected_pd4 = 0;
    const uint32_t can_timeout_ms = a->sh->d.can_timeout_ms;
    const uint32_t startup_feedback_grace_ms = a->sh->d.startup_feedback_grace_ms;
    const int in_startup_grace = stc_is_within_startup_grace(now, a->sh->d.server_start_ms, startup_feedback_grace_ms);
    const int strict_stale_policy = a->sh->d.strict_stale_policy ? 1 : 0;
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      const int expected = a->sh->d.axis_expected[axis] ? 1 : 0;
      const int connected = a->sh->d.axis_available[axis] ? 1 : 0;
      const stc_axis_driver_t driver = a->sh->d.cfg[axis].driver;
      int axis_fault = 0;
      int stale = 0;

      if (expected && driver == STC_AXIS_DRIVER_PD4C) {
        any_expected_pd4 = 1;
        axis_fault = stc_detect_ds402_fault(a->sh->d.telem[axis].status_word) ? 1 : 0;
      } else if (expected && driver == STC_AXIS_DRIVER_MYACT_X8_25) {
        /* MYACT status_word is populated from 0x9A error_flags in backend. */
        axis_fault = (a->sh->d.telem[axis].status_word != 0u) ? 1 : 0;
      }
      if (!in_startup_grace && can_timeout_ms > 0) {
        const int timed_out = elapsed_ms_nonnegative(now, a->sh->d.last_rx_ms[axis]) > can_timeout_ms;
        stale = strict_stale_policy ? (expected && timed_out) : (expected && connected && timed_out);
      }
      a->sh->d.axis_fault_active[axis] = axis_fault ? 1u : 0u;
      if (expected && axis_fault) critical_axis_faults++;
      if (stale) critical_axis_stale++;
    }

    if (a->sh->d.have_hb) {
      if (a->sh->d.hb_state != 0x05u) critical_hb = 1; /* OPERATIONAL */
      if (can_timeout_ms > 0 &&
          elapsed_ms_nonnegative(now, a->sh->d.ts_hb_ms) > can_timeout_ms) {
        critical_hb = 1;
      }
    }
    if (any_expected_pd4 && a->sh->d.have_emcy && a->sh->d.emcy_code != 0) critical_emcy = 1;
    /*
     * Heartbeat is currently treated as diagnostic-only (non-fatal).
     *
     * Rationale:
     * - Some deployed drives run with 0x1017 heartbeat producer time = 0
     *   (no periodic heartbeat), especially during bring-up/bench testing.
     * - Marking heartbeat as fatal forces immediate FAIL even when motion I/O
     *   over PDO/SDO is otherwise healthy.
     *
     * Keep hb assessment/logging for observability, but do not gate
     * critical-fault transition on hb alone.
     */
    const int critical_condition =
      (critical_axis_faults > 0) ||
      (critical_axis_stale > 0) ||
      critical_emcy;
    stc_diag_set_cbit_critical(&a->sh->d, critical_condition ? true : false);
    const stc_diag_health_t cbit_health = run_cbit_cycle(&a->sh->d);
    if (cbit_health == STC_DIAG_HEALTH_CRITICAL_FAULT &&
        a->sh->d.st_state != STC_STATE_FAIL) {
      if (shared_transition_locked(a->sh, EVT_FAULT_CRITICAL) == 0) {
        STC_LOG_ERROR(
          "CBIT critical fault detected -> FAIL: axis_faults=%d axis_stale=%d hb=%d emcy=%d",
          critical_axis_faults, critical_axis_stale, critical_hb, critical_emcy);
      } else {
        STC_LOG_ERROR(
          "CBIT critical fault transition rejected: state=%d axis_faults=%d axis_stale=%d hb=%d emcy=%d",
          (int)a->sh->d.st_state,
          critical_axis_faults, critical_axis_stale, critical_hb, critical_emcy);
      }
    }
    pthread_mutex_unlock(&a->sh->mtx);

    stc_action_t act;
    memset(&act, 0, sizeof(act));
    while (actionq_try_peek(a->q, &act) == 1) {
#ifdef STC_TEST_HOOKS
      atomic_fetch_add(&g_can_worker_action_dispatch_iterations, 1);
#endif
      const uint64_t exec_start_ns = mono_ns();
      if (act.obs_tcp_rx_ns > 0 && act.obs_queued_ns > 0) {
        pthread_mutex_lock(&a->sh->mtx);
        cmd_timing_update(&a->sh->d.cmd_timing, act.obs_tcp_rx_ns, act.obs_queued_ns, exec_start_ns);
        pthread_mutex_unlock(&a->sh->mtx);
      }
      switch (act.type) {
        case ACTION_PP_MOVE:
          (void)motion_backend_apply_pp_move(backend, act.axes_mask, act.u.target_deg);
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            if (act.axes_mask & (1u << axis)) axis_enabled[axis] = 1;
          }
          break;
        case ACTION_MOTION:
          if (motion_backend_apply_motion(backend, act.axes_mask, act.u.motion.kind, act.u.motion.value_deg_or_dps, act.u.motion.speed_limit_deg_s) != 0) {
            STC_LOG_WARN("motion_backend_apply_motion failed; command dropped without PP fallback");
          }
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            if (act.axes_mask & (1u << axis)) axis_enabled[axis] = 1;
          }
          break;
        case ACTION_STREAM_SPEED:
          (void)motion_backend_apply_speed(backend, act.axes_mask, act.u.speed_deg_s);
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            if (act.axes_mask & (1u << axis)) axis_enabled[axis] = 1;
          }
          break;
        case ACTION_STOP:
        case ACTION_SHUTDOWN:
          (void)motion_backend_quick_stop(backend, act.axes_mask);
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            if (act.axes_mask & (1u << axis)) {
              axis_enabled[axis] = 0;
              pthread_mutex_lock(&a->sh->mtx);
              a->sh->d.stream[axis].enabled = 0;
              pthread_mutex_unlock(&a->sh->mtx);
            }
          }
          break;
        case ACTION_CAN_REDISCOVER: {
          STC_LOG_INFO("CAN rediscovery requested by client");
          const int disc_rc = motion_backend_discover(backend);
          if (disc_rc == 0) {
            STC_LOG_INFO("CAN rediscovery completed successfully");
          } else {
            STC_LOG_WARN("CAN rediscovery failed (rc=%d)", disc_rc);
          }
          break;
        }
        case ACTION_CLEAR_ERRORS:
          (void)motion_backend_clear_faults(backend, act.axes_mask);
          break;
        case ACTION_RESET:
          (void)motion_backend_quick_stop(backend, act.axes_mask);
          memset(axis_enabled, 0, sizeof(axis_enabled));
          break;
        case ACTION_SET_LASER: {
          const uint8_t laser_id = act.u.laser.laser_id;
          const uint8_t on_off = act.u.laser.on_off ? 1u : 0u;
          pthread_mutex_lock(&a->sh->mtx);
          const int sut_idx = sut_index_from_laser_id(&a->sh->d, laser_id);
          if (sut_idx >= 0) {
            a->sh->d.sut_laser_on[sut_idx] = on_off;
          }
          pthread_mutex_unlock(&a->sh->mtx);
          STC_LOG_WARN("LASER stub: SUT%u requested %s; hardware backend not present",
                       (unsigned)laser_id,
                       on_off ? "ON" : "OFF");
          (void)backend;
          break;
        }
        case ACTION_ENTER_STORE: {
          uint32_t storage_axes_mask = 0;
          double storage_targets[STC_AXIS_COUNT] = {0};
          pthread_mutex_lock(&a->sh->mtx);
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            storage_targets[axis] = a->sh->d.storage_deg[axis];
            if (a->sh->d.axis_available[axis]) storage_axes_mask |= (1u << axis);
          }
          pthread_mutex_unlock(&a->sh->mtx);

          if (storage_axes_mask == 0) {
            pthread_mutex_lock(&a->sh->mtx);
            if (shared_transition_locked(a->sh, EVT_STORE_FAIL) != 0) {
              (void)shared_transition_locked(a->sh, EVT_FAULT_CRITICAL);
            }
            pthread_mutex_unlock(&a->sh->mtx);
            STC_LOG_ERROR("STORE command failed: no connected axes available");
            break;
          }

          const int store_rc = motion_backend_apply_pp_move(backend, storage_axes_mask, storage_targets);
          pthread_mutex_lock(&a->sh->mtx);
          if (store_rc == 0) {
            if (shared_transition_locked(a->sh, EVT_STORE_OK) != 0) {
              STC_LOG_ERROR("STORE success transition rejected: state=%d event=%d",
                            (int)a->sh->d.st_state,
                            (int)EVT_STORE_OK);
              (void)shared_transition_locked(a->sh, EVT_FAULT_CRITICAL);
            } else {
              STC_LOG_INFO("STORE command completed successfully: axes_mask=0x%08x", storage_axes_mask);
            }
          } else {
            if (shared_transition_locked(a->sh, EVT_STORE_FAIL) != 0) {
              STC_LOG_ERROR("STORE fail transition rejected: state=%d event=%d",
                            (int)a->sh->d.st_state,
                            (int)EVT_STORE_FAIL);
              (void)shared_transition_locked(a->sh, EVT_FAULT_CRITICAL);
            }
            STC_LOG_ERROR("STORE command failed: motion_backend_apply_pp_move rc=%d", store_rc);
          }
          pthread_mutex_unlock(&a->sh->mtx);
          break;
        }
        default:
          break;
      }
      for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
        if (act.axes_mask & (1u << axis)) {
          STC_LOG_INFO("CAN action complete: axis%zu target reached/processed", axis);
          log_axis_final_status(a->sh, (int)axis);
        }
      }
      if (!actionq_drop_head(a->q)) break;
      memset(&act, 0, sizeof(act));
    }

    stc_state_t st_state = STC_STATE_INIT;
    int watchdog_ms = 0;
    uint8_t fail_stop_pending = 0;
    uint32_t fail_stop_axes_mask = 0;
    pthread_mutex_lock(&a->sh->mtx);
    st_state = a->sh->d.st_state;
    watchdog_ms = a->sh->d.speed_watchdog_ms;
    speed_stream_period_ms = (a->sh->d.speed_stream_period_ms > 0) ? (uint64_t)a->sh->d.speed_stream_period_ms : 50u;
    fail_stop_pending = a->sh->d.fail_stop_pending;
    fail_stop_axes_mask = a->sh->d.fail_stop_axes_mask;
    a->sh->d.fail_stop_pending = 0;
    a->sh->d.fail_stop_axes_mask = 0;
    pthread_mutex_unlock(&a->sh->mtx);

    if (fail_stop_pending) {
      (void)motion_backend_quick_stop(backend, fail_stop_axes_mask);
      for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
        if ((fail_stop_axes_mask & (1u << axis)) == 0) continue;
        axis_enabled[axis] = 0;
        pthread_mutex_lock(&a->sh->mtx);
        a->sh->d.stream[axis].enabled = 0;
        pthread_mutex_unlock(&a->sh->mtx);
      }
      STC_LOG_WARN("FAIL entry quick-stop executed: axes_mask=0x%08x", fail_stop_axes_mask);
    }

    if (!stc_state_can_stream(st_state)) {
      for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
        pthread_mutex_lock(&a->sh->mtx);
        int speed_on = a->sh->d.stream[axis].enabled;
        a->sh->d.stream[axis].enabled = 0;
        pthread_mutex_unlock(&a->sh->mtx);
        if (speed_on) {
          (void)motion_backend_quick_stop(backend, (1u << axis));
          axis_enabled[axis] = 0;
        }
      }
    }

    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      if (!stc_state_can_stream(st_state)) continue;
      int speed_on = 0;
      double desired_deg_s = 0.0;
      uint64_t last_cmd_ms = 0;
      pthread_mutex_lock(&a->sh->mtx);
      speed_on = a->sh->d.stream[axis].enabled;
      desired_deg_s = a->sh->d.stream[axis].desired_speed_deg_s;
      last_cmd_ms = a->sh->d.stream[axis].last_cmd_ms;
      pthread_mutex_unlock(&a->sh->mtx);

      if (!speed_on) {
        if (speed_stream_active[axis]) {
          const uint64_t duration_ms = (now >= speed_stream_start_ms[axis]) ? (now - speed_stream_start_ms[axis]) : 0;
          STC_LOG_INFO("axis%zu MYACT speed stream summary tx=%u fail=%u duration_ms=%" PRIu64,
                       axis,
                       speed_stream_tx_count[axis],
                       speed_stream_fail_count[axis],
                       duration_ms);
          STC_LOG_INFO("axis%zu MYACT speed stream stop", axis);
          speed_stream_active[axis] = 0;
        }
        continue;
      }
      if (watchdog_ms > 0 && (int)(now - last_cmd_ms) > watchdog_ms) {
        pthread_mutex_lock(&a->sh->mtx);
        a->sh->d.stream[axis].enabled = 0;
        pthread_mutex_unlock(&a->sh->mtx);
        double zero_speeds[STC_AXIS_COUNT] = {0};
        zero_speeds[axis] = 0.0;
        (void)motion_backend_apply_speed(backend, (1u << axis), zero_speeds);
        axis_enabled[axis] = 0;
        if (speed_stream_active[axis]) {
          const uint64_t duration_ms = (now >= speed_stream_start_ms[axis]) ? (now - speed_stream_start_ms[axis]) : 0;
          STC_LOG_INFO("axis%zu MYACT speed stream summary tx=%u fail=%u duration_ms=%" PRIu64,
                       axis,
                       speed_stream_tx_count[axis],
                       speed_stream_fail_count[axis],
                       duration_ms);
          STC_LOG_INFO("axis%zu speed watchdog expired", axis);
          STC_LOG_INFO("axis%zu MYACT speed stream stop", axis);
          speed_stream_active[axis] = 0;
        }
      } else {
        if (!speed_stream_active[axis]) {
          double speeds[STC_AXIS_COUNT] = {0};
          int send_rc = 0;
          speeds[axis] = desired_deg_s;
          send_rc = motion_backend_apply_speed(backend, (1u << axis), speeds);
          speed_stream_active[axis] = 1;
          speed_stream_target_deg_s[axis] = desired_deg_s;
          next_speed_send_ms[axis] = now + speed_stream_period_ms;
          speed_stream_start_ms[axis] = now;
          speed_stream_tx_count[axis] = (send_rc == 0) ? 1u : 0u;
          speed_stream_fail_count[axis] = (send_rc == 0) ? 0u : 1u;
          speed_stream_consecutive_failures[axis] = (send_rc == 0) ? 0u : 1u;
          speed_stream_last_fail_log_ms[axis] = 0;
          STC_LOG_INFO("axis%zu MYACT speed stream start speed_deg_s=%.3f period_ms=%" PRIu64, axis, desired_deg_s, speed_stream_period_ms);
          if (send_rc != 0) STC_LOG_WARN("axis%zu MYACT speed stream A2 tx failed rc=%d", axis, send_rc);
        } else if (now >= next_speed_send_ms[axis]) {
          double speeds[STC_AXIS_COUNT] = {0};
          const int target_changed = (desired_deg_s != speed_stream_target_deg_s[axis]);
          speeds[axis] = desired_deg_s;
          if (target_changed) speed_stream_target_deg_s[axis] = desired_deg_s;
          const int send_rc = motion_backend_apply_speed(backend, (1u << axis), speeds);
          if (send_rc == 0) {
            speed_stream_tx_count[axis]++;
            speed_stream_consecutive_failures[axis] = 0;
          } else {
            speed_stream_fail_count[axis]++;
            speed_stream_consecutive_failures[axis]++;
            if ((now - speed_stream_last_fail_log_ms[axis]) >= 1000 || speed_stream_consecutive_failures[axis] >= 3) {
              STC_LOG_WARN("axis%zu MYACT speed stream A2 tx failed rc=%d consec=%u", axis, send_rc, speed_stream_consecutive_failures[axis]);
              speed_stream_last_fail_log_ms[axis] = now;
            }
          }
          next_speed_send_ms[axis] += speed_stream_period_ms;
          if (next_speed_send_ms[axis] < now) next_speed_send_ms[axis] = now + speed_stream_period_ms;
        }
        axis_enabled[axis] = 1;
      }
    }

    observability_emit_periodic_log(a->sh, now);
    next_maintenance_ms = now + maintenance_period_ms;

    pthread_mutex_lock(&a->sh->mtx);
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) a->sh->d.axis_enabled[axis] = axis_enabled[axis] ? 1u : 0u;
    pthread_mutex_unlock(&a->sh->mtx);
  }

  motion_backend_stop(backend);
  return NULL;
}


/* ----------------------------- telemetry threads ----------------------------- */

typedef struct {
  int udp_fd;
  const char* config_ip;
  struct in_addr config_addr;
  int config_addr_valid;
  int port;
  long period_ns;
  shared_t* sh;
  stc_icd_cfg_t cfg;
  uint8_t sender_id;
} tlm_tx_arg_t;

static int parse_ipv4_addr(const char* ip, struct in_addr* out) {
  if (!ip || !out) return -1;
  return (inet_pton(AF_INET, ip, out) == 1) ? 0 : -1;
}

static void format_ipv4_addr(const struct in_addr* addr, char* out, size_t out_sz) {
  if (!addr || !out || out_sz == 0) return;
  if (!inet_ntop(AF_INET, addr, out, out_sz)) {
    snprintf(out, out_sz, "<invalid>");
  }
}

static int telemetry_send_if_dest_available(const tlm_tx_arg_t* a,
                                            const uint8_t* buf,
                                            size_t blen) {
  struct in_addr dest_addr;
  int have_dest = 0;

  pthread_mutex_lock(&a->sh->mtx);
  if (a->sh->d.active_cu_connected) {
    dest_addr = a->sh->d.active_cu_addr;
    have_dest = 1;
  }
  pthread_mutex_unlock(&a->sh->mtx);

  if (!have_dest && a->config_addr_valid) {
    dest_addr = a->config_addr;
    have_dest = 1;
  }

  if (!have_dest) {
    // No active CU and no configured static destination.
    return 1;
  }

  char ipbuf[INET_ADDRSTRLEN];
  format_ipv4_addr(&dest_addr, ipbuf, sizeof(ipbuf));
  return (udp_send_to(a->udp_fd, ipbuf, a->port, buf, blen) == 0) ? 0 : -1;
}

static long telemetry_interval_ms_to_period_ns(int interval_ms, long default_period_ns) {
  if (interval_ms <= 0) return default_period_ns;

  long period_ns = (long)interval_ms * 1000000L;
  if (period_ns <= 0) return default_period_ns;
  return period_ns;
}

static int telemetry_interval_ms_to_rate_hz(int interval_ms, int default_hz) {
  if (interval_ms <= 0) return default_hz;

  int hz = (int)lround(1000.0 / (double)interval_ms);
  return (hz < 1) ? 1 : hz;
}

static long telemetry_rate_hz_to_period_ns(int rate_hz, long default_period_ns) {
  if (rate_hz <= 0) return default_period_ns;
  return 1000000000L / rate_hz;
}

static double ns_to_us(uint64_t ns) {
  return (double)ns / 1000.0;
}

static void observability_emit_periodic_log(shared_t* sh, uint64_t now_ms) {
  pthread_mutex_lock(&sh->mtx);
  if (elapsed_ms_nonnegative(now_ms, sh->d.obs_last_log_ms) < 1000) {
    pthread_mutex_unlock(&sh->mtx);
    return;
  }
  sh->d.obs_last_log_ms = now_ms;

  tx_timing_stats_t h = sh->d.tlmh_timing;
  tx_timing_stats_t l = sh->d.tlml_timing;
  cmd_timing_stats_t c = sh->d.cmd_timing;
  uint64_t qpeak = sh->d.action_queue_depth_peak;
  pthread_mutex_unlock(&sh->mtx);

  if (c.samples > 0) {
    STC_LOG_INFO(
      "OBS timing | tlmh(samples=%llu sent=%llu deadline_miss=%llu no_dest=%llu send_err=%llu jitter_us=min:%.1f avg:%.1f max:%.1f) | tlml(samples=%llu sent=%llu deadline_miss=%llu no_dest=%llu send_err=%llu jitter_us=min:%.1f avg:%.1f max:%.1f) | cmd(samples=%llu invalid=%llu tcp_to_queue_us=%.1f queue_to_exec_us=%.1f end_to_end_us=%.1f) | queue_depth_peak=%llu",
      (unsigned long long)h.sample_count,
      (unsigned long long)h.packets_sent_count,
      (unsigned long long)h.deadline_miss_count,
      (unsigned long long)h.no_destination_count,
      (unsigned long long)h.send_error_count,
      ns_to_us(h.last_sec_jitter_min_ns),
      ns_to_us(h.last_sec_jitter_avg_ns),
      ns_to_us(h.last_sec_jitter_max_ns),
      (unsigned long long)l.sample_count,
      (unsigned long long)l.packets_sent_count,
      (unsigned long long)l.deadline_miss_count,
      (unsigned long long)l.no_destination_count,
      (unsigned long long)l.send_error_count,
      ns_to_us(l.last_sec_jitter_min_ns),
      ns_to_us(l.last_sec_jitter_avg_ns),
      ns_to_us(l.last_sec_jitter_max_ns),
      (unsigned long long)c.samples,
      (unsigned long long)c.invalid_samples,
      ns_to_us(c.tcp_to_queue_avg_ns),
      ns_to_us(c.queue_to_exec_avg_ns),
      ns_to_us(c.end_to_end_avg_ns),
      (unsigned long long)qpeak);
  } else {
    STC_LOG_INFO(
      "OBS timing | tlmh(samples=%llu sent=%llu deadline_miss=%llu no_dest=%llu send_err=%llu jitter_us=min:%.1f avg:%.1f max:%.1f) | tlml(samples=%llu sent=%llu deadline_miss=%llu no_dest=%llu send_err=%llu jitter_us=min:%.1f avg:%.1f max:%.1f) | cmd(samples=0 pending) | queue_depth_peak=%llu",
      (unsigned long long)h.sample_count,
      (unsigned long long)h.packets_sent_count,
      (unsigned long long)h.deadline_miss_count,
      (unsigned long long)h.no_destination_count,
      (unsigned long long)h.send_error_count,
      ns_to_us(h.last_sec_jitter_min_ns),
      ns_to_us(h.last_sec_jitter_avg_ns),
      ns_to_us(h.last_sec_jitter_max_ns),
      (unsigned long long)l.sample_count,
      (unsigned long long)l.packets_sent_count,
      (unsigned long long)l.deadline_miss_count,
      (unsigned long long)l.no_destination_count,
      (unsigned long long)l.send_error_count,
      ns_to_us(l.last_sec_jitter_min_ns),
      ns_to_us(l.last_sec_jitter_avg_ns),
      ns_to_us(l.last_sec_jitter_max_ns),
      (unsigned long long)qpeak);
  }
}

typedef struct {
  shared_t* sh;
  int port;
} diag_server_arg_t;

static void* diag_server_thread(void* arg) {
  diag_server_arg_t* darg = (diag_server_arg_t*)arg;
  if (!darg || darg->port <= 0) return NULL;

  int srv = socket(AF_INET, SOCK_STREAM, 0);
  if (srv < 0) {
    STC_LOG_ERROR("diag server socket failed: %s", strerror(errno));
    return NULL;
  }

  int yes = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons((uint16_t)darg->port);

  if (bind(srv, (struct sockaddr*)&addr, sizeof(addr)) < 0 || listen(srv, 2) < 0) {
    STC_LOG_ERROR("diag server bind/listen failed on 127.0.0.1:%d: %s", darg->port, strerror(errno));
    close(srv);
    return NULL;
  }

  STC_LOG_INFO("diagnostics endpoint enabled on 127.0.0.1:%d", darg->port);

  while (!atomic_load(&g_stop)) {
    struct pollfd pfd = {.fd = srv, .events = POLLIN, .revents = 0};
    int prc = poll(&pfd, 1, 500);
    if (prc <= 0 || (pfd.revents & POLLIN) == 0) continue;

    int cfd = accept(srv, NULL, NULL);
    if (cfd < 0) continue;

    char out[1024];
    pthread_mutex_lock(&darg->sh->mtx);
    tx_timing_stats_t h = darg->sh->d.tlmh_timing;
    tx_timing_stats_t l = darg->sh->d.tlml_timing;
    cmd_timing_stats_t c = darg->sh->d.cmd_timing;
    uint64_t qpeak = darg->sh->d.action_queue_depth_peak;
    uint8_t backend_mode = darg->sh->d.backend_mode;
    uint8_t transport_status = darg->sh->d.transport_status;
    uint8_t supports_a2_speed = darg->sh->d.supports_native_a2_speed;
    uint8_t supports_absolute = darg->sh->d.supports_native_absolute_position;
    pthread_mutex_unlock(&darg->sh->mtx);

    int n = snprintf(out, sizeof(out),
                     "{\"tlmh\":{\"samples\":%llu,\"sent\":%llu,\"deadline_miss\":%llu,\"no_destination\":%llu,\"send_errors\":%llu,\"expected_wake_ns\":%llu,\"actual_wake_ns\":%llu,\"send_complete_ns\":%llu,\"jitter_last_sec\":{\"min\":%llu,\"max\":%llu,\"avg\":%llu}},"
                     "\"tlml\":{\"samples\":%llu,\"sent\":%llu,\"deadline_miss\":%llu,\"no_destination\":%llu,\"send_errors\":%llu,\"expected_wake_ns\":%llu,\"actual_wake_ns\":%llu,\"send_complete_ns\":%llu,\"jitter_last_sec\":{\"min\":%llu,\"max\":%llu,\"avg\":%llu}},"
                     "\"command_path\":{\"samples\":%llu,\"invalid_samples\":%llu,\"tcp_recv_ns\":%llu,\"action_queued_ns\":%llu,\"exec_start_ns\":%llu,\"tcp_to_queue_avg_ns\":%llu,\"queue_to_exec_avg_ns\":%llu,\"end_to_end_avg_ns\":%llu},"
                     "\"queue_depth_peak\":%llu,\"backend\":{\"mode\":%u,\"transport_status\":%u,\"capabilities\":{\"native_a2_speed\":%u,\"native_absolute_position\":%u}}}\n",
                     (unsigned long long)h.sample_count,
                     (unsigned long long)h.packets_sent_count,
                     (unsigned long long)h.deadline_miss_count,
                     (unsigned long long)h.no_destination_count,
                     (unsigned long long)h.send_error_count,
                     (unsigned long long)h.expected_wake_ns,
                     (unsigned long long)h.actual_wake_ns,
                     (unsigned long long)h.send_complete_ns,
                     (unsigned long long)h.last_sec_jitter_min_ns,
                     (unsigned long long)h.last_sec_jitter_max_ns,
                     (unsigned long long)h.last_sec_jitter_avg_ns,
                     (unsigned long long)l.sample_count,
                     (unsigned long long)l.packets_sent_count,
                     (unsigned long long)l.deadline_miss_count,
                     (unsigned long long)l.no_destination_count,
                     (unsigned long long)l.send_error_count,
                     (unsigned long long)l.expected_wake_ns,
                     (unsigned long long)l.actual_wake_ns,
                     (unsigned long long)l.send_complete_ns,
                     (unsigned long long)l.last_sec_jitter_min_ns,
                     (unsigned long long)l.last_sec_jitter_max_ns,
                     (unsigned long long)l.last_sec_jitter_avg_ns,
                     (unsigned long long)c.samples,
                     (unsigned long long)c.invalid_samples,
                     (unsigned long long)c.tcp_recv_ns,
                     (unsigned long long)c.action_queued_ns,
                     (unsigned long long)c.exec_start_ns,
                     (unsigned long long)c.tcp_to_queue_avg_ns,
                     (unsigned long long)c.queue_to_exec_avg_ns,
                     (unsigned long long)c.end_to_end_avg_ns,
                     (unsigned long long)qpeak,
                     (unsigned)backend_mode,
                     (unsigned)transport_status,
                     (unsigned)supports_a2_speed,
                     (unsigned)supports_absolute);
    if (n > 0) (void)send_all(cfd, (const uint8_t*)out, (size_t)n);
    close(cfd);
  }

  close(srv);
  return NULL;
}

static void* tlmh_tx_thread(void* arg) {
  tlm_tx_arg_t* a = (tlm_tx_arg_t*)arg;
  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  const long period_ns = a->period_ns;
  uint64_t last_send_error_log_ms = 0;

  while (!atomic_load(&g_stop)) {
    const uint64_t expected_wake_ns = (uint64_t)next.tv_sec * 1000000000ULL + (uint64_t)next.tv_nsec;
    const uint64_t actual_wake_ns = mono_ns();
    stc_tlmh_t m;
    memset(&m, 0, sizeof(m));
    memcpy(m.magic, a->cfg.tlmh_magic, 4);
    m.version = a->cfg.protocol_version_default;

    shared_data_t snap;
    pthread_mutex_lock(&a->sh->mtx);
    m.seq = a->sh->d.seq_h++;
    snap = a->sh->d;
    pthread_mutex_unlock(&a->sh->mtx);

    m.timestamp_ms = wall_ms();

    for (size_t i = 0; i < STC_AXIS_COUNT; i++) {
      m.position[i] = (float)snap.telem[i].pos_deg;
      m.rates[i] = (float)snap.telem[i].vel_deg_s;
      m.acceleration[i] = (float)snap.telem[i].accel_deg_s2;
    }

    uint8_t buf[512];
    size_t blen = 0;
    if (stc_icd_tlmh_pack(&a->cfg, &m, buf, sizeof(buf), &blen) == STC_ICD_OK) {
      int tx_rc = telemetry_send_if_dest_available(a, buf, blen);
      pthread_mutex_lock(&a->sh->mtx);
      if (tx_rc == 0) {
        a->sh->d.tlmh_timing.packets_sent_count++;
      } else if (tx_rc > 0) {
        a->sh->d.tlmh_timing.no_destination_count++;
      } else {
        a->sh->d.tlmh_timing.send_error_count++;
        const uint64_t now_ms = mono_ms();
        if (last_send_error_log_ms == 0 || (now_ms - last_send_error_log_ms) >= 1000ULL) {
          STC_LOG_WARN("TLMH sendto failed: %s", strerror(errno));
          last_send_error_log_ms = now_ms;
        }
      }
      pthread_mutex_unlock(&a->sh->mtx);
    }

    const uint64_t send_complete_ns = mono_ns();
    pthread_mutex_lock(&a->sh->mtx);
    tx_timing_update(&a->sh->d.tlmh_timing, expected_wake_ns, actual_wake_ns, send_complete_ns, period_ns);
    pthread_mutex_unlock(&a->sh->mtx);
    observability_emit_periodic_log(a->sh, mono_ms());
    sleep_until(&next, period_ns);
  }
  return NULL;
}

static void* tlml_tx_thread(void* arg) {
  tlm_tx_arg_t* a = (tlm_tx_arg_t*)arg;
  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  const long period_ns = a->period_ns;
  uint64_t last_send_error_log_ms = 0;

  while (!atomic_load(&g_stop)) {
    const uint64_t expected_wake_ns = (uint64_t)next.tv_sec * 1000000000ULL + (uint64_t)next.tv_nsec;
    const uint64_t actual_wake_ns = mono_ns();
    stc_tlml_t m;
    memset(&m, 0, sizeof(m));
    memcpy(m.magic, a->cfg.tlml_magic, 4);
    m.version = a->cfg.protocol_version_default;
    m.controller_id = a->sender_id;

    uint64_t nowm = mono_ms();
    uint64_t noww = wall_ms();

    pthread_mutex_lock(&a->sh->mtx);
    m.seq = a->sh->d.seq_l++;
    m.state = stc_state_to_wire(a->sh->d.st_state);
    uint64_t uptime_ms = nowm - a->sh->d.boot_ms;
    m.uptime_sec = (uint32_t)(uptime_ms / 1000ULL);
    m.lat = a->sh->d.has_manualpos ? a->sh->d.lat : 0.0;
    m.lon = a->sh->d.has_manualpos ? a->sh->d.lon : 0.0;
    m.alt_m = a->sh->d.has_manualpos ? a->sh->d.alt : 0.0;
    m.heading_deg = a->sh->d.has_manualpos ? a->sh->d.heading : 0.0;

    bool saturated[STC_AXIS_COUNT] = {false};
    uint8_t axis_fault_active[STC_AXIS_COUNT] = {0};
    uint8_t axis_enabled[STC_AXIS_COUNT] = {0};
    uint8_t axis_available[STC_AXIS_COUNT] = {0};
    uint8_t axis_expected[STC_AXIS_COUNT] = {0};
    uint64_t last_rx_ms[STC_AXIS_COUNT] = {0};
    uint8_t diag_cat[STC_AXIS_COUNT] = {0};
    uint8_t diag_subcat[STC_AXIS_COUNT] = {0};
    uint8_t stream_enabled[STC_AXIS_COUNT] = {0};
    double axis_speed_deg_s[STC_AXIS_COUNT] = {0.0};

    for (size_t i = 0; i < STC_AXIS_COUNT; i++) {
      saturated[i] = a->sh->d.saturated[i] && (nowm <= a->sh->d.saturated_until_ms[i]);
      axis_fault_active[i] = a->sh->d.axis_fault_active[i];
      axis_enabled[i] = a->sh->d.axis_enabled[i];
      axis_available[i] = a->sh->d.axis_available[i];
      axis_expected[i] = a->sh->d.axis_expected[i];
      last_rx_ms[i] = a->sh->d.last_rx_ms[i];
      diag_cat[i] = a->sh->d.diag_cat[i];
      diag_subcat[i] = a->sh->d.diag_subcat[i];
      if (!saturated[i]) {
        a->sh->d.saturated[i] = false;
      }
    }
    uint32_t can_timeout_ms = a->sh->d.can_timeout_ms;
    uint32_t startup_feedback_grace_ms = a->sh->d.startup_feedback_grace_ms;
    uint8_t laser_status_snapshot[STC_ICD_LASER_COUNT];
    for (size_t i = 0; i < STC_ICD_LASER_COUNT; ++i) {
      laser_status_snapshot[i] = a->sh->d.sut_laser_on[i] ? 1u : 0u;
    }
    int strict_stale_policy = a->sh->d.strict_stale_policy ? 1 : 0;
    for (size_t i = 0; i < STC_AXIS_COUNT; i++) {
      stream_enabled[i] = a->sh->d.stream[i].enabled ? 1u : 0u;
      axis_speed_deg_s[i] = a->sh->d.telem[i].vel_deg_s;
    }
    const int in_startup_grace = stc_is_within_startup_grace(nowm, a->sh->d.server_start_ms, startup_feedback_grace_ms);
    pthread_mutex_unlock(&a->sh->mtx);

    m.timestamp_ms = noww;
    m.temperature_c = 0.0f;
    m.humidity_pct = 0.0f;
    m.pressure_hpa = 0.0f;
    m.level_x = 0.0f;
    m.level_y = 0.0f;

    for (size_t axis = 0; axis < STC_ICD_AXES_COUNT; axis++) {
      const int enabled = axis_enabled[axis] ? 1 : 0;
      if (!enabled) {
        m.motor_status[axis] = 0x00;
        continue;
      }

      const int expected = axis_expected[axis] ? 1 : 0;
      const int connected = axis_available[axis] ? 1 : 0;
      const int timed_out = elapsed_ms_nonnegative(nowm, last_rx_ms[axis]) > can_timeout_ms;
      const int stale = (!in_startup_grace && can_timeout_ms > 0)
        ? (strict_stale_policy ? (expected && timed_out) : (expected && connected && timed_out))
        : 0;
      const int moving_or_streaming = (fabs(axis_speed_deg_s[axis]) > 0.01) || stream_enabled[axis];
      const int degraded = stale;
      m.motor_status[axis] = build_axis_health_byte(enabled,
                                                    connected,
                                                    axis_fault_active[axis] ? 1 : 0,
                                                    stale,
                                                    saturated[axis] ? 1 : 0,
                                                    moving_or_streaming,
                                                    degraded);
    }

    memset(m.laser_status, 0, sizeof(m.laser_status));
    for (size_t i = 0; i < STC_ICD_LASER_COUNT; ++i) {
      m.laser_status[i] = laser_status_snapshot[i];
    }

    m.error_count = 0;
    for (size_t i = 0; i < STC_AXIS_COUNT && m.error_count < STC_ICD_MAX_ERRORS; i++) {
      const int expected = axis_expected[i] ? 1 : 0;
      const int connected = axis_available[i] ? 1 : 0;
      const int timed_out = elapsed_ms_nonnegative(nowm, last_rx_ms[i]) > can_timeout_ms;
      const int stale = (!in_startup_grace && can_timeout_ms > 0)
        ? (strict_stale_policy ? (expected && timed_out) : (expected && connected && timed_out))
        : 0;
      if (axis_fault_active[i]) {
        m.errors[m.error_count++] = stc_diag_encode(diag_cat[i], diag_subcat[i], STC_DIAG_ERR_AXIS_FAULT);
      }
      if (stale && m.error_count < STC_ICD_MAX_ERRORS) {
        m.errors[m.error_count++] = stc_diag_encode(diag_cat[i], diag_subcat[i], STC_DIAG_ERR_AXIS_STALE);
      }
      if (m.error_count < STC_ICD_MAX_ERRORS) {
        const uint16_t myact_flags = (uint16_t)(a->sh->d.telem[i].status_word & 0xFFFFu);
        const uint16_t myact_code = stc_diag_myact_primary_error_code(myact_flags);
        if (myact_code != 0u) {
          m.errors[m.error_count++] = stc_diag_encode(diag_cat[i], diag_subcat[i], myact_code);
        }
      }
    }

    for (size_t i = 0; i < STC_AXIS_COUNT && m.error_count < STC_ICD_MAX_ERRORS; i++) {
      if (saturated[i]) {
        m.errors[m.error_count++] = stc_diag_encode(diag_cat[i], diag_subcat[i], STC_DIAG_WARN_AXIS_SAT);
      }
    }

    uint8_t buf[1024];
    size_t blen = 0;
    if (stc_icd_tlml_pack(&a->cfg, &m, buf, sizeof(buf), &blen) == STC_ICD_OK) {
      int tx_rc = telemetry_send_if_dest_available(a, buf, blen);
      pthread_mutex_lock(&a->sh->mtx);
      if (tx_rc == 0) {
        a->sh->d.tlml_timing.packets_sent_count++;
      } else if (tx_rc > 0) {
        a->sh->d.tlml_timing.no_destination_count++;
      } else {
        a->sh->d.tlml_timing.send_error_count++;
        const uint64_t now_ms = mono_ms();
        if (last_send_error_log_ms == 0 || (now_ms - last_send_error_log_ms) >= 1000ULL) {
          STC_LOG_WARN("TLML sendto failed: %s", strerror(errno));
          last_send_error_log_ms = now_ms;
        }
      }
      pthread_mutex_unlock(&a->sh->mtx);
    }

    const uint64_t send_complete_ns = mono_ns();
    pthread_mutex_lock(&a->sh->mtx);
    tx_timing_update(&a->sh->d.tlml_timing, expected_wake_ns, actual_wake_ns, send_complete_ns, period_ns);
    pthread_mutex_unlock(&a->sh->mtx);
    observability_emit_periodic_log(a->sh, mono_ms());
    sleep_until(&next, period_ns);
  }
  return NULL;
}

/* ----------------------------- TCP server loop ----------------------------- */

static void print_usage(FILE* out, const char* p) {
  fprintf(out, "Usage: %s [--tcp-port 7000] [--can can0] [--node 1] [--can-bitrate 1000] [--can-restart-ms 100]\n"
          "          [--can-scan-max-node 32]\n"
          "          [--config dev/config/stc_default.ini]\n"
          "          [--counts-per-rev N] [--gear-ratio N] [--vel-scale N]\n"
          "          [--az-min DEG] [--az-max DEG] [--max-speed-deg-s X]\n"
          "          [--speed-watchdog-ms MS] [--pp-profile-vel U16]\n"
          "          [--ds402-timeout-ms MS] [--pp-sdo-diag 0|1]\n"
          "          [--tlmh-ip 127.0.0.1 --tlmh-port 6010 --tlmh-hz 100]\n"
          "          [--tlml-ip 127.0.0.1 --tlml-port 6020 --tlml-hz 10]\n"
          "          [--rt-enabled 0|1] [--rt-strict 0|1] [--rt-policy fifo|rr|other]\n"
          "          [--can-worker-prio 80] [--tlmh-prio 70] [--tlml-prio 60]\n"
          "          [--diag-port 0|PORT] [--strict-stale-policy 0|1]\n",
          p);
}

static void usage(const char* p) {
  STC_LOG_ERROR("Invalid command-line arguments");
  print_usage(stderr, p);
}


typedef struct {
  const char* can_if;
  int can_bitrate;
  int can_restart_ms;
  int can_scan_max_node;
  uint32_t can_timeout_ms;
  int node_id;
  int tcp_port;
  const char* tlmh_ip;
  int tlmh_port;
  int tlmh_hz;
  long tlmh_period_ns;
  const char* tlml_ip;
  int tlml_port;
  int tlml_hz;
  long tlml_period_ns;
  int diag_port;
  runtime_sched_t sched;
} runtime_net_t;

typedef struct {
  int has_tlmh_ip;
  int has_tlmh_port;
  int has_tlmh_hz;
  int has_tlml_ip;
  int has_tlml_port;
  int has_tlml_hz;
} cli_override_flags_t;

static stc_event_t run_startup_pbit(const shared_t* sh, const runtime_net_t* net) {
  if (!sh || !net) {
    STC_LOG_ERROR("PBIT FAIL: invalid startup context");
    return EVT_PBIT_FAIL;
  }

  int expected_motors = 0;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (sh->d.axis_expected[axis]) expected_motors++;
  }

  int network_ok = 1;
  struct in_addr parsed_addr;
  if (net->can_if == NULL || net->can_if[0] == '\0' || net->can_bitrate <= 0) network_ok = 0;
  if (net->tcp_port <= 0 || net->tcp_port > 65535) network_ok = 0;
  if (net->tlmh_port <= 0 || net->tlmh_port > 65535) network_ok = 0;
  if (net->tlml_port <= 0 || net->tlml_port > 65535) network_ok = 0;
  if (parse_ipv4_addr(net->tlmh_ip, &parsed_addr) != 0) network_ok = 0;
  if (parse_ipv4_addr(net->tlml_ip, &parsed_addr) != 0) network_ok = 0;

  const int motors_ok = (expected_motors > 0);
  STC_LOG_INFO("PBIT motor_check=%s expected_motors=%d", motors_ok ? "OK" : "FAIL", expected_motors);
  STC_LOG_INFO("PBIT network_check=%s can_if=%s can_bitrate=%d tcp_port=%d tlmh=%s:%d tlml=%s:%d",
               network_ok ? "OK" : "FAIL",
               net->can_if ? net->can_if : "<null>",
               net->can_bitrate,
               net->tcp_port,
               net->tlmh_ip ? net->tlmh_ip : "<null>",
               net->tlmh_port,
               net->tlml_ip ? net->tlml_ip : "<null>",
               net->tlml_port);

  stc_diag_set_pbit_result(motors_ok && network_ok);
  const int pass = run_pbit_startup() ? 1 : 0;
  if (pass) {
    STC_LOG_INFO("PBIT result: PASS");
    return EVT_PBIT_OK;
  }

  STC_LOG_ERROR("PBIT result: FAIL");
  return EVT_PBIT_FAIL;
}

static stc_event_t run_admin_reset_ibit(const shared_t* sh, const runtime_net_t* net) {
  if (!sh || !net) {
    STC_LOG_ERROR("RESET/IBIT recovery FAIL: invalid context");
    return EVT_IBIT_FAIL;
  }

  int expected_motors = 0;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (sh->d.axis_expected[axis]) expected_motors++;
  }

  int network_ok = 1;
  struct in_addr parsed_addr;
  if (net->can_if == NULL || net->can_if[0] == '\0' || net->can_bitrate <= 0) network_ok = 0;
  if (net->tcp_port <= 0 || net->tcp_port > 65535) network_ok = 0;
  if (net->tlmh_port <= 0 || net->tlmh_port > 65535) network_ok = 0;
  if (net->tlml_port <= 0 || net->tlml_port > 65535) network_ok = 0;
  if (parse_ipv4_addr(net->tlmh_ip, &parsed_addr) != 0) network_ok = 0;
  if (parse_ipv4_addr(net->tlml_ip, &parsed_addr) != 0) network_ok = 0;

  const int motors_ok = (expected_motors > 0);
  STC_LOG_INFO("RESET/IBIT motor_check=%s expected_motors=%d", motors_ok ? "OK" : "FAIL", expected_motors);
  STC_LOG_INFO("RESET/IBIT network_check=%s can_if=%s can_bitrate=%d tcp_port=%d tlmh=%s:%d tlml=%s:%d",
               network_ok ? "OK" : "FAIL",
               net->can_if ? net->can_if : "<null>",
               net->can_bitrate,
               net->tcp_port,
               net->tlmh_ip ? net->tlmh_ip : "<null>",
               net->tlmh_port,
               net->tlml_ip ? net->tlml_ip : "<null>",
               net->tlml_port);

  stc_diag_set_ibit_result(motors_ok && network_ok);
  const int pass = run_ibit_sequence((shared_data_t*)&sh->d) ? 1 : 0;
  if (pass) {
    STC_LOG_INFO("RESET/IBIT result: PASS");
    return EVT_IBIT_COMPLETE_OK;
  }

  STC_LOG_ERROR("RESET/IBIT result: FAIL");
  return EVT_IBIT_COMPLETE_FAIL;
}

static void apply_config_to_shared(shared_t* sh, const stc_config_t* cfg) {
  sh->d.num_axes = cfg->num_axes;
  memset(sh->d.axis_expected, 0, sizeof(sh->d.axis_expected));
  for (int i = 0; i < cfg->num_axes && i < (int)STC_AXIS_COUNT; i++) {
    sh->d.axis_node_id[i] = (uint8_t)cfg->axis_node_id[i];
    stc_axis_cfg_from_config(cfg, i, &sh->d.cfg[i], &sh->d.axis_expected[i]);

    sh->d.storage_deg[i] = cfg->axis_storage_deg[i];
    sh->d.diag_cat[i] = cfg->axis_diag_cat[i];
    sh->d.diag_subcat[i] = cfg->axis_diag_subcat[i];
  }

  sh->d.turret_az_axis = cfg->turret_az_axis;
  sh->d.turret_el_axis = cfg->turret_el_axis;

  sh->d.sut_count = cfg->sut_count;
  memcpy(sh->d.sut_az_axis, cfg->sut_az_axis, sizeof(sh->d.sut_az_axis));
  memcpy(sh->d.sut_el_axis, cfg->sut_el_axis, sizeof(sh->d.sut_el_axis));
  memcpy(sh->d.sut_laser_on, cfg->sut_laser_on, sizeof(sh->d.sut_laser_on));

  sh->d.mirror_count = cfg->mirror_count;
  memcpy(sh->d.mirror_axis, cfg->mirror_axis, sizeof(sh->d.mirror_axis));
  sh->d.controller_id_strict_legacy = cfg->controller_id_strict_legacy ? 1 : 0;
  sh->d.startup_feedback_grace_ms = (cfg->startup_feedback_grace_ms > 0) ? (uint32_t)cfg->startup_feedback_grace_ms : 0u;
  sh->d.speed_watchdog_ms = (cfg->speed_watchdog_ms >= 0) ? cfg->speed_watchdog_ms : 0;
  sh->d.speed_stream_period_ms = (cfg->speed_stream_period_ms > 0) ? cfg->speed_stream_period_ms : 50;
  sh->d.position_tolerance_deg = (cfg->position_tolerance_deg > 0.0) ? cfg->position_tolerance_deg : 1.0;
  sh->d.position_timeout_ms = (cfg->position_timeout_ms > 0) ? cfg->position_timeout_ms : 5000;
  sh->d.position_poll_ms = (cfg->position_poll_ms > 0) ? cfg->position_poll_ms : 50;
  sh->d.strict_stale_policy = cfg->strict_stale_policy ? 1 : 0;
}

static int config_has_enabled_driver(const stc_config_t* cfg, stc_axis_driver_t driver) {
  if (!cfg) return 0;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!cfg->axis_enabled[axis]) continue;
    if (cfg->axis_driver[axis] == driver) return 1;
  }
  return 0;
}

static void log_enabled_axis_drivers(const stc_config_t* cfg) {
  if (!cfg) return;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!cfg->axis_enabled[axis]) continue;
    STC_LOG_ERROR("enabled axis[%zu] driver=%s", axis, axis_driver_to_string(cfg->axis_driver[axis]));
  }
}

static const char* select_backend_name(const stc_config_t* cfg) {
  if (!cfg) return NULL;

  const int has_pd4c = config_has_enabled_driver(cfg, STC_AXIS_DRIVER_PD4C);
  const int has_myact = config_has_enabled_driver(cfg, STC_AXIS_DRIVER_MYACT_X8_25);

  if (strcmp(cfg->backend, "simulator") == 0) {
#ifdef SIMULATION
    return "simulator";
#else
    STC_LOG_ERROR("backend='simulator' requested but binary is not built with SIMULATION");
    return NULL;
#endif
  }

  if (strcmp(cfg->backend, "pd4c") == 0) {
    if (has_myact) {
      STC_LOG_ERROR("backend='pd4c' cannot be used with mixed hardware drivers (single-backend runtime)");
      log_enabled_axis_drivers(cfg);
      return NULL;
    }
    if (!has_pd4c) {
      STC_LOG_WARN("backend='pd4c' requested but no enabled PD4C axes found");
    }
    return "pd4c";
  }

  if (strcmp(cfg->backend, "myactuator") == 0) {
    if (has_pd4c) {
      STC_LOG_ERROR("backend='myactuator' cannot be used with mixed hardware drivers (single-backend runtime)");
      log_enabled_axis_drivers(cfg);
      return NULL;
    }
    if (!has_myact) {
      STC_LOG_WARN("backend='myactuator' requested but no enabled myactuator axes found");
    }
    return "myactuator";
  }

  STC_LOG_ERROR("invalid backend='%s' (expected simulator|pd4c|myactuator)", cfg->backend);
  return NULL;
}

int main(int argc, char** argv) {
  int exit_code = 0;
  int threads_started = 0;
  int can_thread_started = 0;
  int tlmh_thread_started = 0;
  int tlml_thread_started = 0;
  int diag_thread_started = 0;
  int udp_h_open = 0;
  int udp_l_open = 0;
  int srv_open = 0;

  pthread_t th_can = 0;
  pthread_t th_h = 0;
  pthread_t th_l = 0;
  pthread_t th_diag = 0;
  int udp_h = -1;
  int udp_l = -1;
  int srv = -1;

  stc_log_init("stc.log", STC_LOG_LEVEL_DEBUG);

  runtime_net_t net = {
    .can_if = "can0",
    .can_bitrate = 1000,
    .can_restart_ms = 100,
    .can_scan_max_node = 32,
    .can_timeout_ms = 1000,
    .node_id = 1,
    .tcp_port = 7000,
    .tlmh_ip = "127.0.0.1",
    .tlmh_port = 6010,
    .tlmh_hz = 100,
    .tlmh_period_ns = 10000000L,
    .tlml_ip = "127.0.0.1",
    .tlml_port = 6020,
    .tlml_hz = 10,
    .tlml_period_ns = 100000000L,
    .diag_port = 0,
    .sched = {
      .rt_enabled = 1,
      .rt_strict = 0,
      .sched_policy = SCHED_FIFO,
      .can_worker_priority = 80,
      .tlmh_tx_priority = 70,
      .tlml_tx_priority = 60,
    },
  };
  const char* config_path = NULL;
  int legacy_cli_mode = 0;
  cli_override_flags_t cli_overrides;
  memset(&cli_overrides, 0, sizeof(cli_overrides));

  stc_config_t parsed_cfg;
  stc_config_set_defaults(&parsed_cfg);

  shared_t sh;
  memset(&sh, 0, sizeof(sh));
  pthread_mutex_init(&sh.mtx, NULL);
  memset(&sh.d.cfg, 0, sizeof(sh.d.cfg));
  memset(&sh.d.telem, 0, sizeof(sh.d.telem));
  memset(&sh.d.stream, 0, sizeof(sh.d.stream));
  sh.d.boot_ms = mono_ms();
  sh.d.server_start_ms = sh.d.boot_ms;

  // defaults (axis 0 from CLI; axes 1..17 remain zero/disabled in step3)
  sh.d.cfg[0].counts_per_rev = 4096.0;
  sh.d.cfg[0].gear_ratio = 1.0;
  sh.d.cfg[0].vel_raw_units = AXIS_VEL_RAW_DEG_PER_SEC;
  sh.d.cfg[0].vel_scale = 1.0;
  sh.d.cfg[0].az_min_deg = -180.0;
  sh.d.cfg[0].az_max_deg = 180.0;
  sh.d.cfg[0].max_speed_deg_s = 90.0;
  sh.d.speed_watchdog_ms = 250;
  sh.d.position_tolerance_deg = 1.0;
  sh.d.position_timeout_ms = 5000;
  sh.d.position_poll_ms = 50;
  sh.d.cfg[0].pp_profile_vel = 500;       // default profile velocity
  sh.d.ds402_timeout_ms = 2000;
  sh.d.pp_sdo_diag = 0;
  sh.d.can_timeout_ms = 1000;
  sh.d.startup_feedback_grace_ms = 3000;
  sh.d.strict_stale_policy = 0;
  sh.d.st_state = STC_STATE_INIT;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    sh.d.last_rx_ms[axis] = sh.d.boot_ms;
    sh.d.axis_enabled[axis] = 0;
    sh.d.axis_fault_active[axis] = 0;
    sh.d.axis_available[axis] = 0;
  }
  sh.d.can_bitrate_kbps = (uint16_t)((net.can_bitrate > 0) ? net.can_bitrate : 0);
  sh.d.can_restart_ms = (uint16_t)((net.can_restart_ms > 0) ? net.can_restart_ms : 0);
  sh.d.can_scan_max_node = (uint8_t)((net.can_scan_max_node > 0) ? net.can_scan_max_node : 0);

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      print_usage(stdout, argv[0]);
      return 0;
    }
    else if (!strcmp(argv[i], "--can") && i + 1 < argc) net.can_if = argv[++i];
    else if (!strcmp(argv[i], "--can-bitrate") && i + 1 < argc) net.can_bitrate = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--can-restart-ms") && i + 1 < argc) net.can_restart_ms = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--can-scan-max-node") && i + 1 < argc) net.can_scan_max_node = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--node") && i + 1 < argc) {
      net.node_id = atoi(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tcp-port") && i + 1 < argc) {
      net.tcp_port = atoi(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--config") && i + 1 < argc) config_path = argv[++i];
    else if (!strcmp(argv[i], "--counts-per-rev") && i + 1 < argc) {
      sh.d.cfg[0].counts_per_rev = atof(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--gear-ratio") && i + 1 < argc) {
      sh.d.cfg[0].gear_ratio = atof(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--vel-scale") && i + 1 < argc) {
      sh.d.cfg[0].vel_scale = atof(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--az-min") && i + 1 < argc) {
      sh.d.cfg[0].az_min_deg = atof(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--az-max") && i + 1 < argc) {
      sh.d.cfg[0].az_max_deg = atof(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--max-speed-deg-s") && i + 1 < argc) {
      sh.d.cfg[0].max_speed_deg_s = atof(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--speed-watchdog-ms") && i + 1 < argc) {
      sh.d.speed_watchdog_ms = atoi(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--pp-profile-vel") && i + 1 < argc) {
      sh.d.cfg[0].pp_profile_vel = (uint16_t)atoi(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--ds402-timeout-ms") && i + 1 < argc) sh.d.ds402_timeout_ms = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--pp-sdo-diag") && i + 1 < argc) sh.d.pp_sdo_diag = atoi(argv[++i]) ? 1 : 0;
    else if (!strcmp(argv[i], "--can-timeout-ms") && i + 1 < argc) {
      net.can_timeout_ms = (uint32_t)atoi(argv[++i]);
      sh.d.can_timeout_ms = net.can_timeout_ms;
    }
    else if (!strcmp(argv[i], "--strict-stale-policy") && i + 1 < argc) {
      sh.d.strict_stale_policy = atoi(argv[++i]) ? 1 : 0;
    }
    else if (!strcmp(argv[i], "--tlmh-ip") && i + 1 < argc) {
      net.tlmh_ip = argv[++i];
      cli_overrides.has_tlmh_ip = 1;
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlmh-port") && i + 1 < argc) {
      net.tlmh_port = atoi(argv[++i]);
      cli_overrides.has_tlmh_port = 1;
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlmh-hz") && i + 1 < argc) {
      net.tlmh_hz = atoi(argv[++i]);
      if (net.tlmh_hz < 1) net.tlmh_hz = 1;
      net.tlmh_period_ns = telemetry_rate_hz_to_period_ns(net.tlmh_hz, 10000000L);
      cli_overrides.has_tlmh_hz = 1;
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlml-ip") && i + 1 < argc) {
      net.tlml_ip = argv[++i];
      cli_overrides.has_tlml_ip = 1;
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlml-port") && i + 1 < argc) {
      net.tlml_port = atoi(argv[++i]);
      cli_overrides.has_tlml_port = 1;
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlml-hz") && i + 1 < argc) {
      net.tlml_hz = atoi(argv[++i]);
      if (net.tlml_hz < 1) net.tlml_hz = 1;
      net.tlml_period_ns = telemetry_rate_hz_to_period_ns(net.tlml_hz, 100000000L);
      cli_overrides.has_tlml_hz = 1;
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--diag-port") && i + 1 < argc) {
      net.diag_port = atoi(argv[++i]);
    }
    else if (!strcmp(argv[i], "--rt-enabled") && i + 1 < argc) {
      net.sched.rt_enabled = atoi(argv[++i]) ? 1 : 0;
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--rt-strict") && i + 1 < argc) {
      net.sched.rt_strict = atoi(argv[++i]) ? 1 : 0;
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--rt-policy") && i + 1 < argc) {
      if (parse_sched_policy_cli(argv[++i], &net.sched.sched_policy) != 0) {
        STC_LOG_ERROR("invalid --rt-policy value; expected fifo|rr|other");
        exit_code = 2;
        goto cleanup;
      }
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--can-worker-prio") && i + 1 < argc) {
      net.sched.can_worker_priority = atoi(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlmh-prio") && i + 1 < argc) {
      net.sched.tlmh_tx_priority = atoi(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlml-prio") && i + 1 < argc) {
      net.sched.tlml_tx_priority = atoi(argv[++i]);
      legacy_cli_mode = 1;
    }
    else {
      usage(argv[0]);
      exit_code = 2;
      goto cleanup;
    }
  }

  if (!config_path && !legacy_cli_mode) {
    FILE* f = fopen("dev/config/stc_default.ini", "r");
    if (f) {
      fclose(f);
      config_path = "dev/config/stc_default.ini";
    }
  }

  if (config_path) {
    if (!load_config(config_path, &parsed_cfg)) {
      STC_LOG_ERROR("Failed to load config file: %s", config_path);
      exit_code = 1;
      goto cleanup;
    }

    net.node_id = parsed_cfg.axis_node_id[0];
    net.tcp_port = parsed_cfg.tcp_port;
    net.tlmh_ip = parsed_cfg.tlm_dest_ip;
    net.tlml_ip = parsed_cfg.tlm_dest_ip;
    net.tlmh_port = parsed_cfg.tlmh_port;
    net.tlml_port = parsed_cfg.tlml_port;
    net.tlmh_period_ns = telemetry_interval_ms_to_period_ns(parsed_cfg.tlmh_interval_ms, 10000000L);
    net.tlml_period_ns = telemetry_interval_ms_to_period_ns(parsed_cfg.tlml_interval_ms, 100000000L);
    net.tlmh_hz = telemetry_interval_ms_to_rate_hz(parsed_cfg.tlmh_interval_ms, 100);
    net.tlml_hz = telemetry_interval_ms_to_rate_hz(parsed_cfg.tlml_interval_ms, 10);

    if (cli_overrides.has_tlmh_ip) {
      for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--tlmh-ip") && i + 1 < argc) {
          net.tlmh_ip = argv[i + 1];
        }
      }
    }
    if (cli_overrides.has_tlmh_port) {
      for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--tlmh-port") && i + 1 < argc) {
          net.tlmh_port = atoi(argv[i + 1]);
        }
      }
    }
    if (cli_overrides.has_tlmh_hz) {
      for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--tlmh-hz") && i + 1 < argc) {
          net.tlmh_hz = atoi(argv[i + 1]);
          if (net.tlmh_hz < 1) net.tlmh_hz = 1;
          net.tlmh_period_ns = telemetry_rate_hz_to_period_ns(net.tlmh_hz, 10000000L);
        }
      }
    }
    if (cli_overrides.has_tlml_ip) {
      for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--tlml-ip") && i + 1 < argc) {
          net.tlml_ip = argv[i + 1];
        }
      }
    }
    if (cli_overrides.has_tlml_port) {
      for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--tlml-port") && i + 1 < argc) {
          net.tlml_port = atoi(argv[i + 1]);
        }
      }
    }
    if (cli_overrides.has_tlml_hz) {
      for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--tlml-hz") && i + 1 < argc) {
          net.tlml_hz = atoi(argv[i + 1]);
          if (net.tlml_hz < 1) net.tlml_hz = 1;
          net.tlml_period_ns = telemetry_rate_hz_to_period_ns(net.tlml_hz, 100000000L);
        }
      }
    }

    net.sched.rt_enabled = parsed_cfg.rt_enabled ? 1 : 0;
    net.sched.rt_strict = parsed_cfg.rt_strict ? 1 : 0;
    net.sched.sched_policy = parsed_cfg.rt_sched_policy;
    net.sched.can_worker_priority = parsed_cfg.can_worker_priority;
    net.sched.tlmh_tx_priority = parsed_cfg.tlmh_tx_priority;
    net.sched.tlml_tx_priority = parsed_cfg.tlml_tx_priority;
    net.can_scan_max_node = parsed_cfg.can_scan_max_node;
    net.can_if = parsed_cfg.can_if;
    net.can_timeout_ms = (parsed_cfg.can_timeout_ms > 0) ? (uint32_t)parsed_cfg.can_timeout_ms : 0;
    sh.d.can_timeout_ms = net.can_timeout_ms;
    sh.d.can_scan_max_node = (uint8_t)((net.can_scan_max_node > 0) ? net.can_scan_max_node : 0);
    sh.d.strict_stale_policy = parsed_cfg.strict_stale_policy ? 1 : 0;

    apply_config_to_shared(&sh, &parsed_cfg);
  } else {
    parsed_cfg.num_axes = 1;
    parsed_cfg.axis_node_id[0] = net.node_id;
    parsed_cfg.axis_counts_per_rev[0] = sh.d.cfg[0].counts_per_rev;
    parsed_cfg.axis_gear_ratio[0] = sh.d.cfg[0].gear_ratio;
    parsed_cfg.axis_min_deg[0] = sh.d.cfg[0].az_min_deg;
    parsed_cfg.axis_max_deg[0] = sh.d.cfg[0].az_max_deg;
    parsed_cfg.axis_max_speed_deg_s[0] = sh.d.cfg[0].max_speed_deg_s;
    parsed_cfg.axis_storage_deg[0] = 0.0;
    parsed_cfg.axis_pp_profile_vel[0] = sh.d.cfg[0].pp_profile_vel;
    parsed_cfg.axis_diag_cat[0] = 0;
    parsed_cfg.axis_diag_subcat[0] = 0;
    parsed_cfg.turret_az_axis = 0;
    parsed_cfg.turret_el_axis = 0;

    apply_config_to_shared(&sh, &parsed_cfg);
  }

  char binary_hint[PATH_MAX];
  const char* binary_path = resolve_binary_hint(binary_hint, sizeof(binary_hint), argv[0]);
  if (enforce_rt_startup_policy(&net.sched, binary_path) != 0) {
    exit_code = 1;
    goto cleanup;
  }

  sh.d.can_timeout_ms = net.can_timeout_ms;
  sh.d.can_scan_max_node = (uint8_t)((net.can_scan_max_node > 0) ? net.can_scan_max_node : 0);

  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);

  stc_state_t next_state = STC_STATE_INIT;
  const stc_event_t pbit_evt = run_startup_pbit(&sh, &net);
  if (stc_state_transition(sh.d.st_state, pbit_evt, &next_state) == 0) {
    sh.d.st_state = next_state;
  } else {
    STC_LOG_ERROR("startup transition rejected: state=%d event=%d", (int)sh.d.st_state, (int)pbit_evt);
  }
  if (sh.d.st_state != STC_STATE_FAIL &&
      stc_state_transition(sh.d.st_state, EVT_ENTER_SAFE, &next_state) == 0) {
    STC_LOG_INFO("state transition: %s -> %s",
                 stc_state_to_string(sh.d.st_state),
                 stc_state_to_string(next_state));
    sh.d.st_state = next_state;
  }

  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();
  cfg.strict_magic = 0;    // allow us to return NACK even when magic is wrong
  cfg.strict_version = 0;

  actionq_t q;
  actionq_init(&q);

  const char* backend_name = select_backend_name(&parsed_cfg);
  if (!backend_name) {
    STC_LOG_ERROR("failed to select motion backend (requested=%s)", parsed_cfg.backend);
    exit_code = 1;
    goto cleanup;
  }
  STC_LOG_INFO("backend_requested=%s backend_effective=%s", parsed_cfg.backend, backend_name);
  if (parsed_cfg.num_axes > 0 && parsed_cfg.axis_enabled[0]) {
    STC_LOG_INFO("axis0 driver=%s node=%d",
                 axis_driver_to_string(parsed_cfg.axis_driver[0]),
                 parsed_cfg.axis_node_id[0]);
  }

  // CAN worker thread
  can_worker_arg_t carg = { .can_if = net.can_if, .can_bitrate = net.can_bitrate, .can_restart_ms = net.can_restart_ms, .can_scan_max_node = net.can_scan_max_node, .backend_name = backend_name, .primary_node_id = parsed_cfg.primary_node_id, .primary_tx_id = parsed_cfg.primary_tx_id, .primary_rx_id = parsed_cfg.primary_rx_id, .primary_motion_tx_id = parsed_cfg.primary_motion_tx_id, .primary_motion_rx_id = parsed_cfg.primary_motion_rx_id, .travel_min_deg = parsed_cfg.travel_min_deg, .travel_max_deg = parsed_cfg.travel_max_deg, .profile_max_speed_deg_s = parsed_cfg.profile_max_speed_deg_s, .profile_max_accel_deg_s2 = parsed_cfg.profile_max_accel_deg_s2, .telemetry_interval_ms = parsed_cfg.telemetry_interval_ms, .experimental_native_speed_mode = parsed_cfg.experimental_native_speed_mode, .sh = &sh, .q = &q };
  if (create_thread_with_sched(&th_can,
                               "can_worker_thread",
                               can_worker_thread,
                               &carg,
                               &net.sched,
                               net.sched.can_worker_priority) != 0) {
    STC_LOG_ERROR("failed to create CAN worker thread");
    exit_code = 1;
    goto cleanup;
  }
  can_thread_started = 1;
  threads_started = 1;

  // UDP telemetry threads
  udp_h = open_udp_tx();
  if (udp_h >= 0) udp_h_open = 1;
  udp_l = open_udp_tx();
  if (udp_l >= 0) udp_l_open = 1;
  if (udp_h < 0 || udp_l < 0) {
    STC_LOG_ERROR("udp socket error: %s", strerror(errno));
    exit_code = 1;
    goto cleanup;
  }

  tlm_tx_arg_t harg = { .udp_fd = udp_h, .config_ip = net.tlmh_ip, .port = net.tlmh_port, .period_ns = net.tlmh_period_ns, .sh = &sh, .cfg = cfg, .sender_id = 1 };
  tlm_tx_arg_t larg = { .udp_fd = udp_l, .config_ip = net.tlml_ip, .port = net.tlml_port, .period_ns = net.tlml_period_ns, .sh = &sh, .cfg = cfg, .sender_id = 1 };
  const int static_telemetry_configured = (net.tlmh_ip && net.tlml_ip &&
                                           net.tlmh_ip[0] != '\0' && net.tlml_ip[0] != '\0');
  harg.config_addr_valid = (parse_ipv4_addr(harg.config_ip, &harg.config_addr) == 0);
  larg.config_addr_valid = (parse_ipv4_addr(larg.config_ip, &larg.config_addr) == 0);
  if (create_thread_with_sched(&th_h,
                               "tlmh_tx_thread",
                               tlmh_tx_thread,
                               &harg,
                               &net.sched,
                               net.sched.tlmh_tx_priority) != 0) {
    STC_LOG_ERROR("failed to create TLMH thread");
    exit_code = 1;
    goto cleanup;
  }
  tlmh_thread_started = 1;
  if (create_thread_with_sched(&th_l,
                               "tlml_tx_thread",
                               tlml_tx_thread,
                               &larg,
                               &net.sched,
                               net.sched.tlml_tx_priority) != 0) {
    STC_LOG_ERROR("failed to create TLML thread");
    exit_code = 1;
    goto cleanup;
  }
  tlml_thread_started = 1;

  diag_server_arg_t diag_arg = { .sh = &sh, .port = net.diag_port };
  if (net.diag_port > 0) {
    if (pthread_create(&th_diag, NULL, diag_server_thread, &diag_arg) != 0) {
      STC_LOG_ERROR("failed to create diagnostics thread");
      exit_code = 1;
      goto cleanup;
    }
    diag_thread_started = 1;
  }

  // TCP server
  srv = socket(AF_INET, SOCK_STREAM, 0);
  if (srv < 0) {
    STC_LOG_ERROR("socket error: %s", strerror(errno));
    exit_code = 1;
    goto cleanup;
  }
  srv_open = 1;
  int yes = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons((uint16_t)net.tcp_port);
  if (bind(srv, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    STC_LOG_ERROR("bind error: %s", strerror(errno));
    exit_code = 1;
    goto cleanup;
  }
  if (listen(srv, 4) < 0) {
    STC_LOG_ERROR("listen error: %s", strerror(errno));
    exit_code = 1;
    goto cleanup;
  }
  atomic_store(&g_listener_fd, srv);

  STC_LOG_INFO("CAN worker on %s (multi-axis TPDO1/2/3 + RPDO1/2/3)", net.can_if);
  STC_LOG_INFO("STC server listening on TCP port %d", net.tcp_port);
  if (static_telemetry_configured) {
    STC_LOG_INFO("UDP telemetry static destination: TLMH %s:%d @%dHz | TLML %s:%d @%dHz", net.tlmh_ip, net.tlmh_port, net.tlmh_hz, net.tlml_ip, net.tlml_port, net.tlml_hz);
  } else {
    STC_LOG_INFO("UDP unicast telemetry: TLMH %s:%d @%dHz | TLML %s:%d @%dHz", net.tlmh_ip, net.tlmh_port, net.tlmh_hz, net.tlml_ip, net.tlml_port, net.tlml_hz);
  }

  while (!atomic_load(&g_stop)) {
    struct pollfd pfd = {.fd = srv, .events = POLLIN, .revents = 0};
    int prc = poll(&pfd, 1, 500);
    if (prc < 0) {
      if (errno == EINTR) continue;
      STC_LOG_WARN("poll error on listener: %s", strerror(errno));
      break;
    }
    if (prc == 0 || (pfd.revents & POLLIN) == 0) {
      continue;
    }

    struct sockaddr_in cli;
    socklen_t clen = sizeof(cli);
    int cfd = accept(srv, (struct sockaddr*)&cli, &clen);
    if (cfd < 0) {
      if (errno == EINTR) continue;
      if (atomic_load(&g_stop)) {
        break;
      }
      STC_LOG_WARN("accept error: %s", strerror(errno));
      continue;
    }
    atomic_store(&g_client_fd, cfd);

    char ipbuf[64];
    inet_ntop(AF_INET, &cli.sin_addr, ipbuf, sizeof(ipbuf));
    STC_LOG_INFO("Client connected: %s:%d", ipbuf, ntohs(cli.sin_port));

    pthread_mutex_lock(&sh.mtx);
    sh.d.active_cu_addr = cli.sin_addr;
    sh.d.active_cu_connected = 1;
    pthread_mutex_unlock(&sh.mtx);

    if (harg.config_addr_valid && harg.config_addr.s_addr == cli.sin_addr.s_addr) {
      STC_LOG_INFO("TLMH config IP matches active CU IP: %s", ipbuf);
    } else if (harg.config_addr_valid) {
      STC_LOG_WARN("TLMH config IP (%s) differs from active CU IP (%s)", harg.config_ip, ipbuf);
    }
    if (larg.config_addr_valid && larg.config_addr.s_addr == cli.sin_addr.s_addr) {
      STC_LOG_INFO("TLML config IP matches active CU IP: %s", ipbuf);
    } else if (larg.config_addr_valid) {
      STC_LOG_WARN("TLML config IP (%s) differs from active CU IP (%s)", larg.config_ip, ipbuf);
    }

    while (!atomic_load(&g_stop)) {
      uint8_t hdr[20];
      int rr = recv_exact(cfd, hdr, sizeof(hdr));
      if (rr == 0) {
        STC_LOG_INFO("Client closed");
        break;
      }
      if (rr < 0) {
        STC_LOG_WARN("recv header error: %s", strerror(errno));
        break;
      }

      uint16_t payload_size = (uint16_t)(hdr[18] | ((uint16_t)hdr[19] << 8));
      if (payload_size > STC_ICD_MAX_PAYLOAD) {
        // Drain and NACK.
        uint8_t junk[1024];
        size_t toread = payload_size;
        while (toread > 0) {
          size_t chunk = (toread > sizeof(junk)) ? sizeof(junk) : toread;
          int r2 = recv_exact(cfd, junk, chunk);
          if (r2 <= 0) break;
          toread -= chunk;
        }

        stc_icd_rsp_t rsp;
        memset(&rsp, 0, sizeof(rsp));
        rsp.version = cfg.protocol_version_default;
        rsp.cmd_id = (uint16_t)(hdr[6] | ((uint16_t)hdr[7] << 8));
        rsp.seq = (uint16_t)(hdr[8] | ((uint16_t)hdr[9] << 8));
        rsp.result = STC_RSP_NACK;
        rsp.err_code = STC_ERR_VALUE_OUT_OF_RANGE;
        rsp.payload_size = 0;
        uint8_t out[64];
        size_t out_len = 0;
        if (stc_icd_rsp_pack(&cfg, &rsp, out, sizeof(out), &out_len) == STC_ICD_OK) {
          (void)send_all(cfd, out, out_len);
        }
        continue;
      }

      uint8_t buf[20 + STC_ICD_MAX_PAYLOAD];
      memcpy(buf, hdr, 20);
      if (payload_size > 0) {
        rr = recv_exact(cfd, buf + 20, payload_size);
        if (rr <= 0) {
          STC_LOG_WARN("recv payload error: %s", strerror(errno));
          break;
        }
      }
      uint64_t tcp_recv_ns = mono_ns();

      stc_icd_cmd_t cmd;
      stc_icd_rc_t prc = stc_icd_cmd_parse(&cfg, buf, 20u + payload_size, &cmd);

      // Extract header fields for best-effort NACK on parse failure.
      uint16_t cmd_id = (uint16_t)(hdr[6] | ((uint16_t)hdr[7] << 8));
      uint16_t seq = (uint16_t)(hdr[8] | ((uint16_t)hdr[9] << 8));

      if (prc != STC_ICD_OK) {
        // classify error
        stc_nack_err_t ec = STC_ERR_GENERIC;
        int magic_ok = (hdr[0] == cfg.cmd_magic[0] && hdr[1] == cfg.cmd_magic[1] &&
                        hdr[2] == cfg.cmd_magic[2] && hdr[3] == cfg.cmd_magic[3]);
        if (!magic_ok) ec = STC_ERR_PROTOCOL_MISMATCH;
        else if (hdr[4] != cfg.protocol_version_default) ec = STC_ERR_PROTOCOL_MISMATCH;
        else if (!is_allowed_controller(hdr[5], &sh.d)) ec = STC_ERR_CLIENT_UNKNOWN;
        else if (cmd_id < STC_CMD_CU_STATE || cmd_id > STC_CMD_REQ_WORK_TIME || cmd_id == 0x0009) ec = STC_ERR_COMMAND_NOT_IMPLEMENTED;
        else ec = STC_ERR_SYNTAX_ERROR;

        stc_icd_rsp_t rsp;
        memset(&rsp, 0, sizeof(rsp));
        rsp.version = cfg.protocol_version_default;
        rsp.cmd_id = cmd_id;
        rsp.seq = seq;
        rsp.result = STC_RSP_NACK;
        rsp.err_code = ec;
        rsp.payload_size = 0;

        uint8_t out[64];
        size_t out_len = 0;
        if (stc_icd_rsp_pack(&cfg, &rsp, out, sizeof(out), &out_len) == STC_ICD_OK) {
          (void)send_all(cfd, out, out_len);
        }
        continue;
      }

      pthread_mutex_lock(&sh.mtx);
      memset(sh.d.saturated, 0, sizeof(sh.d.saturated));
      pthread_mutex_unlock(&sh.mtx);

      // Fast snapshot for validation (single mutex lock)
      shared_data_t snap;
      pthread_mutex_lock(&sh.mtx);
      snap = sh.d;
      pthread_mutex_unlock(&sh.mtx);

      print_cmd(&cmd);
      stc_nack_err_t v = validate_cmd_fast(&cfg, &snap, &cmd);

      // Fast side-effects that should be reflected in TLML immediately
      stc_action_t act;
      int has_action = 0;
      if (v == STC_ERR_NO_ERROR) {
        pthread_mutex_lock(&sh.mtx);
        if (cmd.payload_decoded) {
          if (cmd.cmd_id == STC_CMD_CU_STATE) {
            if (cmd.u.cu_state.state == sh.d.st_state) {
              v = STC_ERR_NO_ERROR;
            } else {
              stc_event_t evt;
              if (cmd.u.cu_state.state == STC_STATE_SAFE) evt = EVT_ENTER_SAFE;
              else if (cmd.u.cu_state.state == STC_STATE_OPER) evt = EVT_ENTER_OPER;
              else {
                v = STC_ERR_NOT_ALLOWED_STATE;
                evt = EVT_ENTER_SAFE;
              }
              if (v == STC_ERR_NO_ERROR && shared_transition_locked(&sh, evt) != 0) {
                v = STC_ERR_NOT_ALLOWED_STATE;
              }
            }
          } else if (cmd.cmd_id == STC_CMD_SELFLEV) {
            if (cmd.u.selflev.v == 0) {
              sh.d.selflev_pending = 0;
              sh.d.selflev_due_ms = 0;
              if (shared_transition_locked(&sh, EVT_ENTER_SAFE) != 0) v = STC_ERR_NOT_ALLOWED_STATE;
            } else if (shared_transition_locked(&sh, EVT_SELF_LEV) != 0) {
              v = STC_ERR_NOT_ALLOWED_STATE;
            } else {
              sh.d.selflev_pending = 1;
              sh.d.selflev_due_ms = mono_ms() + 100;
            }
          } else if (cmd.cmd_id == STC_CMD_IBIT) {
            if (cmd.u.ibit.v == 0) {
              sh.d.ibit_pending = 0;
              sh.d.ibit_due_ms = 0;
              if (shared_transition_locked(&sh, EVT_ENTER_SAFE) != 0) v = STC_ERR_NOT_ALLOWED_STATE;
            } else if (shared_start_ibit_locked(&sh, mono_ms()) != 0) {
              v = STC_ERR_NOT_ALLOWED_STATE;
            }
          } else if (cmd.cmd_id == STC_CMD_MANUALPOS) {
            if (sh.d.st_state != STC_STATE_SAFE && sh.d.st_state != STC_STATE_OPER) {
              v = STC_ERR_NOT_ALLOWED_STATE;
            } else {
              sh.d.lat = cmd.u.manualpos.lat;
              sh.d.lon = cmd.u.manualpos.lon;
              sh.d.alt = cmd.u.manualpos.alt;
              sh.d.heading = cmd.u.manualpos.heading;
              sh.d.has_manualpos = 1;
            }
          } else if (cmd.cmd_id == STC_CMD_POSMODE) {
            if (sh.d.st_state != STC_STATE_SAFE && sh.d.st_state != STC_STATE_OPER) {
              v = STC_ERR_NOT_ALLOWED_STATE;
            } else {
              sh.d.posmode = cmd.u.posmode.v;
            }
          } else if (cmd.cmd_id == STC_CMD_CONFIG) {
            /* ACK-only placeholder: no state mutation until CONFIG schema is defined. */
            has_action = 0;
          } else if (cmd.cmd_id == STC_CMD_LASER) {
            stc_action_make_set_laser(&act, cmd.u.laser.id, cmd.u.laser.on_off);
            has_action = 1;
          } else if (cmd.cmd_id == STC_CMD_TURRET || cmd.cmd_id == STC_CMD_SUT || cmd.cmd_id == STC_CMD_MIRROR) {
            if (!stc_state_can_move(sh.d.st_state)) {
              v = STC_ERR_NOT_ALLOWED_STATE;
            } else {
              uint32_t axes_mask = 0;
              double targets[STC_AXIS_COUNT] = {0};
              double speeds[STC_AXIS_COUNT] = {0};
              const uint64_t now_cmd = mono_ms();

              if (cmd.cmd_id == STC_CMD_TURRET) {
                const int az = sh.d.turret_az_axis;
                const int el = sh.d.turret_el_axis;
                axes_mask = turret_axes_mask(&sh.d);

                if (cmd.u.turret.motion_type == STC_MOTION_SPEED) {
                  if (az >= 0 && az < (int)STC_AXIS_COUNT) {
                    speeds[az] = (double)cmd.u.turret.az;
                    bool target_limited = false, speed_limited = false;
                    stc_clamp_axis_cmd(&sh.d.cfg[az], NULL, &speeds[az], &target_limited, &speed_limited);
                    mark_axis_saturated(&sh.d, az, speed_limited);
                  }
                  if (el >= 0 && el < (int)STC_AXIS_COUNT) {
                    speeds[el] = (double)cmd.u.turret.el;
                    bool target_limited = false, speed_limited = false;
                    stc_clamp_axis_cmd(&sh.d.cfg[el], NULL, &speeds[el], &target_limited, &speed_limited);
                    mark_axis_saturated(&sh.d, el, speed_limited);
                  }
                } else {
                  if (az >= 0 && az < (int)STC_AXIS_COUNT) {
                    targets[az] = (double)cmd.u.turret.az;
                    if (cmd.u.turret.motion_type == STC_MOTION_RELATIVE) {
                      targets[az] += axis_counts_to_deg(&sh.d.cfg[az], sh.d.telem[az].pos_raw);
                    }
                    const double requested_abs = targets[az];
                    bool target_limited = false, speed_limited = false;
                    stc_clamp_axis_cmd(&sh.d.cfg[az], &targets[az], NULL, &target_limited, &speed_limited);
                    log_axis_abs_saturation_if_needed(az, requested_abs, targets[az], &sh.d.cfg[az], target_limited);
                    mark_axis_saturated(&sh.d, az, target_limited);
                  }
                  if (el >= 0 && el < (int)STC_AXIS_COUNT) {
                    targets[el] = (double)cmd.u.turret.el;
                    if (cmd.u.turret.motion_type == STC_MOTION_RELATIVE) {
                      targets[el] += axis_counts_to_deg(&sh.d.cfg[el], sh.d.telem[el].pos_raw);
                    }
                    const double requested_abs = targets[el];
                    bool target_limited = false, speed_limited = false;
                    stc_clamp_axis_cmd(&sh.d.cfg[el], &targets[el], NULL, &target_limited, &speed_limited);
                    log_axis_abs_saturation_if_needed(el, requested_abs, targets[el], &sh.d.cfg[el], target_limited);
                    mark_axis_saturated(&sh.d, el, target_limited);
                  }
                }
              } else if (cmd.cmd_id == STC_CMD_SUT) {
                axes_mask = sut_axes_mask(&sh.d, cmd.u.sut.id);
                for (int i = 0; i < sh.d.sut_count; i++) {
                  if (cmd.u.sut.id != 0xFF && cmd.u.sut.id != (uint8_t)(i + 1)) continue;
                  int az = sh.d.sut_az_axis[i];
                  int el = sh.d.sut_el_axis[i];

                  if (cmd.u.sut.id == 0xFF && az >= 0 && !axis_is_operational(&sh.d, az)) {
                    STC_LOG_WARN("SUT All skipped axis %d (out-of-range/not-expected/disconnected/fault)", az);
                  }
                  if (cmd.u.sut.id == 0xFF && el >= 0 && !axis_is_operational(&sh.d, el)) {
                    STC_LOG_WARN("SUT All skipped axis %d (out-of-range/not-expected/disconnected/fault)", el);
                  }

                  if (az >= 0 && az < (int)STC_AXIS_COUNT) {
                    if (cmd.u.sut.id == 0xFF && !axis_is_operational(&sh.d, az)) continue;
                    if (cmd.u.sut.motion_type == STC_MOTION_SPEED) {
                      speeds[az] = (double)cmd.u.sut.az;
                      bool target_limited = false, speed_limited = false;
                      stc_clamp_axis_cmd(&sh.d.cfg[az], NULL, &speeds[az], &target_limited, &speed_limited);
                      mark_axis_saturated(&sh.d, az, speed_limited);
                    } else {
                      targets[az] = (double)cmd.u.sut.az;
                      if (cmd.u.sut.motion_type == STC_MOTION_RELATIVE) {
                        targets[az] += axis_counts_to_deg(&sh.d.cfg[az], sh.d.telem[az].pos_raw);
                      }
                      const double requested_abs = targets[az];
                      bool target_limited = false, speed_limited = false;
                      stc_clamp_axis_cmd(&sh.d.cfg[az], &targets[az], NULL, &target_limited, &speed_limited);
                      log_axis_abs_saturation_if_needed(az, requested_abs, targets[az], &sh.d.cfg[az], target_limited);
                      mark_axis_saturated(&sh.d, az, target_limited);
                    }
                  }
                  if (el >= 0 && el < (int)STC_AXIS_COUNT) {
                    if (cmd.u.sut.id == 0xFF && !axis_is_operational(&sh.d, el)) continue;
                    if (cmd.u.sut.motion_type == STC_MOTION_SPEED) {
                      speeds[el] = (double)cmd.u.sut.el;
                      bool target_limited = false, speed_limited = false;
                      stc_clamp_axis_cmd(&sh.d.cfg[el], NULL, &speeds[el], &target_limited, &speed_limited);
                      mark_axis_saturated(&sh.d, el, speed_limited);
                    } else {
                      targets[el] = (double)cmd.u.sut.el;
                      if (cmd.u.sut.motion_type == STC_MOTION_RELATIVE) {
                        targets[el] += axis_counts_to_deg(&sh.d.cfg[el], sh.d.telem[el].pos_raw);
                      }
                      const double requested_abs = targets[el];
                      bool target_limited = false, speed_limited = false;
                      stc_clamp_axis_cmd(&sh.d.cfg[el], &targets[el], NULL, &target_limited, &speed_limited);
                      log_axis_abs_saturation_if_needed(el, requested_abs, targets[el], &sh.d.cfg[el], target_limited);
                      mark_axis_saturated(&sh.d, el, target_limited);
                    }
                  }
                }
              } else if (cmd.cmd_id == STC_CMD_MIRROR) {
                axes_mask = mirror_axes_mask(&sh.d, cmd.u.mirror.mirror_id);
                for (int i = 0; i < sh.d.mirror_count; i++) {
                  if (cmd.u.mirror.mirror_id != 0xFF && cmd.u.mirror.mirror_id != (uint8_t)(i + 1)) continue;
                  int axis = sh.d.mirror_axis[i];
                  if (cmd.u.mirror.mirror_id == 0xFF && axis >= 0 && !axis_is_operational(&sh.d, axis)) {
                    STC_LOG_WARN("Mirror All skipped axis %d (out-of-range/not-expected/disconnected/fault)", axis);
                  }
                  if (axis < 0 || axis >= (int)STC_AXIS_COUNT) continue;
                  if (cmd.u.mirror.mirror_id == 0xFF && !axis_is_operational(&sh.d, axis)) continue;
                  if (cmd.u.mirror.motion_type == STC_MOTION_SPEED) {
                    speeds[axis] = (double)cmd.u.mirror.elevation;
                    bool target_limited = false, speed_limited = false;
                    stc_clamp_axis_cmd(&sh.d.cfg[axis], NULL, &speeds[axis], &target_limited, &speed_limited);
                    mark_axis_saturated(&sh.d, axis, speed_limited);
                  } else {
                    targets[axis] = (double)cmd.u.mirror.elevation;
                    if (cmd.u.mirror.motion_type == STC_MOTION_RELATIVE) {
                      targets[axis] += axis_counts_to_deg(&sh.d.cfg[axis], sh.d.telem[axis].pos_raw);
                    }
                    const double requested_abs = targets[axis];
                    bool target_limited = false, speed_limited = false;
                    stc_clamp_axis_cmd(&sh.d.cfg[axis], &targets[axis], NULL, &target_limited, &speed_limited);
                    log_axis_abs_saturation_if_needed(axis, requested_abs, targets[axis], &sh.d.cfg[axis], target_limited);
                    mark_axis_saturated(&sh.d, axis, target_limited);
                  }
                }
              }

              if (command_targets_unmapped_axis(&sh.d, &cmd)) {
                STC_LOG_WARN("requested motion target is not configured");
                v = STC_ERR_NOT_CONFIGURED;
              } else if (axes_mask == 0) {
                v = STC_ERR_VALUE_OUT_OF_RANGE;
              } else if (cmd.cmd_id == STC_CMD_TURRET || cmd.cmd_id == STC_CMD_SUT || cmd.cmd_id == STC_CMD_MIRROR) {
                if ((cmd.cmd_id == STC_CMD_TURRET && cmd.u.turret.motion_type == STC_MOTION_SPEED) ||
                    (cmd.cmd_id == STC_CMD_SUT && cmd.u.sut.motion_type == STC_MOTION_SPEED) ||
                    (cmd.cmd_id == STC_CMD_MIRROR && cmd.u.mirror.motion_type == STC_MOTION_SPEED)) {
                  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
                    if ((axes_mask & (1u << axis)) == 0) continue;
                    sh.d.stream[axis].desired_speed_deg_s = speeds[axis];
                    sh.d.stream[axis].last_cmd_ms = now_cmd;
                    sh.d.stream[axis].enabled = 1;
                  }
                  stc_action_make_stream_speed(&act, axes_mask, speeds);
                } else {
                  stc_axis_motion_kind_t kinds[STC_AXIS_COUNT];
                  double values[STC_AXIS_COUNT];
                  double speed_limits[STC_AXIS_COUNT];
                  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
                    kinds[axis] = STC_AXIS_MOTION_ABS;
                    values[axis] = targets[axis];
                    speed_limits[axis] = sh.d.cfg[axis].max_speed_deg_s;
                  }
                  if ((cmd.cmd_id == STC_CMD_TURRET && cmd.u.turret.motion_type == STC_MOTION_RELATIVE) ||
                      (cmd.cmd_id == STC_CMD_SUT && cmd.u.sut.motion_type == STC_MOTION_RELATIVE) ||
                      (cmd.cmd_id == STC_CMD_MIRROR && cmd.u.mirror.motion_type == STC_MOTION_RELATIVE)) {
                    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) if (axes_mask & (1u << axis)) {
                      kinds[axis] = STC_AXIS_MOTION_REL;
                      values[axis] = (cmd.cmd_id == STC_CMD_MIRROR) ? cmd.u.mirror.elevation :
                                     (cmd.cmd_id == STC_CMD_TURRET ? ((axis == (size_t)sh.d.turret_az_axis) ? cmd.u.turret.az : cmd.u.turret.el) :
                                      values[axis] - axis_counts_to_deg(&sh.d.cfg[axis], sh.d.telem[axis].pos_raw));
                    }
                  }
                  stc_action_make_motion(&act, axes_mask, kinds, values, speed_limits);
                }
                has_action = 1;
              }
            }
          }
        } else {
          if (cmd.cmd_id == STC_CMD_RESET) {
            if (sh.d.st_state != STC_STATE_FAIL) {
              v = STC_ERR_NOT_ALLOWED_STATE;
            } else {
              stc_event_t reset_evt = EVT_IBIT_COMPLETE_FAIL;
              if (shared_start_ibit_locked(&sh, mono_ms()) != 0) {
                v = STC_ERR_NOT_ALLOWED_STATE;
              } else {
                reset_evt = run_admin_reset_ibit(&sh, &net);
                if (shared_transition_locked(&sh, reset_evt) != 0) {
                  STC_LOG_ERROR("RESET/IBIT transition rejected: state=%d event=%d",
                                (int)sh.d.st_state,
                                (int)reset_evt);
                  v = STC_ERR_NOT_ALLOWED_STATE;
                } else {
                  stc_action_make_simple(&act, ACTION_RESET, 0xFFFFFFFFu);
                  has_action = 1;
                }
              }
            }
          } else if (cmd.cmd_id == STC_CMD_DELLOG) {
            stc_action_make_simple(&act, ACTION_CLEAR_ERRORS, 0xFFFFFFFFu);
            has_action = 1;
          } else if (cmd.cmd_id == STC_CMD_FACTORY) {
            stc_action_make_simple(&act, ACTION_FACTORY, 0u);
            has_action = 1;
          } else if (cmd.cmd_id == STC_CMD_SELFTEST) {
            stc_action_make_simple(&act, ACTION_CAN_REDISCOVER, 0xFFFFFFFFu);
            has_action = 1;
          } else if (cmd.cmd_id == STC_CMD_SHUTDOWN) {
            stc_action_make_simple(&act, ACTION_SHUTDOWN, 0u);
            has_action = 1;
          } else if (cmd.cmd_id == STC_CMD_STORAGE) {
            if (shared_transition_locked(&sh, EVT_STORE) != 0) {
              v = STC_ERR_NOT_ALLOWED_STATE;
            } else {
              stc_action_make_simple(&act, ACTION_ENTER_STORE, 0xFFFFFFFFu);
              has_action = 1;
            }
          }
        }
        pthread_mutex_unlock(&sh.mtx);
      }

      if (v == STC_ERR_NO_ERROR && has_action) {
        act.obs_tcp_rx_ns = tcp_recv_ns;
        act.obs_queued_ns = mono_ns();
        if (actionq_push(&q, &act) != 0) {
          v = STC_ERR_INSUFFICIENT_RES;
        } else {
          size_t qdepth = actionq_count(&q);
          pthread_mutex_lock(&sh.mtx);
          if ((uint64_t)qdepth > sh.d.action_queue_depth_peak) {
            sh.d.action_queue_depth_peak = (uint64_t)qdepth;
          }
          pthread_mutex_unlock(&sh.mtx);
        }
      }

      // Build response
      stc_icd_rsp_t rsp;
      memset(&rsp, 0, sizeof(rsp));
      rsp.version = cfg.protocol_version_default;
      rsp.cmd_id = cmd.cmd_id;
      rsp.seq = cmd.seq;

      if (v == STC_ERR_NO_ERROR) {
        rsp.result = STC_RSP_ACK;
        rsp.err_code = 0;
      } else {
        rsp.result = STC_RSP_NACK;
        rsp.err_code = v;
      }

      if (v == STC_ERR_NO_ERROR && cmd.cmd_id == STC_CMD_REQ_SW_VERSION) {
        rsp.payload_size = (uint16_t)strlen(STC_SW_VERSION);
        memcpy(rsp.payload, STC_SW_VERSION, rsp.payload_size);
      } else if (v == STC_ERR_NO_ERROR && cmd.cmd_id == STC_CMD_REQ_WORK_TIME) {
        uint32_t uptime = 0;
        pthread_mutex_lock(&sh.mtx);
        uptime = (uint32_t)((mono_ms() - sh.d.boot_ms) / 1000ULL);
        pthread_mutex_unlock(&sh.mtx);
        memcpy(rsp.payload, &uptime, sizeof(uptime));
        rsp.payload_size = (uint16_t)sizeof(uptime);
      } else if (v == STC_ERR_NO_ERROR && cmd.cmd_id == STC_CMD_REQ_HW_INFO) {
        char hw_info[96];
        snprintf(hw_info,
                 sizeof(hw_info),
                 "STC v%s; axis0 driver=myact_x8_25 node=1 model=X8-PRO- can=can0",
                 STC_SW_VERSION);
        size_t n = strnlen(hw_info, sizeof(hw_info) - 1);
        memcpy(rsp.payload, hw_info, n + 1);
        rsp.payload_size = (uint16_t)(n + 1);
      } else {
        rsp.payload_size = 0;
      }

      uint8_t out[1024];
      size_t out_len = 0;
      if (stc_icd_rsp_pack(&cfg, &rsp, out, sizeof(out), &out_len) != STC_ICD_OK) {
        STC_LOG_ERROR("rsp_pack failed");
        break;
      }
      if (send_all(cfd, out, out_len) != 0) {
        STC_LOG_WARN("send rsp error: %s", strerror(errno));
        break;
      }
      if (rsp.result == STC_RSP_ACK) {
        STC_LOG_INFO("TCP ACK sent: command accepted cmd=0x%02X seq=%u", rsp.cmd_id, rsp.seq);
      }
    }

    close(cfd);
    atomic_store(&g_client_fd, -1);
    pthread_mutex_lock(&sh.mtx);
    sh.d.active_cu_connected = 0;
    if (!static_telemetry_configured) {
      sh.d.active_cu_addr.s_addr = htonl(INADDR_ANY);
    }
    pthread_mutex_unlock(&sh.mtx);
    if (static_telemetry_configured) {
      STC_LOG_INFO("Client disconnected; static telemetry destination preserved");
    } else {
      STC_LOG_INFO("Client disconnected; telemetry destination cleared until CU reconnects");
    }
  }

cleanup:
  if (atomic_load(&g_stop)) {
    STC_LOG_INFO("Shutdown signal received; stopping STC server");
  }
  if (threads_started) {
    atomic_store(&g_stop, 1);
  }

  if (srv_open) {
    const int listener_fd = atomic_exchange(&g_listener_fd, -1);
    if (listener_fd >= 0) {
      close(listener_fd);
    }
  }
  const int client_fd = atomic_exchange(&g_client_fd, -1);
  if (client_fd >= 0) {
    close(client_fd);
  }

  if (tlmh_thread_started) pthread_join(th_h, NULL);
  if (tlml_thread_started) pthread_join(th_l, NULL);
  if (diag_thread_started) pthread_join(th_diag, NULL);
  if (can_thread_started) pthread_join(th_can, NULL);

  if (udp_h_open) close(udp_h);
  if (udp_l_open) close(udp_l);

  stc_log_close();
  return exit_code;
}
