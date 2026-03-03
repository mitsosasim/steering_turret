// stc_server.c
//
// STC runtime daemon (TCP CMD/RSP <-> CANopen PD4-C) + UDP multicast telemetry.
//
// Architecture:
//  - TCP thread (main): receive CMD, parse/validate, perform fast side-effects (state/manualpos),
//    and enqueue discrete actions for CAN worker.
//  - CAN worker thread: owns pd4c_ctx_t, executes queued commands, runs safety watchdog,
//    polls TPDOs and updates shared telemetry.
//  - Telemetry threads: publish TLMH @100Hz and TLML @10Hz over UDP multicast.
//
/* Build example:
 *   gcc -std=c11 -O2 -Wall -Wextra -pthread \
 *       stc_icd.c pd4c.c stc_server.c -o stc_server -lm
 */

#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L

#include "stc_icd.h"
#include "stc_config.h"
#include "stc_diag.h"
#include "stc_model.h"
#include "pd4c.h"
#include "stc_actions.h"
#include "stc_version.h"
#include "stc_limits.h"
#include "stc_state.h"
#include "stc_faults.h"

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
#include <time.h>
#include <unistd.h>
#include <math.h>

/* ----------------------------- time helpers ----------------------------- */

static uint64_t mono_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static uint64_t wall_ms(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}

static void sleep_until(struct timespec* next, long period_ns) {
  next->tv_nsec += period_ns;
  while (next->tv_nsec >= 1000000000L) {
    next->tv_nsec -= 1000000000L;
    next->tv_sec += 1;
  }
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL);
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
      if (errno == EINTR) continue;
      return -1;
    }
    off += (size_t)r;
  }
  return 1;
}

static int open_udp_tx(void) {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return -1;

  unsigned char ttl = 1;
  setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

  unsigned char loop = 1;  // allow local testing
  setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

  return fd;
}

static int udp_send_to(int fd, const char* mcast_ip, int port, const uint8_t* buf, size_t len) {
  struct sockaddr_in dst;
  memset(&dst, 0, sizeof(dst));
  dst.sin_family = AF_INET;
  dst.sin_port = htons((uint16_t)port);
  if (inet_pton(AF_INET, mcast_ip, &dst.sin_addr) != 1) return -1;

  ssize_t s = sendto(fd, buf, len, 0, (struct sockaddr*)&dst, sizeof(dst));
  return (s == (ssize_t)len) ? 0 : -1;
}

/* ----------------------------- shared state ----------------------------- */

typedef struct {
  // --- per-axis config/telemetry/stream (axis0 = turret az active in step3)
  axis_cfg_t cfg[STC_AXIS_COUNT];
  uint8_t axis_node_id[STC_AXIS_COUNT];
  axis_telem_t telem[STC_AXIS_COUNT];
  axis_stream_t stream[STC_AXIS_COUNT];
  bool saturated[STC_AXIS_COUNT];
  double storage_deg[STC_AXIS_COUNT];
  uint8_t diag_cat[STC_AXIS_COUNT];
  uint8_t diag_subcat[STC_AXIS_COUNT];
  int8_t motor_map[8];

  int turret_az_axis;
  int turret_el_axis;
  int sut_count;
  int sut_az_axis[STC_MAX_SUT];
  int sut_el_axis[STC_MAX_SUT];
  int mirror_count;
  int mirror_axis[STC_MAX_MIRROR];

  int speed_watchdog_ms;
  int ds402_timeout_ms;
  int pp_sdo_diag;              // enable SDO snapshot before PP move
  uint32_t can_timeout_ms;

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
  uint64_t last_rx_ms[STC_AXIS_COUNT];
  uint8_t axis_fault_active[STC_AXIS_COUNT];
  uint8_t axis_enabled[STC_AXIS_COUNT];

  // --- telemetry seq + uptime
  uint16_t seq_h;
  uint16_t seq_l;
  uint64_t boot_ms;
} shared_data_t;

typedef struct {
  pthread_mutex_t mtx;
  shared_data_t d;
} shared_t;

/* ----------------------------- signal handling ----------------------------- */

static volatile sig_atomic_t g_stop = 0;

static void on_sig(int s) {
  (void)s;
  g_stop = 1;
}

/* ----------------------------- ICD validation ----------------------------- */

static int is_allowed_controller(uint8_t cid) {
  // ICD list: 11, 13, 15, 21
  return (cid == 11 || cid == 13 || cid == 15 || cid == 21);
}

static stc_nack_err_t validate_cmd_fast(const stc_icd_cfg_t* cfg,
                                       const shared_data_t* sh,
                                       const stc_icd_cmd_t* c) {
  (void)sh;
  // magic
  if (!(c->magic[0] == cfg->cmd_magic[0] && c->magic[1] == cfg->cmd_magic[1] &&
        c->magic[2] == cfg->cmd_magic[2] && c->magic[3] == cfg->cmd_magic[3])) {
    return STC_ERR_PROTOCOL_MISMATCH;
  }
  if (c->version != cfg->protocol_version_default) return STC_ERR_PROTOCOL_MISMATCH;
  if (!is_allowed_controller(c->controller_id)) return STC_ERR_CLIENT_UNKNOWN;

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
    case STC_CMD_FACTORY:
    case STC_CMD_SHUTDOWN:
    case STC_CMD_CONFIG:
    case STC_CMD_MANUALPOS:
    case STC_CMD_POSMODE:
    case STC_CMD_REQ_SW_VERSION:
    case STC_CMD_REQ_HW_INFO:
    case STC_CMD_REQ_WORK_TIME:
      break;
    default:
      return STC_ERR_SYNTAX_ERROR;
  }

  if (c->cmd_id == STC_CMD_CONFIG) return STC_ERR_SYNTAX_ERROR;

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
  if (c->cmd_id == STC_CMD_LASER) {
    if (c->u.laser.id < 1 || c->u.laser.id > 8) return STC_ERR_VALUE_OUT_OF_RANGE;
    if (c->u.laser.on_off > 1) return STC_ERR_VALUE_OUT_OF_RANGE;
  }

  return STC_ERR_NO_ERROR;
}


static int shared_transition_locked(shared_t* sh, stc_event_t evt) {
  stc_state_t next = sh->d.st_state;
  if (stc_state_transition(sh->d.st_state, evt, &next) != 0) return -1;
  sh->d.st_state = next;
  if (!stc_state_can_stream(sh->d.st_state)) {
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      sh->d.stream[axis].enabled = 0;
    }
  }
  return 0;
}

static void mark_axis_saturated(shared_data_t* sh, int axis, int limited) {
  if (!limited) return;
  if (axis < 0 || axis >= (int)STC_AXIS_COUNT) return;
  sh->saturated[axis] = true;
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
      if (az >= 0 && az < (int)STC_AXIS_COUNT) mask |= (1u << az);
      if (el >= 0 && el < (int)STC_AXIS_COUNT) mask |= (1u << el);
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
      if (axis >= 0 && axis < (int)STC_AXIS_COUNT) mask |= (1u << axis);
    }
    return mask;
  }

  int idx = (int)id - 1;
  if (idx < 0 || idx >= sh->mirror_count) return 0;
  if (sh->mirror_axis[idx] >= 0 && sh->mirror_axis[idx] < (int)STC_AXIS_COUNT) mask |= (1u << sh->mirror_axis[idx]);
  return mask;
}

static void print_cmd(const stc_icd_cmd_t* c) {
  printf("CMD: ver=%u ctrl=%u cmd_id=0x%04X seq=%u ts=%llu payload=%u\n",
         c->version, c->controller_id, c->cmd_id, c->seq,
         (unsigned long long)c->timestamp_ms, c->payload_size);

  if (!c->payload_decoded) {
    if (c->payload_size) printf("  payload: (raw %u bytes)\n", c->payload_size);
    return;
  }
  switch (c->cmd_id) {
    case STC_CMD_CU_STATE:
      printf("  CU_STATE=%u\n", c->u.cu_state.state);
      break;
    case STC_CMD_SELFLEV:
      printf("  SELFLEV start_stop=%u\n", c->u.selflev.v);
      break;
    case STC_CMD_IBIT:
      printf("  IBIT start_stop=%u\n", c->u.ibit.v);
      break;
    case STC_CMD_STORAGE:
      printf("  STORAGE\n");
      break;
    case STC_CMD_LASER:
      printf("  LASER id=%u on_off=%u\n", c->u.laser.id, c->u.laser.on_off);
      break;
    case STC_CMD_TURRET:
      printf("  TURRET motion=%u az=%f el=%f\n", c->u.turret.motion_type, c->u.turret.az, c->u.turret.el);
      break;
    case STC_CMD_MIRROR:
      printf("  MIRROR id=%u motion=%u elev=%f\n", c->u.mirror.mirror_id, c->u.mirror.motion_type, c->u.mirror.elevation);
      break;
    case STC_CMD_SUT:
      printf("  SUT id=%u motion=%u az=%f el=%f\n", c->u.sut.id, c->u.sut.motion_type, c->u.sut.az, c->u.sut.el);
      break;
    case STC_CMD_MANUALPOS:
      printf("  MANUALPOS lat=%lf lon=%lf alt=%lf heading=%lf (encoded_as_double=%d)\n",
             c->u.manualpos.lat, c->u.manualpos.lon, c->u.manualpos.alt, c->u.manualpos.heading,
             c->u.manualpos.encoded_as_double);
      break;
    case STC_CMD_POSMODE:
      printf("  POSMODE mode=%u\n", c->u.posmode.v);
      break;
    case STC_CMD_REQ_SW_VERSION:
      printf("  REQ_SW_VERSION\n");
      break;
    case STC_CMD_RESET:
      printf("  RESET\n");
      break;
    case STC_CMD_FACTORY:
      printf("  FACTORY\n");
      break;
    case STC_CMD_SHUTDOWN:
      printf("  SHUTDOWN\n");
      break;
    default:
      printf("  (decoded)\n");
      break;
  }
}

/* ----------------------------- CAN worker ----------------------------- */

typedef enum {
  MTR_MODE_NONE = 0,
  MTR_MODE_PV   = PD4C_MODE_PROFILE_VELOCITY,
  MTR_MODE_PP   = PD4C_MODE_PROFILE_POSITION
} mtr_mode_t;

typedef struct {
  const char* can_if;
  shared_t* sh;
  actionq_t* q;
} can_worker_arg_t;

static void copy_pd4c_telem_into_shared(shared_t* sh, size_t axis, const pd4c_telemetry_t* t, uint64_t now_ms) {
  // Caller holds sh->mtx
  axis_telem_update(&sh->d.telem[axis],
                    &sh->d.cfg[axis],
                    t->pos_actual,
                    t->vel_actual,
                    t->statusword,
                    t->mode_display,
                    now_ms);
  if (axis == 0) {
    sh->d.hb_state    = (uint8_t)t->hb_state;
    sh->d.have_hb     = t->have_hb ? 1 : 0;

    sh->d.have_emcy   = t->have_emcy ? 1 : 0;
    sh->d.emcy_code   = t->emcy_error_code;
    sh->d.emcy_reg    = t->emcy_error_reg;

    sh->d.ts_tpdo1_ms = t->ts_tpdo1_ms;
    sh->d.ts_tpdo2_ms = t->ts_tpdo2_ms;
    sh->d.ts_tpdo3_ms = t->ts_tpdo3_ms;
    sh->d.ts_hb_ms    = t->ts_hb_ms;
    sh->d.ts_emcy_ms  = t->ts_emcy_ms;
  }
}

static void motor_stop_and_disable(pd4c_ctx_t* ctx, mtr_mode_t mode) {
  if (!ctx) return;
  (void)pd4c_ds402_quick_stop(ctx, (int8_t)mode);
  usleep(20000);
  (void)pd4c_rpdo1_send_control_mode(ctx, 0x0006, (int8_t)mode);
}

static int motor_ensure_enabled(pd4c_ctx_t* ctx, shared_t* sh, mtr_mode_t mode, mtr_mode_t* cur_mode, int* enabled) {
  if (!ctx || !sh || !cur_mode || !enabled) return PD4C_EINVAL;
  if (*enabled && *cur_mode == mode) return PD4C_OK;

  int rc = pd4c_ds402_enable_operation(ctx, (int8_t)mode, sh->d.ds402_timeout_ms);
  if (rc == PD4C_OK) {
    *cur_mode = mode;
    *enabled = 1;
  }
  return rc;
}

static int motor_do_pp_move(pd4c_ctx_t* ctx,
                            shared_t* sh,
                            size_t axis,
                            double target_deg,
                            mtr_mode_t* cur_mode,
                            int* enabled) {
  if (!ctx || !sh) return PD4C_EINVAL;

  int rc = motor_ensure_enabled(ctx, sh, MTR_MODE_PP, cur_mode, enabled);
  if (rc != PD4C_OK) return rc;

  int32_t cur_pos_raw = 0;
  double counts_per_rev = 0.0;
  double gear_ratio = 0.0;
  uint16_t pp_profile_vel = 0;
  int run_sdo_diag = 0;
  pthread_mutex_lock(&sh->mtx);
  cur_pos_raw = sh->d.telem[axis].pos_raw;
  counts_per_rev = sh->d.cfg[axis].counts_per_rev;
  gear_ratio = sh->d.cfg[axis].gear_ratio;
  pp_profile_vel = sh->d.cfg[axis].pp_profile_vel;
  run_sdo_diag = sh->d.pp_sdo_diag;
  pthread_mutex_unlock(&sh->mtx);

  // Convert turret degrees -> motor counts and call low-level PP handshake.
  const int32_t target_counts = axis_deg_to_counts(&sh->d.cfg[axis], target_deg);
  const uint16_t pv = pp_profile_vel;
  const double cur_deg = axis_counts_to_deg(&sh->d.cfg[axis], cur_pos_raw);

  fprintf(stderr,
          "PP move request: axis=%zu requested_az_deg=%.3f target_counts=%d current_pos_raw=%d current_deg=%.3f "
          "counts_per_rev=%.3f gear_ratio=%.3f pp_profile_vel=%u\n",
          axis,
          target_deg,
          target_counts,
          cur_pos_raw,
          cur_deg,
          counts_per_rev,
          gear_ratio,
          (unsigned)pv);

  if (run_sdo_diag) {
    uint8_t mode_disp = 0;
    int32_t min_pos_lim = 0;
    int32_t max_pos_lim = 0;
    uint32_t home_method = 0;
    int32_t home_offset = 0;
    int32_t home_speed_fast = 0;
    int32_t home_speed_slow = 0;

    int rc_mode = pd4c_sdo_read_u8(ctx, 0x6061, 0x00, &mode_disp);
    int rc_min_lim = pd4c_sdo_read_i32(ctx, 0x607D, 0x01, &min_pos_lim);
    int rc_max_lim = pd4c_sdo_read_i32(ctx, 0x607D, 0x02, &max_pos_lim);
    int rc_home_method = pd4c_sdo_read_u32(ctx, 0x6098, 0x00, &home_method);
    int rc_home_offset = pd4c_sdo_read_i32(ctx, 0x607C, 0x00, &home_offset);
    int rc_home_fast = pd4c_sdo_read_i32(ctx, 0x6099, 0x01, &home_speed_fast);
    int rc_home_slow = pd4c_sdo_read_i32(ctx, 0x6099, 0x02, &home_speed_slow);

    fprintf(stderr,
            "PP pre-move SDO diag: 6061:0 mode_display rc=%d val=%u | "
            "607D:1 min_pos_limit rc=%d val=%d | 607D:2 max_pos_limit rc=%d val=%d | "
            "6098:0 homing_method rc=%d val=%u | 607C:0 home_offset rc=%d val=%d | "
            "6099:1 home_speed_fast rc=%d val=%d | 6099:2 home_speed_slow rc=%d val=%d\n",
            rc_mode,
            (unsigned)mode_disp,
            rc_min_lim,
            min_pos_lim,
            rc_max_lim,
            max_pos_lim,
            rc_home_method,
            home_method,
            rc_home_offset,
            home_offset,
            rc_home_fast,
            home_speed_fast,
            rc_home_slow,
            home_speed_slow);
  }

  // NOTE: stc_server already resolves ABS vs REL at a higher level:
  // - ABS: target_deg is absolute angle
  // - REL: you already computed absolute target (cur + delta) before calling motor_do_pp_move()
  // Therefore we call pd4c_pp_move as absolute (is_relative=0).
  const int is_relative = 0;

  rc = pd4c_pp_move(ctx, target_counts, pv, is_relative, sh->d.ds402_timeout_ms);
  return rc;
}



static void* can_worker_thread(void* arg) {
  can_worker_arg_t* a = (can_worker_arg_t*)arg;
  pd4c_ctx_t pd_ctx[STC_AXIS_COUNT];
  int axis_connected[STC_AXIS_COUNT] = {0};
  mtr_mode_t axis_mode[STC_AXIS_COUNT] = {0};
  int axis_enabled[STC_AXIS_COUNT] = {0};

  memset(pd_ctx, 0, sizeof(pd_ctx));

  int any_connected = 0;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    uint8_t node_id;
    pthread_mutex_lock(&a->sh->mtx);
    node_id = a->sh->d.axis_node_id[axis];
    pthread_mutex_unlock(&a->sh->mtx);

    if (node_id == 0) continue;

    int rc = pd4c_open(&pd_ctx[axis], a->can_if, node_id);
    if (rc == PD4C_OK) {
      (void)pd4c_nmt_start(&pd_ctx[axis]);
      axis_connected[axis] = 1;
      any_connected = 1;
      fprintf(stderr, "CAN axis %zu connected on %s (node=%u)\n", axis, a->can_if, (unsigned)node_id);
    } else {
      fprintf(stderr, "pd4c_open(%s,node=%u) axis=%zu failed: %s\n",
              a->can_if,
              (unsigned)node_id,
              axis,
              pd4c_strerror(rc));
    }
  }

  pthread_mutex_lock(&a->sh->mtx);
  a->sh->d.can_ok = any_connected;
  pthread_mutex_unlock(&a->sh->mtx);

  while (!g_stop) {
    uint64_t now = mono_ms();

    struct pollfd pfds[STC_AXIS_COUNT];
    size_t axis_idx[STC_AXIS_COUNT];
    nfds_t nfds = 0;

    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      if (!axis_connected[axis]) continue;
      pfds[nfds].fd = pd_ctx[axis].sock;
      pfds[nfds].events = POLLIN;
      pfds[nfds].revents = 0;
      axis_idx[nfds] = axis;
      nfds++;
    }

    if (nfds > 0) {
      int prc = poll(pfds, nfds, 10);
      if (prc > 0) {
        for (nfds_t j = 0; j < nfds; j++) {
          if ((pfds[j].revents & POLLIN) == 0) continue;
          size_t axis = axis_idx[j];
          int ev = pd4c_poll(&pd_ctx[axis], 0);
          if (ev >= 0) {
            const pd4c_telemetry_t* t = pd4c_telem(&pd_ctx[axis]);
            axis_mode[axis] = (mtr_mode_t)t->mode_display;
            pthread_mutex_lock(&a->sh->mtx);
            copy_pd4c_telem_into_shared(a->sh, axis, t, now);
            a->sh->d.last_rx_ms[axis] = now;
            pthread_mutex_unlock(&a->sh->mtx);
          }
        }
      }
    }

    int enter_fail = 0;
    pthread_mutex_lock(&a->sh->mtx);
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      if (!axis_connected[axis]) continue;
      if (stc_detect_ds402_fault(a->sh->d.telem[axis].status_word)) {
        a->sh->d.axis_fault_active[axis] = 1;
        enter_fail = 1;
      }
      if (stc_detect_can_timeout(now, a->sh->d.last_rx_ms[axis], a->sh->d.can_timeout_ms)) {
        a->sh->d.axis_fault_active[axis] = 1;
        enter_fail = 1;
      }
    }
    if (enter_fail) {
      (void)shared_transition_locked(a->sh, EVT_FAULT_CRITICAL);
    }
    stc_state_t post_fault_state = a->sh->d.st_state;
    pthread_mutex_unlock(&a->sh->mtx);

    if (post_fault_state == STC_STATE_FAIL) {
      for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
        if (axis_connected[axis]) {
          (void)pd4c_ds402_quick_stop(&pd_ctx[axis], (int8_t)axis_mode[axis]);
          (void)pd4c_rpdo1_send_control_mode(&pd_ctx[axis], 0x0006, (int8_t)axis_mode[axis]);
          axis_enabled[axis] = 0;
        }
      }
    }

    stc_action_t act;
    while (actionq_try_peek(a->q, &act)) {
      pthread_mutex_lock(&a->sh->mtx);
      stc_state_t queue_state = a->sh->d.st_state;
      pthread_mutex_unlock(&a->sh->mtx);
      if (queue_state == STC_STATE_FAIL && act.type != ACTION_RESET) {
        (void)actionq_drop_head(a->q);
        continue;
      }
      switch (act.type) {
        case ACTION_SET_STATE:
          break;
        case ACTION_PP_MOVE:
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            if ((act.axes_mask & (1u << axis)) == 0 || !axis_connected[axis]) continue;
            pthread_mutex_lock(&a->sh->mtx);
            a->sh->d.stream[axis].enabled = 0;
            pthread_mutex_unlock(&a->sh->mtx);

            int rc = motor_do_pp_move(&pd_ctx[axis], a->sh, axis, act.u.target_deg[axis], &axis_mode[axis], &axis_enabled[axis]);
            fprintf(stderr,
                    "PP_MOVE axis=%zu pd4c_pp_move=%d latest_statusword=0x%04X\n",
                    axis,
                    rc,
                    pd4c_telem(&pd_ctx[axis])->statusword);
          }
          break;
        case ACTION_STREAM_SPEED:
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            if ((act.axes_mask & (1u << axis)) == 0 || !axis_connected[axis]) continue;
            pthread_mutex_lock(&a->sh->mtx);
            stc_state_t action_state = a->sh->d.st_state;
            pthread_mutex_unlock(&a->sh->mtx);
            if (!stc_state_can_stream(action_state)) continue;
            (void)motor_ensure_enabled(&pd_ctx[axis], a->sh, MTR_MODE_PV, &axis_mode[axis], &axis_enabled[axis]);
            int clipped = 0;
            int32_t tv_raw = axis_deg_s_to_target_vel_units(&a->sh->d.cfg[axis], act.u.speed_deg_s[axis], &clipped);
            if (tv_raw > INT16_MAX) tv_raw = INT16_MAX;
            if (tv_raw < INT16_MIN) tv_raw = INT16_MIN;
            (void)pd4c_rpdo3_send_target_velocity(&pd_ctx[axis], (int16_t)tv_raw);
          }
          break;
        case ACTION_STOP:
        case ACTION_QUICK_STOP:
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            if ((act.axes_mask & (1u << axis)) == 0 || !axis_connected[axis]) continue;
            (void)pd4c_ds402_quick_stop(&pd_ctx[axis], (int8_t)axis_mode[axis]);
            axis_enabled[axis] = 0;
          }
          break;
        case ACTION_ENTER_STORE:
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            if ((act.axes_mask & (1u << axis)) == 0 || !axis_connected[axis]) continue;
            (void)pd4c_sdo_write_u32(&pd_ctx[axis], 0x1010, 0x01, 0x65766173UL);
          }
          break;
        case ACTION_RESET: {
          int reset_ok = 0;
          pthread_mutex_lock(&a->sh->mtx);
          reset_ok = (shared_transition_locked(a->sh, EVT_RESET) == 0);
          if (reset_ok) {
            memset(a->sh->d.axis_fault_active, 0, sizeof(a->sh->d.axis_fault_active));
          }
          pthread_mutex_unlock(&a->sh->mtx);
          if (!reset_ok) break;
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            if ((act.axes_mask & (1u << axis)) == 0 || !axis_connected[axis]) continue;
            (void)pd4c_ds402_quick_stop(&pd_ctx[axis], (int8_t)axis_mode[axis]);
            (void)pd4c_ds402_fault_reset(&pd_ctx[axis], (int8_t)axis_mode[axis], a->sh->d.ds402_timeout_ms);
            (void)pd4c_nmt_start(&pd_ctx[axis]);
            axis_enabled[axis] = 0;
          }
          break;
        }
        case ACTION_FACTORY:
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            if ((act.axes_mask & (1u << axis)) == 0 || !axis_connected[axis]) continue;
            (void)pd4c_nmt_stop(&pd_ctx[axis]);
            axis_enabled[axis] = 0;
            axis_mode[axis] = MTR_MODE_NONE;
          }
          break;
        case ACTION_SHUTDOWN:
          for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
            if ((act.axes_mask & (1u << axis)) == 0 || !axis_connected[axis]) continue;
            pthread_mutex_lock(&a->sh->mtx);
            a->sh->d.stream[axis].enabled = 0;
            pthread_mutex_unlock(&a->sh->mtx);
            motor_stop_and_disable(&pd_ctx[axis], axis_mode[axis]);
            (void)pd4c_nmt_stop(&pd_ctx[axis]);
            axis_enabled[axis] = 0;
            axis_mode[axis] = MTR_MODE_NONE;
          }
          break;
        default:
          break;
      }

      (void)actionq_drop_head(a->q);
    }

    stc_state_t st_state = STC_STATE_INIT;
    int watchdog_ms = 0;
    pthread_mutex_lock(&a->sh->mtx);
    st_state = a->sh->d.st_state;
    watchdog_ms = a->sh->d.speed_watchdog_ms;
    pthread_mutex_unlock(&a->sh->mtx);

    if (!stc_state_can_stream(st_state)) {
      for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
        if (!axis_connected[axis]) continue;
        pthread_mutex_lock(&a->sh->mtx);
        int speed_on = a->sh->d.stream[axis].enabled;
        a->sh->d.stream[axis].enabled = 0;
        pthread_mutex_unlock(&a->sh->mtx);
        if (speed_on) {
          motor_stop_and_disable(&pd_ctx[axis], axis_mode[axis]);
          axis_enabled[axis] = 0;
        }
      }
    }

    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      if (!axis_connected[axis] || !stc_state_can_stream(st_state)) continue;

      int speed_on = 0;
      double desired_deg_s = 0.0;
      uint64_t last_cmd_ms = 0;
      pthread_mutex_lock(&a->sh->mtx);
      speed_on = a->sh->d.stream[axis].enabled;
      desired_deg_s = a->sh->d.stream[axis].desired_speed_deg_s;
      last_cmd_ms = a->sh->d.stream[axis].last_cmd_ms;
      pthread_mutex_unlock(&a->sh->mtx);

      if (!speed_on) continue;

      if (watchdog_ms > 0 && (int)(now - last_cmd_ms) > watchdog_ms) {
        pthread_mutex_lock(&a->sh->mtx);
        a->sh->d.stream[axis].enabled = 0;
        pthread_mutex_unlock(&a->sh->mtx);
        (void)pd4c_ds402_quick_stop(&pd_ctx[axis], (int8_t)axis_mode[axis]);
        axis_enabled[axis] = 0;
      } else {
        (void)motor_ensure_enabled(&pd_ctx[axis], a->sh, MTR_MODE_PV, &axis_mode[axis], &axis_enabled[axis]);
        int clipped = 0;
        int32_t tv_raw = axis_deg_s_to_target_vel_units(&a->sh->d.cfg[axis], desired_deg_s, &clipped);
        if (tv_raw > INT16_MAX) tv_raw = INT16_MAX;
        if (tv_raw < INT16_MIN) tv_raw = INT16_MIN;
        (void)pd4c_rpdo3_send_target_velocity(&pd_ctx[axis], (int16_t)tv_raw);
      }
    }

    pthread_mutex_lock(&a->sh->mtx);
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      a->sh->d.axis_enabled[axis] = axis_enabled[axis] ? 1u : 0u;
    }
    pthread_mutex_unlock(&a->sh->mtx);
  }

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (axis_connected[axis]) pd4c_close(&pd_ctx[axis]);
  }
  return NULL;
}

/* ----------------------------- telemetry threads ----------------------------- */

typedef struct {
  int udp_fd;
  const char* ip;
  int port;
  int rate_hz;
  shared_t* sh;
  stc_icd_cfg_t cfg;
  uint8_t sender_id;
} tlm_tx_arg_t;

static void* tlmh_tx_thread(void* arg) {
  tlm_tx_arg_t* a = (tlm_tx_arg_t*)arg;
  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  const long period_ns = (a->rate_hz > 0) ? (1000000000L / a->rate_hz) : 10000000L;

  while (!g_stop) {
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
      (void)udp_send_to(a->udp_fd, a->ip, a->port, buf, blen);
    }
    sleep_until(&next, period_ns);
  }
  return NULL;
}

static void* tlml_tx_thread(void* arg) {
  tlm_tx_arg_t* a = (tlm_tx_arg_t*)arg;
  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  const long period_ns = (a->rate_hz > 0) ? (1000000000L / a->rate_hz) : 100000000L;

  while (!g_stop) {
    stc_tlml_t m;
    memset(&m, 0, sizeof(m));
    memcpy(m.magic, a->cfg.tlml_magic, 4);
    m.version = a->cfg.protocol_version_default;
    m.controller_id = a->sender_id;

    uint64_t nowm = mono_ms();
    uint64_t noww = wall_ms();

    pthread_mutex_lock(&a->sh->mtx);
    m.seq = a->sh->d.seq_l++;
    m.state = (uint8_t)a->sh->d.st_state;
    uint64_t uptime_ms = nowm - a->sh->d.boot_ms;
    m.uptime_sec = (uint32_t)(uptime_ms / 1000ULL);
    m.lat = a->sh->d.has_manualpos ? a->sh->d.lat : 0.0;
    m.lon = a->sh->d.has_manualpos ? a->sh->d.lon : 0.0;
    m.alt_m = a->sh->d.has_manualpos ? a->sh->d.alt : 0.0;
    m.heading_deg = a->sh->d.has_manualpos ? a->sh->d.heading : 0.0;

    bool saturated[STC_AXIS_COUNT] = {false};
    uint8_t axis_fault_active[STC_AXIS_COUNT] = {0};
    uint8_t axis_enabled[STC_AXIS_COUNT] = {0};
    uint64_t last_rx_ms[STC_AXIS_COUNT] = {0};
    uint8_t diag_cat[STC_AXIS_COUNT] = {0};
    uint8_t diag_subcat[STC_AXIS_COUNT] = {0};
    int8_t motor_map[8] = {-1,-1,-1,-1,-1,-1,-1,-1};

    for (size_t i = 0; i < STC_AXIS_COUNT; i++) {
      saturated[i] = a->sh->d.saturated[i];
      axis_fault_active[i] = a->sh->d.axis_fault_active[i];
      axis_enabled[i] = a->sh->d.axis_enabled[i];
      last_rx_ms[i] = a->sh->d.last_rx_ms[i];
      diag_cat[i] = a->sh->d.diag_cat[i];
      diag_subcat[i] = a->sh->d.diag_subcat[i];
      a->sh->d.saturated[i] = false;
    }
    memcpy(motor_map, a->sh->d.motor_map, sizeof(motor_map));
    uint32_t can_timeout_ms = a->sh->d.can_timeout_ms;
    pthread_mutex_unlock(&a->sh->mtx);

    m.timestamp_ms = noww;
    m.temperature_c = 0.0f;
    m.humidity_pct = 0.0f;
    m.pressure_hpa = 0.0f;
    m.level_x = 0.0f;
    m.level_y = 0.0f;

    for (int j = 0; j < 8; j++) {
      int ax = motor_map[j];
      if (ax < 0 || ax >= (int)STC_AXIS_COUNT) {
        m.motor_status[j] = 0;
        continue;
      }

      uint8_t flags = 0;
      if (axis_enabled[ax]) flags |= (1u << 0);
      if (axis_fault_active[ax]) flags |= (1u << 1);
      if (nowm - last_rx_ms[ax] > can_timeout_ms) flags |= (1u << 2);
      if (saturated[ax]) flags |= (1u << 3);
      m.motor_status[j] = flags;
    }

    memset(m.laser_status, 0, sizeof(m.laser_status));

    enum {
      STC_DIAG_ERR_AXIS_FAULT = 0x0001,
      STC_DIAG_ERR_AXIS_STALE = 0x0002,
      STC_DIAG_WARN_AXIS_SAT = 0x0100,
    };

    m.error_count = 0;
    for (size_t i = 0; i < STC_AXIS_COUNT && m.error_count < STC_ICD_MAX_ERRORS; i++) {
      const int stale = (nowm - last_rx_ms[i] > can_timeout_ms);
      if (axis_fault_active[i]) {
        m.errors[m.error_count++] = stc_diag_encode(diag_cat[i], diag_subcat[i], STC_DIAG_ERR_AXIS_FAULT);
      }
      if (stale && m.error_count < STC_ICD_MAX_ERRORS) {
        m.errors[m.error_count++] = stc_diag_encode(diag_cat[i], diag_subcat[i], STC_DIAG_ERR_AXIS_STALE);
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
      (void)udp_send_to(a->udp_fd, a->ip, a->port, buf, blen);
    }
    sleep_until(&next, period_ns);
  }
  return NULL;
}

/* ----------------------------- TCP server loop ----------------------------- */

static void usage(const char* p) {
  fprintf(stderr,
          "Usage: %s [--tcp-port 7000] [--can can0] [--node 1]\n"
          "          [--config dev/config/stc_default.ini]\n"
          "          [--counts-per-rev N] [--gear-ratio N] [--vel-scale N]\n"
          "          [--az-min DEG] [--az-max DEG] [--max-speed-deg-s X]\n"
          "          [--speed-watchdog-ms MS] [--pp-profile-vel U16]\n"
          "          [--ds402-timeout-ms MS] [--pp-sdo-diag 0|1]\n"
          "          [--tlmh-ip 239.255.10.2 --tlmh-port 6010 --tlmh-hz 100]\n"
          "          [--tlml-ip 239.255.10.3 --tlml-port 6020 --tlml-hz 10]\n",
          p);
}

typedef struct {
  const char* can_if;
  int node_id;
  int tcp_port;
  const char* tlmh_ip;
  int tlmh_port;
  int tlmh_hz;
  const char* tlml_ip;
  int tlml_port;
  int tlml_hz;
} runtime_net_t;

static void apply_config_to_shared(shared_t* sh, const stc_config_t* cfg) {
  for (int i = 0; i < cfg->num_axes && i < (int)STC_AXIS_COUNT; i++) {
    sh->d.axis_node_id[i] = (uint8_t)cfg->axis_node_id[i];
    sh->d.cfg[i].counts_per_rev = cfg->axis_counts_per_rev[i];
    sh->d.cfg[i].gear_ratio = cfg->axis_gear_ratio[i];
    sh->d.cfg[i].vel_raw_units = cfg->axis_vel_is_rpm[i] ? AXIS_VEL_RAW_RPM : AXIS_VEL_RAW_DEG_PER_SEC;
    sh->d.cfg[i].vel_scale = 1.0;
    sh->d.cfg[i].az_min_deg = cfg->axis_min_deg[i];
    sh->d.cfg[i].az_max_deg = cfg->axis_max_deg[i];
    sh->d.cfg[i].max_speed_deg_s = cfg->axis_max_speed_deg_s[i];
    sh->d.cfg[i].pp_profile_vel = cfg->axis_pp_profile_vel[i];

    sh->d.storage_deg[i] = cfg->axis_storage_deg[i];
    sh->d.diag_cat[i] = cfg->axis_diag_cat[i];
    sh->d.diag_subcat[i] = cfg->axis_diag_subcat[i];
  }

  sh->d.turret_az_axis = cfg->turret_az_axis;
  sh->d.turret_el_axis = cfg->turret_el_axis;

  sh->d.sut_count = cfg->sut_count;
  memcpy(sh->d.sut_az_axis, cfg->sut_az_axis, sizeof(sh->d.sut_az_axis));
  memcpy(sh->d.sut_el_axis, cfg->sut_el_axis, sizeof(sh->d.sut_el_axis));

  sh->d.mirror_count = cfg->mirror_count;
  memcpy(sh->d.mirror_axis, cfg->mirror_axis, sizeof(sh->d.mirror_axis));
  memcpy(sh->d.motor_map, cfg->motor_status_map, sizeof(sh->d.motor_map));
}

int main(int argc, char** argv) {
  runtime_net_t net = {
    .can_if = "can0",
    .node_id = 1,
    .tcp_port = 7000,
    .tlmh_ip = "239.255.10.2",
    .tlmh_port = 6010,
    .tlmh_hz = 100,
    .tlml_ip = "239.255.10.3",
    .tlml_port = 6020,
    .tlml_hz = 10,
  };
  const char* config_path = NULL;
  int legacy_cli_mode = 0;

  stc_config_t parsed_cfg;
  stc_config_set_defaults(&parsed_cfg);

  shared_t sh;
  memset(&sh, 0, sizeof(sh));
  pthread_mutex_init(&sh.mtx, NULL);
  memset(&sh.d.cfg, 0, sizeof(sh.d.cfg));
  memset(&sh.d.telem, 0, sizeof(sh.d.telem));
  memset(&sh.d.stream, 0, sizeof(sh.d.stream));
  sh.d.boot_ms = mono_ms();

  // defaults (axis 0 from CLI; axes 1..17 remain zero/disabled in step3)
  sh.d.cfg[0].counts_per_rev = 4096.0;
  sh.d.cfg[0].gear_ratio = 1.0;
  sh.d.cfg[0].vel_raw_units = AXIS_VEL_RAW_DEG_PER_SEC;
  sh.d.cfg[0].vel_scale = 1.0;
  sh.d.cfg[0].az_min_deg = -180.0;
  sh.d.cfg[0].az_max_deg = 180.0;
  sh.d.cfg[0].max_speed_deg_s = 90.0;
  sh.d.speed_watchdog_ms = 250;
  sh.d.cfg[0].pp_profile_vel = 500;       // default profile velocity
  sh.d.ds402_timeout_ms = 2000;
  sh.d.pp_sdo_diag = 0;
  sh.d.can_timeout_ms = 1000;
  sh.d.st_state = STC_STATE_INIT;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    sh.d.last_rx_ms[axis] = sh.d.boot_ms;
    sh.d.axis_enabled[axis] = 0;
    sh.d.axis_fault_active[axis] = 0;
  }

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--can") && i + 1 < argc) net.can_if = argv[++i];
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
    else if (!strcmp(argv[i], "--can-timeout-ms") && i + 1 < argc) sh.d.can_timeout_ms = (uint32_t)atoi(argv[++i]);
    else if (!strcmp(argv[i], "--tlmh-ip") && i + 1 < argc) {
      net.tlmh_ip = argv[++i];
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlmh-port") && i + 1 < argc) {
      net.tlmh_port = atoi(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlmh-hz") && i + 1 < argc) {
      net.tlmh_hz = atoi(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlml-ip") && i + 1 < argc) {
      net.tlml_ip = argv[++i];
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlml-port") && i + 1 < argc) {
      net.tlml_port = atoi(argv[++i]);
      legacy_cli_mode = 1;
    }
    else if (!strcmp(argv[i], "--tlml-hz") && i + 1 < argc) {
      net.tlml_hz = atoi(argv[++i]);
      legacy_cli_mode = 1;
    }
    else {
      usage(argv[0]);
      return 2;
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
      fprintf(stderr, "Failed to load config file: %s\n", config_path);
      return 1;
    }

    net.node_id = parsed_cfg.axis_node_id[0];
    net.tcp_port = parsed_cfg.tcp_port;
    net.tlmh_ip = parsed_cfg.mcast_ip;
    net.tlml_ip = parsed_cfg.mcast_ip;
    net.tlmh_port = parsed_cfg.mcast_tlmh_port;
    net.tlml_port = parsed_cfg.mcast_tlml_port;
    net.tlmh_hz = (parsed_cfg.tlmh_interval_ms > 0) ? (1000 / parsed_cfg.tlmh_interval_ms) : 100;
    net.tlml_hz = (parsed_cfg.tlml_interval_ms > 0) ? (1000 / parsed_cfg.tlml_interval_ms) : 10;

    apply_config_to_shared(&sh, &parsed_cfg);
  } else {
    parsed_cfg.num_axes = 1;
    parsed_cfg.axis_node_id[0] = net.node_id;
    parsed_cfg.axis_counts_per_rev[0] = sh.d.cfg[0].counts_per_rev;
    parsed_cfg.axis_gear_ratio[0] = sh.d.cfg[0].gear_ratio;
    parsed_cfg.axis_vel_is_rpm[0] = (sh.d.cfg[0].vel_raw_units == AXIS_VEL_RAW_RPM) ? 1 : 0;
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

  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);

  stc_state_t next_state = STC_STATE_INIT;
  if (stc_state_transition(sh.d.st_state, EVT_INIT_COMPLETE, &next_state) == 0) sh.d.st_state = next_state;
  if (stc_state_transition(sh.d.st_state, EVT_ENTER_SAFE, &next_state) == 0) sh.d.st_state = next_state;

  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();
  cfg.strict_magic = 0;    // allow us to return NACK even when magic is wrong
  cfg.strict_version = 0;

  actionq_t q;
  actionq_init(&q);

  // CAN worker thread
  pthread_t th_can;
  can_worker_arg_t carg = { .can_if = net.can_if, .sh = &sh, .q = &q };
  pthread_create(&th_can, NULL, can_worker_thread, &carg);

  // UDP telemetry threads
  int udp_h = open_udp_tx();
  int udp_l = open_udp_tx();
  if (udp_h < 0 || udp_l < 0) {
    perror("udp socket");
    return 1;
  }

  pthread_t th_h, th_l;
  tlm_tx_arg_t harg = { .udp_fd = udp_h, .ip = net.tlmh_ip, .port = net.tlmh_port, .rate_hz = net.tlmh_hz, .sh = &sh, .cfg = cfg, .sender_id = 1 };
  tlm_tx_arg_t larg = { .udp_fd = udp_l, .ip = net.tlml_ip, .port = net.tlml_port, .rate_hz = net.tlml_hz, .sh = &sh, .cfg = cfg, .sender_id = 1 };
  pthread_create(&th_h, NULL, tlmh_tx_thread, &harg);
  pthread_create(&th_l, NULL, tlml_tx_thread, &larg);

  // TCP server
  int srv = socket(AF_INET, SOCK_STREAM, 0);
  if (srv < 0) {
    perror("socket");
    return 1;
  }
  int yes = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons((uint16_t)net.tcp_port);
  if (bind(srv, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("bind");
    return 1;
  }
  if (listen(srv, 4) < 0) {
    perror("listen");
    return 1;
  }

  printf("CAN worker on %s (multi-axis TPDO1/2/3 + RPDO1/2/3)\n", net.can_if);
  printf("STC server listening on TCP port %d\n", net.tcp_port);
  printf("UDP multicast: TLMH %s:%d @%dHz | TLML %s:%d @%dHz\n", net.tlmh_ip, net.tlmh_port, net.tlmh_hz, net.tlml_ip, net.tlml_port, net.tlml_hz);

  while (!g_stop) {
    struct sockaddr_in cli;
    socklen_t clen = sizeof(cli);
    int cfd = accept(srv, (struct sockaddr*)&cli, &clen);
    if (cfd < 0) {
      if (errno == EINTR) continue;
      perror("accept");
      break;
    }

    char ipbuf[64];
    inet_ntop(AF_INET, &cli.sin_addr, ipbuf, sizeof(ipbuf));
    printf("Client connected: %s:%d\n", ipbuf, ntohs(cli.sin_port));

    while (!g_stop) {
      uint8_t hdr[20];
      int rr = recv_exact(cfd, hdr, sizeof(hdr));
      if (rr == 0) {
        printf("Client closed\n");
        break;
      }
      if (rr < 0) {
        perror("recv header");
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
        rsp.err_code = STC_ERR_SYNTAX_ERROR;
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
          perror("recv payload");
          break;
        }
      }

      stc_icd_cmd_t cmd;
      stc_icd_rc_t prc = stc_icd_cmd_parse(&cfg, buf, 20u + payload_size, &cmd);

      // Extract header fields for best-effort NACK on parse failure.
      uint16_t cmd_id = (uint16_t)(hdr[6] | ((uint16_t)hdr[7] << 8));
      uint16_t seq = (uint16_t)(hdr[8] | ((uint16_t)hdr[9] << 8));

      if (prc != STC_ICD_OK) {
        // classify error
        stc_nack_err_t ec = STC_ERR_SYNTAX_ERROR;
        int magic_ok = (hdr[0] == cfg.cmd_magic[0] && hdr[1] == cfg.cmd_magic[1] &&
                        hdr[2] == cfg.cmd_magic[2] && hdr[3] == cfg.cmd_magic[3]);
        if (!magic_ok) ec = STC_ERR_PROTOCOL_MISMATCH;
        else if (hdr[4] != cfg.protocol_version_default) ec = STC_ERR_PROTOCOL_MISMATCH;
        else if (!is_allowed_controller(hdr[5])) ec = STC_ERR_CLIENT_UNKNOWN;

        stc_icd_rsp_t rsp;
        memset(&rsp, 0, sizeof(rsp));
        rsp.version = cfg.protocol_version_default;
        rsp.cmd_id = cmd_id;
        rsp.seq = seq;
        rsp.result = STC_RSP_NACK;
        rsp.err_code = (uint8_t)ec;
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
          } else if (cmd.cmd_id == STC_CMD_SELFLEV) {
            if (cmd.u.selflev.v == 0) {
              if (shared_transition_locked(&sh, EVT_ENTER_SAFE) != 0) v = STC_ERR_NOT_ALLOWED_STATE;
            } else if (shared_transition_locked(&sh, EVT_SELF_LEV) != 0) {
              v = STC_ERR_NOT_ALLOWED_STATE;
            }
          } else if (cmd.cmd_id == STC_CMD_IBIT) {
            if (cmd.u.ibit.v == 0) {
              if (shared_transition_locked(&sh, EVT_ENTER_SAFE) != 0) v = STC_ERR_NOT_ALLOWED_STATE;
            } else if (shared_transition_locked(&sh, EVT_IBIT) != 0) {
              v = STC_ERR_NOT_ALLOWED_STATE;
            }
          } else if (cmd.cmd_id == STC_CMD_MANUALPOS) {
            sh.d.lat = cmd.u.manualpos.lat;
            sh.d.lon = cmd.u.manualpos.lon;
            sh.d.alt = cmd.u.manualpos.alt;
            sh.d.heading = cmd.u.manualpos.heading;
            sh.d.has_manualpos = 1;
          } else if (cmd.cmd_id == STC_CMD_POSMODE) {
            sh.d.posmode = cmd.u.posmode.v;
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
                    bool target_limited = false, speed_limited = false;
                    stc_clamp_axis_cmd(&sh.d.cfg[az], &targets[az], NULL, &target_limited, &speed_limited);
                    mark_axis_saturated(&sh.d, az, target_limited);
                  }
                  if (el >= 0 && el < (int)STC_AXIS_COUNT) {
                    targets[el] = (double)cmd.u.turret.el;
                    if (cmd.u.turret.motion_type == STC_MOTION_RELATIVE) {
                      targets[el] += axis_counts_to_deg(&sh.d.cfg[el], sh.d.telem[el].pos_raw);
                    }
                    bool target_limited = false, speed_limited = false;
                    stc_clamp_axis_cmd(&sh.d.cfg[el], &targets[el], NULL, &target_limited, &speed_limited);
                    mark_axis_saturated(&sh.d, el, target_limited);
                  }
                }
              } else if (cmd.cmd_id == STC_CMD_SUT) {
                axes_mask = sut_axes_mask(&sh.d, cmd.u.sut.id);
                for (int i = 0; i < sh.d.sut_count; i++) {
                  if (cmd.u.sut.id != 0xFF && cmd.u.sut.id != (uint8_t)(i + 1)) continue;
                  int az = sh.d.sut_az_axis[i];
                  int el = sh.d.sut_el_axis[i];
                  if (az >= 0 && az < (int)STC_AXIS_COUNT) {
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
                      bool target_limited = false, speed_limited = false;
                      stc_clamp_axis_cmd(&sh.d.cfg[az], &targets[az], NULL, &target_limited, &speed_limited);
                      mark_axis_saturated(&sh.d, az, target_limited);
                    }
                  }
                  if (el >= 0 && el < (int)STC_AXIS_COUNT) {
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
                      bool target_limited = false, speed_limited = false;
                      stc_clamp_axis_cmd(&sh.d.cfg[el], &targets[el], NULL, &target_limited, &speed_limited);
                      mark_axis_saturated(&sh.d, el, target_limited);
                    }
                  }
                }
              } else if (cmd.cmd_id == STC_CMD_MIRROR) {
                axes_mask = mirror_axes_mask(&sh.d, cmd.u.mirror.mirror_id);
                for (int i = 0; i < sh.d.mirror_count; i++) {
                  if (cmd.u.mirror.mirror_id != 0xFF && cmd.u.mirror.mirror_id != (uint8_t)(i + 1)) continue;
                  int axis = sh.d.mirror_axis[i];
                  if (axis < 0 || axis >= (int)STC_AXIS_COUNT) continue;
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
                    bool target_limited = false, speed_limited = false;
                    stc_clamp_axis_cmd(&sh.d.cfg[axis], &targets[axis], NULL, &target_limited, &speed_limited);
                    mark_axis_saturated(&sh.d, axis, target_limited);
                  }
                }
              }

              if (axes_mask == 0) {
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
                  stc_action_make_pp_move(&act, axes_mask, targets);
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
              stc_action_make_simple(&act, ACTION_RESET, 0xFFFFFFFFu);
              has_action = 1;
            }
          } else if (cmd.cmd_id == STC_CMD_FACTORY) {
            stc_action_make_simple(&act, ACTION_FACTORY, 0u);
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
        if (actionq_push(&q, &act) != 0) v = STC_ERR_INSUFFICIENT_RES;
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
        rsp.err_code = (uint8_t)v;
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
        snprintf(hw_info, sizeof(hw_info), "Steering Turret Controller v%s", STC_SW_VERSION);
        size_t n = strnlen(hw_info, sizeof(hw_info) - 1);
        memcpy(rsp.payload, hw_info, n + 1);
        rsp.payload_size = (uint16_t)(n + 1);
      } else {
        rsp.payload_size = 0;
      }

      uint8_t out[1024];
      size_t out_len = 0;
      if (stc_icd_rsp_pack(&cfg, &rsp, out, sizeof(out), &out_len) != STC_ICD_OK) {
        fprintf(stderr, "rsp_pack failed\n");
        break;
      }
      if (send_all(cfd, out, out_len) != 0) {
        perror("send rsp");
        break;
      }
    }

    close(cfd);
  }

  close(srv);
  g_stop = 1;

  pthread_join(th_h, NULL);
  pthread_join(th_l, NULL);
  pthread_join(th_can, NULL);

  close(udp_h);
  close(udp_l);
  return 0;
}
