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
#include "stc_model.h"
#include "pd4c.h"
#include "stc_version.h"

#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
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
  axis_telem_t telem[STC_AXIS_COUNT];
  axis_stream_t stream[STC_AXIS_COUNT];

  int speed_watchdog_ms;
  int ds402_timeout_ms;
  int pp_sdo_diag;              // enable SDO snapshot before PP move

  // --- software state / navigation
  uint8_t cu_state;             // from CU_STATE command
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

  // --- telemetry seq + uptime
  uint16_t seq_h;
  uint16_t seq_l;
  uint64_t boot_ms;
} shared_data_t;

typedef struct {
  pthread_mutex_t mtx;
  shared_data_t d;
} shared_t;

static double pos_counts_to_deg(const shared_data_t* sh, int32_t counts) {
  return axis_counts_to_deg(&sh->cfg[0], counts);
}

static int32_t deg_to_counts(const shared_data_t* sh, double deg) {
  return axis_deg_to_counts(&sh->cfg[0], deg);
}

static int16_t degs_to_target_vel_units(const shared_data_t* sh, double deg_s, int* clipped) {
  int32_t raw = axis_deg_s_to_target_vel_units(&sh->cfg[0], deg_s, clipped);
  if (raw > INT16_MAX) {
    if (clipped) *clipped = 1;
    raw = INT16_MAX;
  } else if (raw < INT16_MIN) {
    if (clipped) *clipped = 1;
    raw = INT16_MIN;
  }
  return (int16_t)raw;
}

/* ----------------------------- command queue ----------------------------- */

typedef struct {
  stc_icd_cmd_t items[128];
  size_t head;
  size_t tail;
  size_t count;
  pthread_mutex_t mtx;
  pthread_cond_t cv;
} cmdq_t;

static void cmdq_init(cmdq_t* q) {
  memset(q, 0, sizeof(*q));
  pthread_mutex_init(&q->mtx, NULL);
  pthread_cond_init(&q->cv, NULL);
}

static int cmdq_push(cmdq_t* q, const stc_icd_cmd_t* cmd) {
  pthread_mutex_lock(&q->mtx);
  if (q->count >= (sizeof(q->items) / sizeof(q->items[0]))) {
    pthread_mutex_unlock(&q->mtx);
    return -1;
  }
  q->items[q->tail] = *cmd;
  q->tail = (q->tail + 1) % (sizeof(q->items) / sizeof(q->items[0]));
  q->count++;
  pthread_cond_signal(&q->cv);
  pthread_mutex_unlock(&q->mtx);
  return 0;
}

static int cmdq_try_peek(cmdq_t* q, stc_icd_cmd_t* out) {
  int ok = 0;
  pthread_mutex_lock(&q->mtx);
  if (q->count > 0) {
    *out = q->items[q->head];
    ok = 1;
  }
  pthread_mutex_unlock(&q->mtx);
  return ok;
}

static int cmdq_drop_head(cmdq_t* q) {
  int ok = 0;
  pthread_mutex_lock(&q->mtx);
  if (q->count > 0) {
    q->head = (q->head + 1) % (sizeof(q->items) / sizeof(q->items[0]));
    q->count--;
    ok = 1;
  }
  pthread_mutex_unlock(&q->mtx);
  return ok;
}

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
    case STC_CMD_MANUALPOS:
    case STC_CMD_POSMODE:
    case STC_CMD_REQ_SW_VERSION:
    case STC_CMD_REQ_HW_INFO:
    case STC_CMD_REQ_WORK_TIME:
      break;
    default:
        return STC_ERR_SYNTAX_ERROR;
  }

  // payload decoding must be consistent (for known payload-bearing commands)
  if (c->cmd_id != STC_CMD_CONFIG && c->payload_size > 0 && c->payload_decoded == 0) {
    return STC_ERR_SYNTAX_ERROR;
  }

  // basic value checks
  if (c->cmd_id == STC_CMD_TURRET) {
    if (c->u.turret.motion_type > STC_MOTION_SPEED) return STC_ERR_VALUE_OUT_OF_RANGE;
  }
  if (c->cmd_id == STC_CMD_SUT) {
    if (c->u.sut.motion_type > STC_MOTION_SPEED) return STC_ERR_VALUE_OUT_OF_RANGE;
  }
  if (c->cmd_id == STC_CMD_MIRROR) {
    if (c->u.mirror.motion_type > STC_MOTION_SPEED) return STC_ERR_VALUE_OUT_OF_RANGE;
  }
  if (c->cmd_id == STC_CMD_LASER) {
    if (c->u.laser.id < 1 || c->u.laser.id > 8) return STC_ERR_VALUE_OUT_OF_RANGE;
    if (c->u.laser.on_off > 1) return STC_ERR_VALUE_OUT_OF_RANGE;
  }

  // state gating + limits for turret axis0 (single motor)
  if (c->cmd_id == STC_CMD_TURRET && c->payload_decoded) {
    // Only allow motion when CU state is OPER (3). This matches the integration requirement.
    if (sh->cu_state != 3) return STC_ERR_NOT_ALLOWED_STATE;

    double az = (double)c->u.turret.az;
    if (c->u.turret.motion_type == STC_MOTION_ABSOLUTE) {
      if (az < sh->cfg[0].az_min_deg || az > sh->cfg[0].az_max_deg) return STC_ERR_VALUE_OUT_OF_RANGE;
    } else if (c->u.turret.motion_type == STC_MOTION_RELATIVE) {
      double cur_deg = pos_counts_to_deg(sh, sh->telem[0].pos_raw);
      double tgt = cur_deg + az;
      if (tgt < sh->cfg[0].az_min_deg || tgt > sh->cfg[0].az_max_deg) return STC_ERR_VALUE_OUT_OF_RANGE;
    } else if (c->u.turret.motion_type == STC_MOTION_SPEED) {
      if (az > sh->cfg[0].max_speed_deg_s || az < -sh->cfg[0].max_speed_deg_s) return STC_ERR_VALUE_OUT_OF_RANGE;
    }
  }

  return STC_ERR_NO_ERROR;
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
  int node_id;
  shared_t* sh;
  cmdq_t* q;
} can_worker_arg_t;

static void copy_pd4c_telem_into_shared(shared_t* sh, const pd4c_telemetry_t* t) {
  // Caller holds sh->mtx
  sh->d.telem[0].status_word = t->statusword;
  sh->d.telem[0].mode_disp   = t->mode_display;
  sh->d.telem[0].pos_raw     = t->pos_actual;
  sh->d.telem[0].vel_raw     = t->vel_actual;
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
  cur_pos_raw = sh->d.telem[0].pos_raw;
  counts_per_rev = sh->d.cfg[0].counts_per_rev;
  gear_ratio = sh->d.cfg[0].gear_ratio;
  pp_profile_vel = sh->d.cfg[0].pp_profile_vel;
  run_sdo_diag = sh->d.pp_sdo_diag;
  pthread_mutex_unlock(&sh->mtx);

  // Convert turret degrees -> motor counts and call low-level PP handshake.
  const int32_t target_counts = deg_to_counts(&sh->d, target_deg);
  const uint16_t pv = pp_profile_vel;
  const double cur_deg = pos_counts_to_deg(&sh->d, cur_pos_raw);

  fprintf(stderr,
          "PP move request: requested_az_deg=%.3f target_counts=%d current_pos_raw=%d current_deg=%.3f "
          "counts_per_rev=%.3f gear_ratio=%.3f pp_profile_vel=%u\n",
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
  pd4c_ctx_t ctx;
  memset(&ctx, 0, sizeof(ctx));

  int can_connected = 0;
  int reconnect_delay_ms = 500;
  uint64_t next_reconnect_ms = 0;
  int rc = PD4C_EIO;

  mtr_mode_t cur_mode = MTR_MODE_NONE;
  int enabled = 0;

  uint64_t last_vel_send_ms = 0;

  while (!g_stop) {
    uint64_t now = mono_ms();

    if (!can_connected && now >= next_reconnect_ms) {
      memset(&ctx, 0, sizeof(ctx));
      rc = pd4c_open(&ctx, a->can_if, (uint8_t)a->node_id);
      if (rc == PD4C_OK) {
        // Best-effort: ensure node is started (many setups already are).
        (void)pd4c_nmt_start(&ctx);

        can_connected = 1;
        reconnect_delay_ms = 500;
        pthread_mutex_lock(&a->sh->mtx);
        a->sh->d.can_ok = 1;
        pthread_mutex_unlock(&a->sh->mtx);

        fprintf(stderr, "CAN connected on %s (node=%d)\n", a->can_if, a->node_id);
      } else {
        fprintf(stderr, "pd4c_open(%s,node=%d) failed: %s; retry in %d ms\n",
                a->can_if, a->node_id, pd4c_strerror(rc), reconnect_delay_ms);
        next_reconnect_ms = now + (uint64_t)reconnect_delay_ms;
        reconnect_delay_ms *= 2;
        if (reconnect_delay_ms > 2000) reconnect_delay_ms = 2000;
      }
    }

    // --- poll CAN (non-blocking-ish)
    if (can_connected) {
      int ev = pd4c_poll(&ctx, 1);
      if (ev >= 0) {
        // Drain any extra frames quickly.
        for (int i = 0; i < 32; i++) {
          int ev2 = pd4c_poll(&ctx, 0);
          if (ev2 < 0) break;
        }
        const pd4c_telemetry_t* t = pd4c_telem(&ctx);
        pthread_mutex_lock(&a->sh->mtx);
        copy_pd4c_telem_into_shared(a->sh, t);
        pthread_mutex_unlock(&a->sh->mtx);
      } else if (ev != PD4C_ETIMEDOUT) {
        // Hard CAN error. Close and transition to reconnect loop.
        fprintf(stderr, "CAN poll error: %s\n", pd4c_strerror(ev));
        pd4c_close(&ctx);
        can_connected = 0;
        rc = ev;
        next_reconnect_ms = now + (uint64_t)reconnect_delay_ms;
        reconnect_delay_ms *= 2;
        if (reconnect_delay_ms > 2000) reconnect_delay_ms = 2000;
        enabled = 0;
        cur_mode = MTR_MODE_NONE;
        last_vel_send_ms = 0;
        pthread_mutex_lock(&a->sh->mtx);
        a->sh->d.can_ok = 0;
        pthread_mutex_unlock(&a->sh->mtx);
      }
    }

    // --- process discrete command queue
    stc_icd_cmd_t cmd;
    while (cmdq_try_peek(a->q, &cmd)) {
      if (!can_connected) {
        // CAN not available yet. Keep command queued for reconnect.
        break;
      }

      if (cmd.cmd_id == STC_CMD_RESET) {
        (void)pd4c_ds402_quick_stop(&ctx, (int8_t)cur_mode);
        (void)pd4c_ds402_fault_reset(&ctx, (int8_t)cur_mode, a->sh->d.ds402_timeout_ms);
        enabled = 0;  // force re-enable next command
      } else if (cmd.cmd_id == STC_CMD_SHUTDOWN) {
        pthread_mutex_lock(&a->sh->mtx);
        a->sh->d.stream[0].enabled = 0;
        pthread_mutex_unlock(&a->sh->mtx);
        motor_stop_and_disable(&ctx, cur_mode);
        enabled = 0;
        cur_mode = MTR_MODE_NONE;
      } else if (cmd.cmd_id == STC_CMD_FACTORY) {
        // Restore defaults (0x1011:1 = "load") then reset node.
        (void)pd4c_sdo_write_u32(&ctx, 0x1011, 0x01, 0x64616F6CUL);
        (void)pd4c_nmt_reset_node(&ctx);
        enabled = 0;
        cur_mode = MTR_MODE_NONE;
      } else if (cmd.cmd_id == STC_CMD_STORAGE) {
        // Store params (0x1010:1 = "save")
        (void)pd4c_sdo_write_u32(&ctx, 0x1010, 0x01, 0x65766173UL);
      } else if (cmd.cmd_id == STC_CMD_TURRET && cmd.payload_decoded) {
        // ABS/REL are discrete PP actions. SPEED is handled by streaming below.
        if (cmd.u.turret.motion_type == STC_MOTION_ABSOLUTE || cmd.u.turret.motion_type == STC_MOTION_RELATIVE) {
          const double cmd_az_deg = (double)cmd.u.turret.az;
          double target_deg = cmd_az_deg;
          if (cmd.u.turret.motion_type == STC_MOTION_RELATIVE) {
            pthread_mutex_lock(&a->sh->mtx);
            double cur_deg = pos_counts_to_deg(&a->sh->d, a->sh->d.telem[0].pos_raw);
            pthread_mutex_unlock(&a->sh->mtx);
            target_deg = cur_deg + target_deg;
          }

          const int32_t target_counts = deg_to_counts(&a->sh->d, target_deg);
          fprintf(stderr,
                  "TURRET %s seq=%u az_deg=%.6f target_counts=%" PRId32 "\n",
                  (cmd.u.turret.motion_type == STC_MOTION_ABSOLUTE) ? "ABS" : "REL",
                  cmd.seq,
                  cmd_az_deg,
                  target_counts);

          // PP move should stop any active speed streaming.
          pthread_mutex_lock(&a->sh->mtx);
          a->sh->d.stream[0].enabled = 0;
          pthread_mutex_unlock(&a->sh->mtx);

          rc = motor_do_pp_move(&ctx, a->sh, target_deg, &cur_mode, &enabled);
          fprintf(stderr,
                  "TURRET seq=%u pd4c_pp_move=%d latest_statusword=0x%04X\n",
                  cmd.seq,
                  rc,
                  pd4c_telem(&ctx)->statusword);
        }
      }

      (void)cmdq_drop_head(a->q);
    }

    // --- enforce CU state gating from shared state
    uint8_t cu_state = 0;
    int speed_on = 0;
    double desired_deg_s = 0.0;
    uint64_t last_cmd_ms = 0;
    int watchdog_ms = 0;
    pthread_mutex_lock(&a->sh->mtx);
    cu_state = a->sh->d.cu_state;
    speed_on = a->sh->d.stream[0].enabled;
    desired_deg_s = a->sh->d.stream[0].desired_speed_deg_s;
    last_cmd_ms = a->sh->d.stream[0].last_cmd_ms;
    watchdog_ms = a->sh->d.speed_watchdog_ms;
    pthread_mutex_unlock(&a->sh->mtx);

    if (cu_state != 3) {
      // Not operational: stop motion and disable streaming.
      if (speed_on && can_connected) {
        pthread_mutex_lock(&a->sh->mtx);
        a->sh->d.stream[0].enabled = 0;
        pthread_mutex_unlock(&a->sh->mtx);
        motor_stop_and_disable(&ctx, cur_mode);
        enabled = 0;
      }
    }

    // --- speed watchdog + velocity streaming
    if (can_connected && cu_state == 3) {
      if (speed_on) {
        if (watchdog_ms > 0 && (int)(now - last_cmd_ms) > watchdog_ms) {
          // Watchdog: no new speed cmd.
          pthread_mutex_lock(&a->sh->mtx);
          a->sh->d.stream[0].enabled = 0;
          pthread_mutex_unlock(&a->sh->mtx);
          (void)pd4c_ds402_quick_stop(&ctx, (int8_t)cur_mode);
        } else {
          // Ensure PV mode enabled once.
          (void)motor_ensure_enabled(&ctx, a->sh, MTR_MODE_PV, &cur_mode, &enabled);

          // Send velocity at ~100Hz.
          if (last_vel_send_ms == 0 || (now - last_vel_send_ms) >= 10) {
            int clipped = 0;
            int16_t tv = degs_to_target_vel_units(&a->sh->d, desired_deg_s, &clipped);
            (void)pd4c_rpdo3_send_target_velocity(&ctx, tv);
            last_vel_send_ms = now;
          }
        }
      }
    }
  }

  if (can_connected) pd4c_close(&ctx);
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

    const uint64_t ts_mono_ms = mono_ms();
    pthread_mutex_lock(&a->sh->mtx);
    m.seq = a->sh->d.seq_h++;
    axis_telem_update(&a->sh->d.telem[0],
                      &a->sh->d.cfg[0],
                      a->sh->d.telem[0].pos_raw,
                      a->sh->d.telem[0].vel_raw,
                      ts_mono_ms);
    axis_telem_t axis0 = a->sh->d.telem[0];
    pthread_mutex_unlock(&a->sh->mtx);

    m.timestamp_ms = wall_ms();

    // axis 0 only for now
    m.position[0] = (float)axis0.pos_deg;
    m.rates[0]    = (float)axis0.vel_deg_s;
    m.acceleration[0] = (float)axis0.accel_deg_s2;

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

    // snapshot shared
    pthread_mutex_lock(&a->sh->mtx);
    m.seq = a->sh->d.seq_l++;
    m.state = a->sh->d.cu_state;
    uint64_t uptime_ms = nowm - a->sh->d.boot_ms;
    m.uptime_sec = (uint32_t)(uptime_ms / 1000ULL);
    m.lat = a->sh->d.has_manualpos ? a->sh->d.lat : 0.0;
    m.lon = a->sh->d.has_manualpos ? a->sh->d.lon : 0.0;
    m.alt_m = a->sh->d.has_manualpos ? (float)a->sh->d.alt : 0.0f;
    m.heading_deg = a->sh->d.has_manualpos ? (float)a->sh->d.heading : 0.0f;

    int can_ok = a->sh->d.can_ok;
    uint16_t sw = a->sh->d.telem[0].status_word;
    int have_emcy = a->sh->d.have_emcy;
    uint16_t emcy_code = a->sh->d.emcy_code;
    uint8_t emcy_reg = a->sh->d.emcy_reg;

    uint64_t t1 = a->sh->d.ts_tpdo1_ms;
    uint64_t t2 = a->sh->d.ts_tpdo2_ms;
    uint64_t t3 = a->sh->d.ts_tpdo3_ms;
    pthread_mutex_unlock(&a->sh->mtx);

    m.timestamp_ms = noww;
    m.temperature_c = 0.0f;
    m.humidity_pct  = 0.0f;
    m.pressure_hpa  = 0.0f;
    memset(m.motor_status, 0, sizeof(m.motor_status));
    memset(m.laser_status, 0, sizeof(m.laser_status));
    m.level_x = 0.0f;
    m.level_y = 0.0f;

    // error list
    m.error_count = 0;
    if (!can_ok) {
      m.errors[m.error_count++] = 0xE0010001u;  // CAN down
    }
    // TPDO staleness thresholds: 500ms
    if (can_ok) {
      if (t1 == 0 || (nowm > t1 && (nowm - t1) > 500)) m.errors[m.error_count++] = 0xE0010002u;
      if (t2 == 0 || (nowm > t2 && (nowm - t2) > 500)) m.errors[m.error_count++] = 0xE0010003u;
      if (t3 == 0 || (nowm > t3 && (nowm - t3) > 500)) m.errors[m.error_count++] = 0xE0010004u;
    }
    // DS402 fault pattern
    if ((sw & 0x006F) == 0x0008) {
      m.errors[m.error_count++] = 0xE0020000u | (uint32_t)sw;
    }
    // EMCY
    if (have_emcy) {
      m.errors[m.error_count++] = 0xE0030000u | (uint32_t)emcy_code;
      m.errors[m.error_count++] = 0xE0040000u | (uint32_t)emcy_reg;
    }
    if (m.error_count > STC_ICD_MAX_ERRORS) m.error_count = STC_ICD_MAX_ERRORS;

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
          "          [--counts-per-rev N] [--gear-ratio N] [--vel-scale N]\n"
          "          [--az-min DEG] [--az-max DEG] [--max-speed-deg-s X]\n"
          "          [--speed-watchdog-ms MS] [--pp-profile-vel U16]\n"
          "          [--ds402-timeout-ms MS] [--pp-sdo-diag 0|1]\n"
          "          [--tlmh-ip 239.255.10.2 --tlmh-port 6010 --tlmh-hz 100]\n"
          "          [--tlml-ip 239.255.10.3 --tlml-port 6020 --tlml-hz 10]\n",
          p);
}

int main(int argc, char** argv) {
  const char* can_if = "can0";
  int node_id = 1;
  int tcp_port = 7000;

  const char* tlmh_ip = "239.255.10.2";
  int tlmh_port = 6010;
  int tlmh_hz = 100;

  const char* tlml_ip = "239.255.10.3";
  int tlml_port = 6020;
  int tlml_hz = 10;

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

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--can") && i + 1 < argc) can_if = argv[++i];
    else if (!strcmp(argv[i], "--node") && i + 1 < argc) node_id = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--tcp-port") && i + 1 < argc) tcp_port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--counts-per-rev") && i + 1 < argc) sh.d.cfg[0].counts_per_rev = atof(argv[++i]);
    else if (!strcmp(argv[i], "--gear-ratio") && i + 1 < argc) sh.d.cfg[0].gear_ratio = atof(argv[++i]);
    else if (!strcmp(argv[i], "--vel-scale") && i + 1 < argc) sh.d.cfg[0].vel_scale = atof(argv[++i]);
    else if (!strcmp(argv[i], "--az-min") && i + 1 < argc) sh.d.cfg[0].az_min_deg = atof(argv[++i]);
    else if (!strcmp(argv[i], "--az-max") && i + 1 < argc) sh.d.cfg[0].az_max_deg = atof(argv[++i]);
    else if (!strcmp(argv[i], "--max-speed-deg-s") && i + 1 < argc) sh.d.cfg[0].max_speed_deg_s = atof(argv[++i]);
    else if (!strcmp(argv[i], "--speed-watchdog-ms") && i + 1 < argc) sh.d.speed_watchdog_ms = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--pp-profile-vel") && i + 1 < argc) sh.d.cfg[0].pp_profile_vel = (uint16_t)atoi(argv[++i]);
    else if (!strcmp(argv[i], "--ds402-timeout-ms") && i + 1 < argc) sh.d.ds402_timeout_ms = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--pp-sdo-diag") && i + 1 < argc) sh.d.pp_sdo_diag = atoi(argv[++i]) ? 1 : 0;
    else if (!strcmp(argv[i], "--tlmh-ip") && i + 1 < argc) tlmh_ip = argv[++i];
    else if (!strcmp(argv[i], "--tlmh-port") && i + 1 < argc) tlmh_port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--tlmh-hz") && i + 1 < argc) tlmh_hz = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--tlml-ip") && i + 1 < argc) tlml_ip = argv[++i];
    else if (!strcmp(argv[i], "--tlml-port") && i + 1 < argc) tlml_port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--tlml-hz") && i + 1 < argc) tlml_hz = atoi(argv[++i]);
    else {
      usage(argv[0]);
      return 2;
    }
  }

  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);

  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();
  cfg.strict_magic = 0;    // allow us to return NACK even when magic is wrong
  cfg.strict_version = 0;

  cmdq_t q;
  cmdq_init(&q);

  // CAN worker thread
  pthread_t th_can;
  can_worker_arg_t carg = { .can_if = can_if, .node_id = node_id, .sh = &sh, .q = &q };
  pthread_create(&th_can, NULL, can_worker_thread, &carg);

  // UDP telemetry threads
  int udp_h = open_udp_tx();
  int udp_l = open_udp_tx();
  if (udp_h < 0 || udp_l < 0) {
    perror("udp socket");
    return 1;
  }

  pthread_t th_h, th_l;
  tlm_tx_arg_t harg = { .udp_fd = udp_h, .ip = tlmh_ip, .port = tlmh_port, .rate_hz = tlmh_hz, .sh = &sh, .cfg = cfg, .sender_id = 1 };
  tlm_tx_arg_t larg = { .udp_fd = udp_l, .ip = tlml_ip, .port = tlml_port, .rate_hz = tlml_hz, .sh = &sh, .cfg = cfg, .sender_id = 1 };
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
  addr.sin_port = htons((uint16_t)tcp_port);
  if (bind(srv, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("bind");
    return 1;
  }
  if (listen(srv, 4) < 0) {
    perror("listen");
    return 1;
  }

  printf("CAN worker on %s node=%d (TPDO1/2/3 + RPDO1/2/3)\n", can_if, node_id);
  printf("STC server listening on TCP port %d\n", tcp_port);
  printf("UDP multicast: TLMH %s:%d @%dHz | TLML %s:%d @%dHz\n", tlmh_ip, tlmh_port, tlmh_hz, tlml_ip, tlml_port, tlml_hz);

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

      // Fast snapshot for validation (single mutex lock)
      shared_data_t snap;
      pthread_mutex_lock(&sh.mtx);
      snap = sh.d;
      pthread_mutex_unlock(&sh.mtx);

      print_cmd(&cmd);
      stc_nack_err_t v = validate_cmd_fast(&cfg, &snap, &cmd);

      // Fast side-effects that should be reflected in TLML immediately
      if (v == STC_ERR_NO_ERROR && cmd.payload_decoded) {
        pthread_mutex_lock(&sh.mtx);
        if (cmd.cmd_id == STC_CMD_CU_STATE) {
          sh.d.cu_state = cmd.u.cu_state.state;
        } else if (cmd.cmd_id == STC_CMD_MANUALPOS) {
          sh.d.lat = cmd.u.manualpos.lat;
          sh.d.lon = cmd.u.manualpos.lon;
          sh.d.alt = cmd.u.manualpos.alt;
          sh.d.heading = cmd.u.manualpos.heading;
          sh.d.has_manualpos = 1;
        } else if (cmd.cmd_id == STC_CMD_POSMODE) {
          sh.d.posmode = cmd.u.posmode.v;
        } else if (cmd.cmd_id == STC_CMD_TURRET && cmd.u.turret.motion_type == STC_MOTION_SPEED) {
          // SPEED is handled via streaming: update desired speed + watchdog timestamp
          sh.d.stream[0].desired_speed_deg_s = (double)cmd.u.turret.az;
          sh.d.stream[0].last_cmd_ms = mono_ms();
          sh.d.stream[0].enabled = 1;
        }
        pthread_mutex_unlock(&sh.mtx);
      }

      // Enqueue discrete actions for worker
      if (v == STC_ERR_NO_ERROR) {
        int need_can = 0;
        int enq = 0;

        if (cmd.cmd_id == STC_CMD_RESET || cmd.cmd_id == STC_CMD_FACTORY || cmd.cmd_id == STC_CMD_SHUTDOWN || cmd.cmd_id == STC_CMD_STORAGE) {
          need_can = 1;
          enq = 1;
        } else if (cmd.cmd_id == STC_CMD_TURRET && cmd.payload_decoded) {
          if (cmd.u.turret.motion_type == STC_MOTION_ABSOLUTE || cmd.u.turret.motion_type == STC_MOTION_RELATIVE) {
            need_can = 1;
            enq = 1;
          }
        }

        if (enq) {
          pthread_mutex_lock(&sh.mtx);
          int can_ok = sh.d.can_ok;
          pthread_mutex_unlock(&sh.mtx);
          if (!can_ok && need_can) {
            v = STC_ERR_INSUFFICIENT_RES;
          } else if (cmdq_push(&q, &cmd) != 0) {
            v = STC_ERR_INSUFFICIENT_RES;
          }
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
        rsp.err_code = (uint8_t)v;
      }

      if (v == STC_ERR_NO_ERROR && cmd.cmd_id == STC_CMD_REQ_SW_VERSION) {
        rsp.payload_size = (uint16_t)strlen(STC_SW_VERSION);
        memcpy(rsp.payload, STC_SW_VERSION, rsp.payload_size);
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
