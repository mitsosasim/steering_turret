/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Implementation/declarations for the STC development codebase.
 *
 * Design notes:
 *   Documentation/comment ownership header only; no runtime behavior changes.
 */

#define _POSIX_C_SOURCE 200809L

#include "motion_backend.h"
#include "stc_log.h"
#include "stc_limits.h"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

enum {
  MYACT_CMD_STOP = 0x81,
  MYACT_CMD_SPEED = 0xA2,
  MYACT_CMD_POS_ABS = 0xA4,
  MYACT_CMD_POS_INC = 0xA8,
  MYACT_CMD_BRAKE_RELEASE = 0x77,
  MYACT_CMD_READ_MODEL = 0xB5,
  MYACT_CMD_READ_VERSION = 0xB2,
  MYACT_CMD_READ_RUNTIME = 0xB1,
  MYACT_CMD_READ_STATUS1 = 0x9A,
  MYACT_CMD_READ_ANGLE = 0x92,
  MYACT_CMD_READ_STATUS2 = 0x9C,
  MYACT_CAN_BASE = 0x140,
  TELEMETRY_PERIOD_NS = 20 * 1000 * 1000,
};

typedef enum {
  AXIS_CMD_NONE = 0,
  AXIS_CMD_SPEED,
  AXIS_CMD_POSITION_ABS,
  AXIS_CMD_POSITION_REL,
  AXIS_CMD_STOP,
} axis_cmd_kind_t;

typedef struct {
  axis_cmd_kind_t kind;
  double value;
  uint64_t seq;
  uint64_t cmd_ts_ms;
} axis_mailbox_t;

struct motion_backend {
  motion_backend_cfg_t cfg;
  int can_socket[STC_AXIS_COUNT];
  pthread_t worker;
  int running;
  pthread_mutex_t lock;
  pthread_cond_t wake;

  motion_snapshot_t snap;
  axis_mailbox_t mailbox[STC_AXIS_COUNT];
  uint64_t next_seq;
  uint64_t applied_seq[STC_AXIS_COUNT];
  uint64_t last_speed_cmd_ms[STC_AXIS_COUNT];
  uint64_t last_cmd_ms[STC_AXIS_COUNT];
  int32_t cmd_pos_raw[STC_AXIS_COUNT];
  int32_t cmd_vel_raw[STC_AXIS_COUNT];
  uint8_t cmd_ack_pending[STC_AXIS_COUNT];
  int speed_active[STC_AXIS_COUNT];
  int velocity_session_ready[STC_AXIS_COUNT];
  int32_t last_speed_raw_0p01dps[STC_AXIS_COUNT];
  int native_abs_validated[STC_AXIS_COUNT];
  int last_abs_via_a8_service[STC_AXIS_COUNT];
};

static uint64_t now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}

static int open_can_socket(const char* ifname, uint8_t node_id) {
  int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) return -1;

  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, (ifname && ifname[0]) ? ifname : "can0", IFNAMSIZ - 1);
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    close(fd);
    return -1;
  }

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  const int nonblock = 1;
  if (ioctl(fd, FIONBIO, &nonblock) < 0) { close(fd); return -1; }
  struct can_filter filter;
  filter.can_id = (canid_t)(0x240u + (uint32_t)node_id);
  filter.can_mask = CAN_SFF_MASK;
  if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, (socklen_t)sizeof(filter)) < 0) {
    close(fd);
    return -1;
  }

  if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    close(fd);
    return -1;
  }

  return fd;
}

static int socket_for_node(struct motion_backend* b, uint8_t node_id) {
  if (!b || node_id == 0) return -1;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (b->cfg.axis_node_id[axis] == node_id) return b->can_socket[axis];
  }
  return -1;
}

static int send_myact_cmd(struct motion_backend* b, uint8_t node_id, uint8_t cmd, uint16_t speed_dps, int32_t value) {
  const int fd = socket_for_node(b, node_id);
  if (fd < 0) return -1;

  struct can_frame f;
  memset(&f, 0, sizeof(f));
  f.can_id = MYACT_CAN_BASE + node_id;
  f.can_dlc = 8;
  f.data[0] = cmd;
  f.data[2] = (uint8_t)(speed_dps & 0xFFu);
  f.data[3] = (uint8_t)((speed_dps >> 8) & 0xFFu);
  f.data[4] = (uint8_t)(value & 0xFF);
  f.data[5] = (uint8_t)((value >> 8) & 0xFF);
  f.data[6] = (uint8_t)((value >> 16) & 0xFF);
  f.data[7] = (uint8_t)((value >> 24) & 0xFF);

  return (write(fd, &f, sizeof(f)) == (ssize_t)sizeof(f)) ? 0 : -1;
}

static int32_t local_axis_deg_to_counts(const axis_cfg_t* cfg, double deg) {
  const double gr = (cfg->gear_ratio > 0.0) ? cfg->gear_ratio : 1.0;
  const double cpd = (cfg->counts_per_rev != 0) ? ((double)cfg->counts_per_rev / 360.0) : 1.0;
  return (int32_t)llround(deg * gr * cpd);
}

static double local_axis_counts_to_deg(const axis_cfg_t* cfg, int32_t counts) {
  const double gr = (cfg->gear_ratio > 0.0) ? cfg->gear_ratio : 1.0;
  const double cpd = (cfg->counts_per_rev != 0) ? ((double)cfg->counts_per_rev / 360.0) : 1.0;
  return ((double)counts) / (gr * cpd);
}

static void normalize_snapshot_from_feedback(struct motion_backend* b, size_t axis, int16_t iq_0p01a, int16_t speed_dps, int16_t angle_deg, uint64_t t_ms);

static double clamp_range(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void log_myact_failure(struct motion_backend* b, size_t axis, const char* action, int rc) {
  const unsigned err_flags = b ? (unsigned)b->snap.status_word[axis] : 0u;
  STC_LOG_ERROR("axis%zu MYACT %s failed rc=%d err_flags=0x%04X", axis, action, rc, err_flags);
}




static int send_myact_read(struct motion_backend* b, uint8_t node_id, uint8_t cmd) {
  return send_myact_cmd(b, node_id, cmd, 0, 0);
}

static int recv_myact_reply(struct motion_backend* b, uint8_t node_id, uint8_t expected_cmd, struct can_frame* out, int timeout_ms) {
  if (!b || !out) return -1;
  const int fd = socket_for_node(b, node_id);
  if (fd < 0) return -1;
  const canid_t expected_id = (canid_t)(MYACT_CAN_BASE + (uint32_t)node_id);
  const uint64_t deadline = now_ms() + (uint64_t)((timeout_ms > 0) ? timeout_ms : 50);
  while (now_ms() <= deadline) {
    struct can_frame f;
    const ssize_t r = read(fd, &f, sizeof(f));
    if (r == (ssize_t)sizeof(f)) {
      const canid_t rx_id = f.can_id & CAN_SFF_MASK;
      if (f.can_dlc < 1 || rx_id != expected_id) continue;
      if (f.data[0] != expected_cmd) continue;
      *out = f;
      return 0;
    }
    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      struct timespec ts = {.tv_sec = 0, .tv_nsec = 1 * 1000 * 1000};
      nanosleep(&ts, NULL);
      continue;
    }
    return -1;
  }
  return -1;
}


static int poll_axis_feedback_once(struct motion_backend* b, size_t axis, uint8_t node_id, uint8_t cmd, int timeout_ms) {
  struct can_frame fr;
  if (send_myact_read(b, node_id, cmd) != 0) return -1;
  if (recv_myact_reply(b, node_id, cmd, &fr, timeout_ms) != 0 || fr.can_dlc < 8) return -1;

  const uint64_t t_ms = now_ms();
  if (cmd == MYACT_CMD_READ_ANGLE) {
    const int16_t angle_deg = (int16_t)((uint16_t)fr.data[6] | ((uint16_t)fr.data[7] << 8));
    normalize_snapshot_from_feedback(b, axis, 0, 0, angle_deg, t_ms);
    return 0;
  }

  if (cmd == MYACT_CMD_READ_STATUS1 || cmd == MYACT_CMD_READ_STATUS2) {
    const int16_t iq_0p01a = (int16_t)((uint16_t)fr.data[2] | ((uint16_t)fr.data[3] << 8));
    const int16_t speed_dps = (int16_t)((uint16_t)fr.data[4] | ((uint16_t)fr.data[5] << 8));
    const int16_t angle_deg = (int16_t)((uint16_t)fr.data[6] | ((uint16_t)fr.data[7] << 8));
    normalize_snapshot_from_feedback(b, axis, iq_0p01a, speed_dps, angle_deg, t_ms);
    if (cmd == MYACT_CMD_READ_STATUS1) b->snap.status_word[axis] = (uint16_t)iq_0p01a;
    return 0;
  }

  return -1;
}

static int wait_for_position_target_locked(struct motion_backend* b, size_t axis, uint8_t node_id, double target_deg) {
  const axis_cfg_t* cfg = &b->cfg.axis_cfg[axis];
  const double tolerance_deg = (b->cfg.position_tolerance_deg > 0.0) ? b->cfg.position_tolerance_deg : 1.0;
  const int timeout_ms = (b->cfg.position_timeout_ms > 0) ? b->cfg.position_timeout_ms : 5000;
  const int poll_ms = (b->cfg.position_poll_ms > 0) ? b->cfg.position_poll_ms : 50;
  const uint64_t deadline = now_ms() + (uint64_t)timeout_ms;
  double final_deg = local_axis_counts_to_deg(cfg, b->snap.pos_raw[axis]);
  while (now_ms() <= deadline) {
    if (poll_axis_feedback_once(b, axis, node_id, MYACT_CMD_READ_ANGLE, poll_ms) == 0 ||
        poll_axis_feedback_once(b, axis, node_id, MYACT_CMD_READ_STATUS2, poll_ms) == 0) {
      final_deg = local_axis_counts_to_deg(cfg, b->snap.pos_raw[axis]);
    }

    const double err_deg = fabs(final_deg - target_deg);
    if (err_deg <= tolerance_deg) {
      STC_LOG_INFO("axis%zu MYACT target reached target=%.3f final=%.3f err=%.3f", axis, target_deg, final_deg, err_deg);
      return 0;
    }

    struct timespec ts = {.tv_sec = 0, .tv_nsec = (long)poll_ms * 1000L * 1000L};
    nanosleep(&ts, NULL);
  }

  final_deg = local_axis_counts_to_deg(cfg, b->snap.pos_raw[axis]);
  const double err_deg = fabs(final_deg - target_deg);
  STC_LOG_ERROR("axis%zu MYACT target timeout target=%.3f final=%.3f err=%.3f", axis, target_deg, final_deg, err_deg);
  return -1;
}

static int init_axis_feedback(struct motion_backend* b, size_t axis) {
  const uint8_t node_id = b->cfg.axis_node_id[axis];
  if (!node_id) return 0;

  struct can_frame fr;
  int got_status1 = 0;
  int got_angle = 0;
  int16_t status_iq_0p01a = 0;
  int16_t status_speed_dps = 0;
  int16_t status_angle_deg = 0;
  uint16_t status_err_flags = 0;
  int16_t angle_deg = 0;
  char model[8] = {0};
  uint16_t sw_version = 0;
  uint32_t runtime_ms = 0;

  if (send_myact_read(b, node_id, MYACT_CMD_READ_MODEL) == 0 &&
      recv_myact_reply(b, node_id, MYACT_CMD_READ_MODEL, &fr, 50) == 0 && fr.can_dlc >= 8) {
    memcpy(model, &fr.data[1], 7);
    model[7] = '\0';
  }
  if (send_myact_read(b, node_id, MYACT_CMD_READ_VERSION) == 0 &&
      recv_myact_reply(b, node_id, MYACT_CMD_READ_VERSION, &fr, 50) == 0 && fr.can_dlc >= 8) {
    sw_version = (uint16_t)fr.data[1] | ((uint16_t)fr.data[2] << 8);
  }
  if (send_myact_read(b, node_id, MYACT_CMD_READ_RUNTIME) == 0 &&
      recv_myact_reply(b, node_id, MYACT_CMD_READ_RUNTIME, &fr, 50) == 0 && fr.can_dlc >= 8) {
    runtime_ms = (uint32_t)fr.data[4] | ((uint32_t)fr.data[5] << 8) |
                 ((uint32_t)fr.data[6] << 16) | ((uint32_t)fr.data[7] << 24);
  }

  if (send_myact_read(b, node_id, MYACT_CMD_READ_STATUS1) == 0 && recv_myact_reply(b, node_id, MYACT_CMD_READ_STATUS1, &fr, 80) == 0 && fr.can_dlc >= 8) {
    got_status1 = 1;
    status_iq_0p01a = (int16_t)((uint16_t)fr.data[2] | ((uint16_t)fr.data[3] << 8));
    status_speed_dps = (int16_t)((uint16_t)fr.data[4] | ((uint16_t)fr.data[5] << 8));
    status_angle_deg = (int16_t)((uint16_t)fr.data[6] | ((uint16_t)fr.data[7] << 8));
    status_err_flags = (uint16_t)status_iq_0p01a;
  }

  if (send_myact_read(b, node_id, MYACT_CMD_READ_ANGLE) == 0 && recv_myact_reply(b, node_id, MYACT_CMD_READ_ANGLE, &fr, 80) == 0 && fr.can_dlc >= 8) {
    got_angle = 1;
    angle_deg = (int16_t)((uint16_t)fr.data[6] | ((uint16_t)fr.data[7] << 8));
  }

  if (send_myact_read(b, node_id, MYACT_CMD_READ_STATUS2) == 0)
    (void)recv_myact_reply(b, node_id, MYACT_CMD_READ_STATUS2, &fr, 50);

  if (!got_status1 && !got_angle) return -1;

  const uint64_t t_ms = now_ms();
  const int16_t use_angle = got_angle ? angle_deg : status_angle_deg;
  normalize_snapshot_from_feedback(b, axis, status_iq_0p01a, status_speed_dps, use_angle, t_ms);
  b->snap.vel_raw[axis] = 0;
  b->snap.status_word[axis] = status_err_flags;
  b->snap.mode_display[axis] = 0;
  b->snap.diag_ext[axis] = (uint32_t)(got_status1 ? ((status_err_flags & 0xFFFFu) | ((uint32_t)(fr.data[1] & 0x1u) << 16)) : 0u);
  b->snap.last_rx_ms[axis] = t_ms;
  b->snap.axis_available[axis] = 1;
  STC_LOG_INFO("axis%zu myact init OK node=%u model=%s ver=%u runtime_ms=%u status1 err=0x%04X feedback fresh",
               axis, node_id, model[0] ? model : "unknown", sw_version, runtime_ms, (unsigned)status_err_flags);
  return 0;
}
static int prepare_for_motion_locked(struct motion_backend* b, size_t axis, uint64_t now) {
  (void)now;
  const uint8_t node_id = b->cfg.axis_node_id[axis];
  if (!node_id) return -1;
  (void)send_myact_cmd(b, node_id, MYACT_CMD_BRAKE_RELEASE, 0, 0);
  return 0;
}

static void normalize_snapshot_from_feedback(struct motion_backend* b, size_t axis, int16_t iq_0p01a, int16_t speed_dps, int16_t angle_deg, uint64_t t_ms) {
  const axis_cfg_t* cfg = &b->cfg.axis_cfg[axis];
  const double gr = (cfg->gear_ratio > 0.0) ? cfg->gear_ratio : 1.0;
  const double cpd = (cfg->counts_per_rev != 0) ? ((double)cfg->counts_per_rev / 360.0) : 1.0;
  const double axis_speed_deg_s = ((double)speed_dps) / gr;
  const double axis_angle_deg = ((double)angle_deg) / gr;
  b->snap.vel_raw[axis] = (int32_t)llround(axis_speed_deg_s * cpd);
  b->snap.pos_raw[axis] = (int32_t)llround(axis_angle_deg * cpd);
  (void)iq_0p01a;
  b->snap.last_rx_ms[axis] = t_ms;
}

static void refresh_feedback_locked(struct motion_backend* b, uint64_t t_ms) {
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    const int fd = b->can_socket[axis];
    if (fd < 0) continue;
    struct can_frame f;
    while (read(fd, &f, sizeof(f)) == (ssize_t)sizeof(f)) {
      if (f.can_dlc < 8 || f.can_id < MYACT_CAN_BASE) continue;
      if (f.data[0] == MYACT_CMD_READ_STATUS1) {
        const uint16_t error_flags = (uint16_t)f.data[6] | ((uint16_t)f.data[7] << 8);
        b->snap.status_word[axis] = error_flags;
        b->snap.last_rx_ms[axis] = t_ms;
        continue;
      }
      if (f.data[0] == MYACT_CMD_SPEED || f.data[0] == MYACT_CMD_POS_ABS || f.data[0] == MYACT_CMD_POS_INC) {
        const int16_t iq_0p01a = (int16_t)((uint16_t)f.data[2] | ((uint16_t)f.data[3] << 8));
        const int16_t speed_dps = (int16_t)((uint16_t)f.data[4] | ((uint16_t)f.data[5] << 8));
        const int16_t angle_deg = (int16_t)((uint16_t)f.data[6] | ((uint16_t)f.data[7] << 8));
        normalize_snapshot_from_feedback(b, axis, iq_0p01a, speed_dps, angle_deg, t_ms);
        if (f.data[0] == MYACT_CMD_POS_ABS) b->native_abs_validated[axis] = 1;
      }
    }
  }
}

static int axis_stop_locked(struct motion_backend* b, size_t axis, uint64_t t_ms, int shutdown) {
  const uint8_t node_id = b->cfg.axis_node_id[axis];
  int stop_rc = -1;
  if (node_id) stop_rc = send_myact_cmd(b, node_id, MYACT_CMD_STOP, 0, 0);
  if (shutdown && node_id) {
    struct timespec ts = {.tv_sec = 0, .tv_nsec = 50 * 1000 * 1000};
    nanosleep(&ts, NULL);
    (void)send_myact_cmd(b, node_id, 0x80, 0, 0);
  }
  b->speed_active[axis] = 0;
  b->velocity_session_ready[axis] = 0;
  b->last_speed_raw_0p01dps[axis] = 0;
  b->snap.vel_raw[axis] = 0;
  b->snap.mode_display[axis] = 0;
  b->snap.last_rx_ms[axis] = t_ms;
  return stop_rc;
}

static void apply_axis_command_locked(struct motion_backend* b, size_t axis, uint64_t t_ms) {
  const uint8_t node_id = b->cfg.axis_node_id[axis];
  if (!node_id) return;

  const axis_mailbox_t* m = &b->mailbox[axis];
  if (m->seq == b->applied_seq[axis] && !(m->kind == AXIS_CMD_SPEED && b->speed_active[axis])) return;

  const axis_cfg_t* cfg = &b->cfg.axis_cfg[axis];
  const double gr = (cfg->gear_ratio > 0.0) ? cfg->gear_ratio : 1.0;
  const double max_speed = fmax(0.1, cfg->max_speed_deg_s);

  if (m->kind == AXIS_CMD_SPEED) {
    const double clamped = clamp_range(m->value, -max_speed, max_speed);
    const int32_t speed_0p01dps = (int32_t)llround(clamped * gr * 100.0);
    const int is_refresh = (m->seq == b->applied_seq[axis]);
    const int need_reissue = is_refresh && b->speed_active[axis] && ((t_ms - b->last_speed_cmd_ms[axis]) <= (uint64_t)b->cfg.speed_watchdog_ms);
    if (!is_refresh && !need_reissue && b->speed_active[axis] && speed_0p01dps == b->last_speed_raw_0p01dps[axis]) {
      b->last_speed_cmd_ms[axis] = m->cmd_ts_ms;
      b->applied_seq[axis] = m->seq;
      return;
    }
    if (!b->velocity_session_ready[axis]) {
      (void)send_myact_cmd(b, node_id, MYACT_CMD_BRAKE_RELEASE, 0, 0);
      b->velocity_session_ready[axis] = 1;
    }
    const int speed_rc = send_myact_cmd(b, node_id, MYACT_CMD_SPEED, 0, speed_0p01dps);
    if (speed_rc == 0) {
      b->speed_active[axis] = (speed_0p01dps != 0);
      b->last_speed_raw_0p01dps[axis] = speed_0p01dps;
      b->last_speed_cmd_ms[axis] = m->cmd_ts_ms;
      b->snap.vel_raw[axis] = local_axis_deg_to_counts(cfg, clamped);
      b->snap.mode_display[axis] = (speed_0p01dps != 0) ? 2 : 0;
      b->snap.status_word[axis] &= (uint16_t)~(1u << 10);
      b->snap.last_rx_ms[axis] = t_ms;
      b->last_cmd_ms[axis] = m->cmd_ts_ms;
      b->cmd_vel_raw[axis] = local_axis_deg_to_counts(cfg, clamped);
      b->cmd_ack_pending[axis] = 1u;
      (void)poll_axis_feedback_once(b, axis, node_id, MYACT_CMD_READ_STATUS2, 20);
    } else {
      log_myact_failure(b, axis, "A2", speed_rc);
    }
  } else if (m->kind == AXIS_CMD_POSITION_ABS || m->kind == AXIS_CMD_POSITION_REL) {
    if (prepare_for_motion_locked(b, axis, t_ms) != 0) return;
    const double target_deg = clamp_range(m->value, cfg->az_min_deg, cfg->az_max_deg);
    double expected_target_deg = target_deg;
    const uint16_t speed_dps = (uint16_t)fmax(1.0, fmin(65535.0, max_speed * gr));
    int cmd_ok = -1;
    if (m->kind == AXIS_CMD_POSITION_REL) {
      const double current_deg = local_axis_counts_to_deg(cfg, b->snap.pos_raw[axis]);
      double requested_abs = 0.0;
      double clamped_abs = 0.0;
      double effective_delta = 0.0;
      const bool saturated = stc_compute_effective_delta(current_deg,
                                                         m->value,
                                                         cfg->az_min_deg,
                                                         cfg->az_max_deg,
                                                         &requested_abs,
                                                         &clamped_abs,
                                                         &effective_delta);
      if (saturated) {
        STC_LOG_WARN("axis%zu command saturated: current=%.3f requested_delta=%.3f requested_abs=%.3f clamped_abs=%.3f effective_delta=%.3f",
                     axis,
                     current_deg,
                     m->value,
                     requested_abs,
                     clamped_abs,
                     effective_delta);
      }
      const int32_t delta_0p01deg = (int32_t)llround(effective_delta * gr * 100.0);
      expected_target_deg = current_deg + effective_delta;
      cmd_ok = send_myact_cmd(b, node_id, MYACT_CMD_POS_INC, speed_dps, delta_0p01deg);
      if (cmd_ok == 0) {
        STC_LOG_INFO("axis%zu MYACT A8 relative_deg=%.3f speed=%.3f rc=OK", axis, effective_delta, max_speed);
        b->last_abs_via_a8_service[axis] = 1;
      } else log_myact_failure(b, axis, "A8", cmd_ok);
    } else if (b->native_abs_validated[axis]) {
      const int32_t target_0p01deg = (int32_t)llround(target_deg * gr * 100.0);
      cmd_ok = send_myact_cmd(b, node_id, MYACT_CMD_POS_ABS, speed_dps, target_0p01deg);
      if (cmd_ok == 0) {
        STC_LOG_INFO("axis%zu MYACT A4 abs_deg=%.3f speed=%.3f rc=OK", axis, target_deg, max_speed);
        b->last_abs_via_a8_service[axis] = 0;
      } else log_myact_failure(b, axis, "A4", cmd_ok);
    } else {
      const double current_deg = local_axis_counts_to_deg(cfg, b->snap.pos_raw[axis]);
      const double delta_deg = target_deg - current_deg;
      expected_target_deg = target_deg;
      const int32_t delta_0p01deg = (int32_t)llround(delta_deg * gr * 100.0);
      cmd_ok = send_myact_cmd(b, node_id, MYACT_CMD_POS_INC, speed_dps, delta_0p01deg);
      if (cmd_ok == 0) {
        STC_LOG_INFO("axis%zu MYACT A8 relative_deg=%.3f speed=%.3f rc=OK", axis, delta_deg, max_speed);
        b->last_abs_via_a8_service[axis] = 1;
      } else log_myact_failure(b, axis, "A8", cmd_ok);
    }
    if (cmd_ok == 0) {
      b->speed_active[axis] = 0;
      b->snap.mode_display[axis] = 3;
      b->snap.vel_raw[axis] = 0;
      if (b->last_abs_via_a8_service[axis]) b->snap.status_word[axis] |= (uint16_t)(1u << 10);
      else b->snap.status_word[axis] &= (uint16_t)~(1u << 10);
      b->snap.last_rx_ms[axis] = t_ms;
      b->last_cmd_ms[axis] = m->cmd_ts_ms;
      b->cmd_pos_raw[axis] = local_axis_deg_to_counts(cfg, target_deg);
      b->cmd_ack_pending[axis] = 1u;
      (void)poll_axis_feedback_once(b, axis, node_id, MYACT_CMD_READ_STATUS2, 20);
      if (wait_for_position_target_locked(b, axis, node_id, expected_target_deg) != 0) {
        b->snap.status_word[axis] |= (uint16_t)(1u << 15);
        log_myact_failure(b, axis, "target wait", -1);
      } else {
        b->snap.status_word[axis] &= (uint16_t)~(1u << 15);
      }
    }
  } else if (m->kind == AXIS_CMD_STOP) {
    const int stop_rc = axis_stop_locked(b, axis, t_ms, 1);
    if (stop_rc == 0) {
      STC_LOG_INFO("axis%zu MYACT stop rc=OK", axis);
      (void)poll_axis_feedback_once(b, axis, node_id, MYACT_CMD_READ_STATUS1, 30);
      (void)poll_axis_feedback_once(b, axis, node_id, MYACT_CMD_READ_STATUS2, 30);
      (void)poll_axis_feedback_once(b, axis, node_id, MYACT_CMD_READ_ANGLE, 30);
      b->snap.vel_raw[axis] = 0;
    } else log_myact_failure(b, axis, "stop", stop_rc);
  }

  b->applied_seq[axis] = m->seq;
}

static void refresh_telemetry_locked(struct motion_backend* b, uint64_t t_ms) {
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!b->cfg.axis_node_id[axis]) continue;
    b->snap.last_rx_ms[axis] = t_ms;
  }
}

static void enforce_watchdog_locked(struct motion_backend* b, uint64_t t_ms) {
  if (b->cfg.speed_watchdog_ms == 0) return;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!b->cfg.axis_node_id[axis] || !b->speed_active[axis]) continue;
    const uint64_t dt = t_ms - b->last_speed_cmd_ms[axis];
    if (dt > (uint64_t)b->cfg.speed_watchdog_ms) {
      axis_stop_locked(b, axis, t_ms, 0);
      continue;
    }
    if (dt > (uint64_t)(b->cfg.speed_watchdog_ms / 2)) {
      b->mailbox[axis].kind = AXIS_CMD_SPEED;
      b->mailbox[axis].value = ((double)b->last_speed_raw_0p01dps[axis]) / 100.0;
      b->mailbox[axis].cmd_ts_ms = t_ms;
    }
  }
}

static void* backend_loop(void* arg) {
  struct motion_backend* b = (struct motion_backend*)arg;

  pthread_mutex_lock(&b->lock);
  while (b->running) {
    const uint64_t t_ms = now_ms();
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) apply_axis_command_locked(b, axis, t_ms);
    refresh_feedback_locked(b, t_ms);
    refresh_telemetry_locked(b, t_ms);
    enforce_watchdog_locked(b, t_ms);

    struct timespec wake_at;
    clock_gettime(CLOCK_MONOTONIC, &wake_at);
    wake_at.tv_nsec += TELEMETRY_PERIOD_NS;
    while (wake_at.tv_nsec >= 1000000000L) {
      wake_at.tv_nsec -= 1000000000L;
      wake_at.tv_sec += 1;
    }
    (void)pthread_cond_timedwait(&b->wake, &b->lock, &wake_at);
  }
  pthread_mutex_unlock(&b->lock);
  return NULL;
}

int myact_motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend) {
  if (!cfg || !out_backend) return -1;

  struct motion_backend* b = (struct motion_backend*)calloc(1, sizeof(*b));
  if (!b) return -1;
  b->cfg = *cfg;
  b->next_seq = 1;
  pthread_mutex_init(&b->lock, NULL);
  pthread_cond_init(&b->wake, NULL);
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) b->can_socket[axis] = -1;

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    const uint8_t node_id = cfg->axis_node_id[axis];
    if (!node_id) continue;
    b->can_socket[axis] = open_can_socket(cfg->can_if, node_id);
    if (b->can_socket[axis] < 0) {
      for (size_t i = 0; i < STC_AXIS_COUNT; i++) if (b->can_socket[i] >= 0) close(b->can_socket[i]);
      pthread_cond_destroy(&b->wake);
      pthread_mutex_destroy(&b->lock);
      free(b);
      return -1;
    }
  }

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!cfg->axis_node_id[axis]) continue;
    b->snap.axis_available[axis] = 1;
    b->snap.can_connected_devices++;
  }
  b->snap.can_ok = (b->snap.can_connected_devices > 0);
  b->snap.capabilities.supports_native_a2_speed = 1;
  b->snap.capabilities.supports_native_absolute_position = 0;
  b->snap.backend_mode = 1;
  b->snap.transport_status = 1;

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!cfg->axis_node_id[axis]) continue;
    (void)init_axis_feedback(b, axis);
  }

  b->running = 1;
  if (pthread_create(&b->worker, NULL, backend_loop, b) != 0) {
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) if (b->can_socket[axis] >= 0) close(b->can_socket[axis]);
    pthread_cond_destroy(&b->wake);
    pthread_mutex_destroy(&b->lock);
    free(b);
    return -1;
  }

  *out_backend = b;
  return 0;
}

void myact_motion_backend_stop(motion_backend_t* backend) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return;

  pthread_mutex_lock(&b->lock);
  b->running = 0;
  pthread_cond_signal(&b->wake);
  pthread_mutex_unlock(&b->lock);
  (void)pthread_join(b->worker, NULL);

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) axis_stop_locked(b, axis, now_ms(), 1);

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) if (b->can_socket[axis] >= 0) close(b->can_socket[axis]);
  pthread_cond_destroy(&b->wake);
  pthread_mutex_destroy(&b->lock);
  free(b);
}

int myact_motion_backend_discover(motion_backend_t* backend) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;

  int discovered = 0;
  for (size_t axis = 0; axis < STC_AXIS_COUNT && discovered < 16; axis++) {
    if (!b->cfg.axis_node_id[axis]) continue;
    b->snap.can_discovered_nodes[discovered++] = b->cfg.axis_node_id[axis];
  }
  b->snap.can_discovered_devices = (uint8_t)discovered;
  return discovered;
}

int myact_motion_backend_apply_pp_move(motion_backend_t* backend, uint32_t axes_mask, const double targets_deg[STC_AXIS_COUNT]) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !targets_deg) return -1;

  pthread_mutex_lock(&b->lock);
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0) continue;
    b->mailbox[axis].kind = AXIS_CMD_POSITION_ABS;
    b->mailbox[axis].value = targets_deg[axis];
    b->mailbox[axis].seq = b->next_seq++;
    b->mailbox[axis].cmd_ts_ms = now_ms();
  }
  pthread_cond_signal(&b->wake);
  pthread_mutex_unlock(&b->lock);
  return 0;
}

int myact_motion_backend_apply_motion(motion_backend_t* backend, uint32_t axes_mask, const stc_axis_motion_kind_t kind[STC_AXIS_COUNT], const double values_deg_or_dps[STC_AXIS_COUNT], const double speed_limit_deg_s[STC_AXIS_COUNT]) {
  (void)speed_limit_deg_s;
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !kind || !values_deg_or_dps) return -1;
  const uint64_t t_ms = now_ms();
  pthread_mutex_lock(&b->lock);
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0) continue;
    switch (kind[axis]) {
      case STC_AXIS_MOTION_SPEED: b->mailbox[axis].kind = AXIS_CMD_SPEED; break;
      case STC_AXIS_MOTION_REL: b->mailbox[axis].kind = AXIS_CMD_POSITION_REL; break;
      case STC_AXIS_MOTION_ABS:
      default: b->mailbox[axis].kind = AXIS_CMD_POSITION_ABS; break;
    }
    b->mailbox[axis].value = values_deg_or_dps[axis];
    b->mailbox[axis].seq = b->next_seq++;
    b->mailbox[axis].cmd_ts_ms = t_ms;
  }
  pthread_cond_signal(&b->wake);
  pthread_mutex_unlock(&b->lock);
  return 0;
}

int myact_motion_backend_apply_speed(motion_backend_t* backend, uint32_t axes_mask, const double speeds_deg_s[STC_AXIS_COUNT]) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !speeds_deg_s) return -1;

  const uint64_t t_ms = now_ms();
  pthread_mutex_lock(&b->lock);
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0) continue;
    b->mailbox[axis].kind = AXIS_CMD_SPEED;
    b->mailbox[axis].value = speeds_deg_s[axis];
    b->mailbox[axis].seq = b->next_seq++;
    b->mailbox[axis].cmd_ts_ms = t_ms;
  }
  pthread_cond_signal(&b->wake);
  pthread_mutex_unlock(&b->lock);
  return 0;
}

int myact_motion_backend_quick_stop(motion_backend_t* backend, uint32_t axes_mask) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;

  const uint64_t t_ms = now_ms();
  pthread_mutex_lock(&b->lock);
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0) continue;
    b->mailbox[axis].kind = AXIS_CMD_STOP;
    b->mailbox[axis].value = 0.0;
    b->mailbox[axis].seq = b->next_seq++;
    b->mailbox[axis].cmd_ts_ms = t_ms;
  }
  pthread_cond_signal(&b->wake);
  pthread_mutex_unlock(&b->lock);
  return 0;
}

int myact_motion_backend_clear_faults(motion_backend_t* backend, uint32_t axes_mask) {
  (void)axes_mask;
  return backend ? 0 : -1;
}

int myact_motion_backend_set_laser(motion_backend_t* backend, uint8_t laser_id, uint8_t on_off) {
  (void)laser_id;
  (void)on_off;
  return backend ? 0 : -1;
}

int myact_motion_backend_get_snapshot(motion_backend_t* backend, motion_snapshot_t* snapshot) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !snapshot) return -1;

  pthread_mutex_lock(&b->lock);
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    b->snap.fault_active[axis] = (b->snap.status_word[axis] != 0u) ? 1u : 0u;
    const uint64_t cmd_ms = b->last_cmd_ms[axis];
    b->snap.command_age_ms[axis] = (cmd_ms > 0 && now_ms() >= cmd_ms) ? (uint32_t)(now_ms() - cmd_ms) : 0u;
    const int32_t pos_err = b->snap.pos_raw[axis] - b->cmd_pos_raw[axis];
    const int pos_still_far = (pos_err > 20 || pos_err < -20);
    const int vel_still_zero = (b->snap.vel_raw[axis] < 5 && b->snap.vel_raw[axis] > -5);
    b->snap.cmd_ack_no_motion[axis] = (b->cmd_ack_pending[axis] && pos_still_far && vel_still_zero && b->snap.command_age_ms[axis] > 200u) ? 1u : 0u;
    b->snap.diag_ext[axis] = (uint32_t)(b->last_abs_via_a8_service[axis] ? 1u : 0u);
  }
  *snapshot = b->snap;
  pthread_mutex_unlock(&b->lock);
  return 0;
}
