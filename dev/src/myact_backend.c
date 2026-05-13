#define _POSIX_C_SOURCE 200809L

#include "motion_backend.h"

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
  MYACT_CMD_BRAKE_RELEASE = 0x77,
  MYACT_CAN_BASE = 0x140,
  TELEMETRY_PERIOD_NS = 20 * 1000 * 1000,
};

typedef enum {
  AXIS_CMD_NONE = 0,
  AXIS_CMD_SPEED,
  AXIS_CMD_POSITION,
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
  int can_socket;
  pthread_t worker;
  int running;
  pthread_mutex_t lock;
  pthread_cond_t wake;

  motion_snapshot_t snap;
  axis_mailbox_t mailbox[STC_AXIS_COUNT];
  uint64_t next_seq;
  uint64_t applied_seq[STC_AXIS_COUNT];
  uint64_t last_speed_cmd_ms[STC_AXIS_COUNT];
  int speed_active[STC_AXIS_COUNT];
  int velocity_session_ready[STC_AXIS_COUNT];
  int32_t last_speed_raw_0p01dps[STC_AXIS_COUNT];
};

static uint64_t now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}

static int open_can_socket(const char* ifname) {
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

  if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    close(fd);
    return -1;
  }

  return fd;
}

static int send_myact_cmd(struct motion_backend* b, uint8_t node_id, uint8_t cmd, uint16_t speed_dps, int32_t value) {
  if (!b || b->can_socket < 0 || node_id == 0) return -1;

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

  return (write(b->can_socket, &f, sizeof(f)) == (ssize_t)sizeof(f)) ? 0 : -1;
}

static int32_t local_axis_deg_to_counts(const axis_cfg_t* cfg, double deg) {
  const double gr = (cfg->gear_ratio > 0.0) ? cfg->gear_ratio : 1.0;
  const double cpd = (cfg->counts_per_rev != 0) ? ((double)cfg->counts_per_rev / 360.0) : 1.0;
  return (int32_t)llround(deg * gr * cpd);
}

static double clamp_range(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}


static void normalize_snapshot_from_feedback(struct motion_backend* b, size_t axis, int16_t iq_0p01a, int16_t speed_dps, int16_t angle_deg, uint64_t t_ms) {
  const axis_cfg_t* cfg = &b->cfg.axis_cfg[axis];
  const double gr = (cfg->gear_ratio > 0.0) ? cfg->gear_ratio : 1.0;
  const double cpd = (cfg->counts_per_rev != 0) ? ((double)cfg->counts_per_rev / 360.0) : 1.0;
  const double axis_speed_deg_s = ((double)speed_dps) / gr;
  const double axis_angle_deg = ((double)angle_deg) / gr;
  b->snap.vel_raw[axis] = (int32_t)llround(axis_speed_deg_s * cpd);
  b->snap.pos_raw[axis] = (int32_t)llround(axis_angle_deg * cpd);
  b->snap.status_word[axis] = (uint16_t)iq_0p01a;
  b->snap.last_rx_ms[axis] = t_ms;
}

static void refresh_feedback_locked(struct motion_backend* b, uint64_t t_ms) {
  struct can_frame f;
  while (read(b->can_socket, &f, sizeof(f)) == (ssize_t)sizeof(f)) {
    if (f.can_dlc < 8 || f.can_id < MYACT_CAN_BASE) continue;
    const uint8_t node_id = (uint8_t)(f.can_id - MYACT_CAN_BASE);
    size_t axis;
    for (axis = 0; axis < STC_AXIS_COUNT; axis++) if (b->cfg.axis_node_id[axis] == node_id) break;
    if (axis >= STC_AXIS_COUNT) continue;
    if (f.data[0] == MYACT_CMD_SPEED || f.data[0] == MYACT_CMD_POS_ABS) {
      const int16_t iq_0p01a = (int16_t)((uint16_t)f.data[2] | ((uint16_t)f.data[3] << 8));
      const int16_t speed_dps = (int16_t)((uint16_t)f.data[4] | ((uint16_t)f.data[5] << 8));
      const int16_t angle_deg = (int16_t)((uint16_t)f.data[6] | ((uint16_t)f.data[7] << 8));
      normalize_snapshot_from_feedback(b, axis, iq_0p01a, speed_dps, angle_deg, t_ms);
    }
  }
}

static void axis_stop_locked(struct motion_backend* b, size_t axis, uint64_t t_ms) {
  const uint8_t node_id = b->cfg.axis_node_id[axis];
  if (node_id) (void)send_myact_cmd(b, node_id, MYACT_CMD_STOP, 0, 0);
  b->speed_active[axis] = 0;
  b->velocity_session_ready[axis] = 0;
  b->last_speed_raw_0p01dps[axis] = 0;
  b->snap.vel_raw[axis] = 0;
  b->snap.mode_display[axis] = 0;
  b->snap.last_rx_ms[axis] = t_ms;
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
    if (send_myact_cmd(b, node_id, MYACT_CMD_SPEED, 0, speed_0p01dps) == 0) {
      b->speed_active[axis] = (speed_0p01dps != 0);
      b->last_speed_raw_0p01dps[axis] = speed_0p01dps;
      b->last_speed_cmd_ms[axis] = m->cmd_ts_ms;
      b->snap.vel_raw[axis] = local_axis_deg_to_counts(cfg, clamped);
      b->snap.mode_display[axis] = (speed_0p01dps != 0) ? 2 : 0;
      b->snap.status_word[axis] &= (uint16_t)~(1u << 10);
      b->snap.last_rx_ms[axis] = t_ms;
    }
  } else if (m->kind == AXIS_CMD_POSITION) {
    const double target_deg = clamp_range(m->value, cfg->az_min_deg, cfg->az_max_deg);
    const int32_t target_0p01deg = (int32_t)llround(target_deg * gr * 100.0);
    const uint16_t speed_dps = (uint16_t)fmax(1.0, fmin(65535.0, max_speed * gr));
    if (send_myact_cmd(b, node_id, MYACT_CMD_POS_ABS, speed_dps, target_0p01deg) == 0) {
      b->speed_active[axis] = 0;
      b->snap.mode_display[axis] = 3;
      b->snap.vel_raw[axis] = 0;
      b->snap.status_word[axis] &= (uint16_t)~(1u << 10);
      b->snap.last_rx_ms[axis] = t_ms;
    }
  } else if (m->kind == AXIS_CMD_STOP) {
    axis_stop_locked(b, axis, t_ms);
  }

  b->applied_seq[axis] = m->seq;
}

static void refresh_telemetry_locked(struct motion_backend* b, uint64_t t_ms) {
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!b->cfg.axis_node_id[axis]) continue;
    b->snap.status_word[axis] &= (uint16_t)~(1u << 10);
    b->snap.last_rx_ms[axis] = t_ms;
  }
}

static void enforce_watchdog_locked(struct motion_backend* b, uint64_t t_ms) {
  if (b->cfg.speed_watchdog_ms == 0) return;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!b->cfg.axis_node_id[axis] || !b->speed_active[axis]) continue;
    const uint64_t dt = t_ms - b->last_speed_cmd_ms[axis];
    if (dt > (uint64_t)b->cfg.speed_watchdog_ms) {
      axis_stop_locked(b, axis, t_ms);
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

  b->can_socket = open_can_socket(cfg->can_if);
  if (b->can_socket < 0) {
    pthread_cond_destroy(&b->wake);
    pthread_mutex_destroy(&b->lock);
    free(b);
    return -1;
  }

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!cfg->axis_node_id[axis]) continue;
    b->snap.axis_available[axis] = 1;
    b->snap.can_connected_devices++;
  }
  b->snap.can_ok = (b->snap.can_connected_devices > 0);
  b->snap.capabilities.supports_native_a2_speed = 1;
  b->snap.capabilities.supports_native_absolute_position = 1;
  b->snap.backend_mode = 1;
  b->snap.transport_status = 1;

  b->running = 1;
  if (pthread_create(&b->worker, NULL, backend_loop, b) != 0) {
    close(b->can_socket);
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

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) axis_stop_locked(b, axis, now_ms());

  close(b->can_socket);
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
    b->mailbox[axis].kind = AXIS_CMD_POSITION;
    b->mailbox[axis].value = targets_deg[axis];
    b->mailbox[axis].seq = b->next_seq++;
    b->mailbox[axis].cmd_ts_ms = now_ms();
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
  *snapshot = b->snap;
  pthread_mutex_unlock(&b->lock);
  return 0;
}
