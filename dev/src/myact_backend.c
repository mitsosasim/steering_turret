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
  MYACT_CMD_POS_INC = 0xA8,
  MYACT_CAN_BASE = 0x140,
  BACKEND_PERIOD_NS = 20 * 1000 * 1000,
  BACKEND_PERIOD_MS = 20,
};

typedef enum {
  AXIS_IDLE = 0,
  AXIS_MANUAL = 1,
  AXIS_TARGET = 2,
} axis_mode_t;

struct motion_backend {
  motion_backend_cfg_t cfg;
  int can_socket;
  pthread_t worker;
  int running;
  pthread_mutex_t lock;

  motion_snapshot_t snap;
  axis_mode_t mode[STC_AXIS_COUNT];
  double target_deg[STC_AXIS_COUNT];
  double manual_speed_deg_s[STC_AXIS_COUNT];
  double profile_vel_deg_s[STC_AXIS_COUNT];
  uint64_t last_speed_cmd_ms[STC_AXIS_COUNT];
  uint8_t sat_mask[STC_AXIS_COUNT];
};

static uint64_t now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}

static void normalize_deadline(struct timespec* t) {
  while (t->tv_nsec >= 1000000000L) {
    t->tv_nsec -= 1000000000L;
    t->tv_sec += 1;
  }
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

static double counts_to_axis_deg(const axis_cfg_t* cfg, int32_t counts) {
  const double gr = (cfg->gear_ratio > 0.0) ? cfg->gear_ratio : 1.0;
  const double dpc = (cfg->counts_per_rev != 0) ? (360.0 / (double)cfg->counts_per_rev) : 1.0;
  return ((double)counts * dpc) / gr;
}

static int32_t local_axis_deg_to_counts(const axis_cfg_t* cfg, double deg) {
  const double gr = (cfg->gear_ratio > 0.0) ? cfg->gear_ratio : 1.0;
  const double cpd = (cfg->counts_per_rev != 0) ? ((double)cfg->counts_per_rev / 360.0) : 1.0;
  return (int32_t)llround(deg * gr * cpd);
}

static double clamp_range(double v, double lo, double hi, int* clipped) {
  if (v < lo) {
    if (clipped) *clipped = 1;
    return lo;
  }
  if (v > hi) {
    if (clipped) *clipped = 1;
    return hi;
  }
  return v;
}

static double accel_step(double current, double desired, double max_accel, double dt_s) {
  double dv = desired - current;
  const double max_dv = max_accel * dt_s;
  if (dv > max_dv) dv = max_dv;
  if (dv < -max_dv) dv = -max_dv;
  return current + dv;
}

static double goto_position_profile(double err_deg, double current_vel_deg_s, double max_speed_deg_s, double max_accel_deg_s2, double dt_s) {
  const double stop_dist = (current_vel_deg_s * current_vel_deg_s) / (2.0 * max_accel_deg_s2);
  double desired_v = (err_deg > 0.0) ? max_speed_deg_s : -max_speed_deg_s;
  if (fabs(err_deg) <= stop_dist) desired_v = 0.0;
  return accel_step(current_vel_deg_s, desired_v, max_accel_deg_s2, dt_s);
}

static double jog_velocity_profile(double requested_vel_deg_s, double current_vel_deg_s, double max_speed_deg_s, double max_accel_deg_s2, double dt_s, int* saturated) {
  int sat = 0;
  const double v = clamp_range(requested_vel_deg_s, -max_speed_deg_s, max_speed_deg_s, &sat);
  if (saturated) *saturated = sat;
  return accel_step(current_vel_deg_s, v, max_accel_deg_s2, dt_s);
}

static void axis_stop(struct motion_backend* b, size_t axis) {
  const uint8_t node_id = b->cfg.axis_node_id[axis];
  if (node_id) (void)send_myact_cmd(b, node_id, MYACT_CMD_STOP, 0, 0);
  b->mode[axis] = AXIS_IDLE;
  b->manual_speed_deg_s[axis] = 0.0;
  b->snap.vel_raw[axis] = 0;
  b->snap.mode_display[axis] = 0;
}

static void axis_step_profile(struct motion_backend* b, size_t axis, double dt_s, uint64_t t_ms) {
  const uint8_t node_id = b->cfg.axis_node_id[axis];
  if (!node_id) return;

  if (b->mode[axis] == AXIS_MANUAL && b->cfg.speed_watchdog_ms > 0) {
    const uint64_t age_ms = t_ms - b->last_speed_cmd_ms[axis];
    if (age_ms > (uint64_t)b->cfg.speed_watchdog_ms) {
      axis_stop(b, axis);
      return;
    }
  }

  double delta_deg = 0.0;
  int reached_target = 0;
  int sat = 0;
  const axis_cfg_t* cfg = &b->cfg.axis_cfg[axis];
  const double travel_min = cfg->az_min_deg;
  const double travel_max = cfg->az_max_deg;
  const double max_speed = fmax(0.1, cfg->max_speed_deg_s);
  const double max_accel = fmax(0.1, b->cfg.profile_max_accel_deg_s2);
  const double target_tol_deg = 0.05;

  if (b->mode[axis] == AXIS_MANUAL) {
    b->profile_vel_deg_s[axis] = jog_velocity_profile(
      b->manual_speed_deg_s[axis], b->profile_vel_deg_s[axis], max_speed, max_accel, dt_s, &sat);
    delta_deg = b->profile_vel_deg_s[axis] * dt_s;
  } else if (b->mode[axis] == AXIS_TARGET) {
    const double pos_deg = counts_to_axis_deg(cfg, b->snap.pos_raw[axis]);
    double target = b->target_deg[axis];
    target = clamp_range(target, travel_min, travel_max, &sat);
    b->target_deg[axis] = target;
    const double err = target - pos_deg;
    if (fabs(err) <= target_tol_deg) {
      reached_target = 1;
      b->mode[axis] = AXIS_IDLE;
      b->profile_vel_deg_s[axis] = 0.0;
    } else {
      b->profile_vel_deg_s[axis] = goto_position_profile(err, b->profile_vel_deg_s[axis], max_speed, max_accel, dt_s);
      delta_deg = b->profile_vel_deg_s[axis] * dt_s;
      if (fabs(delta_deg) > fabs(err)) delta_deg = err;
    }
  }

  if (reached_target) {
    b->snap.status_word[axis] |= (1u << 10);
    b->snap.mode_display[axis] = 3;
    b->snap.vel_raw[axis] = 0;
    b->sat_mask[axis] = (uint8_t)sat;
    return;
  }

  if (fabs(delta_deg) <= 1e-6) {
    b->sat_mask[axis] = (uint8_t)sat;
    return;
  }

  double pos_deg = counts_to_axis_deg(cfg, b->snap.pos_raw[axis]);
  double next_pos_deg = pos_deg + delta_deg;
  next_pos_deg = clamp_range(next_pos_deg, travel_min, travel_max, &sat);
  delta_deg = next_pos_deg - pos_deg;

  const double gr = (cfg->gear_ratio > 0.0) ? cfg->gear_ratio : 1.0;
  const int32_t delta_motor_0p01deg = (int32_t)llround(delta_deg * gr * 100.0);
  const uint16_t max_speed_dps = (uint16_t)fmax(1.0, fmin(65535.0, max_speed * gr));

  if (send_myact_cmd(b, node_id, MYACT_CMD_POS_INC, max_speed_dps, delta_motor_0p01deg) == 0) {
    b->snap.pos_raw[axis] += local_axis_deg_to_counts(cfg, delta_deg);
    b->snap.vel_raw[axis] = local_axis_deg_to_counts(cfg, delta_deg / dt_s);
    b->snap.mode_display[axis] = (b->mode[axis] == AXIS_MANUAL) ? 2 : 3;
    b->snap.status_word[axis] &= (uint16_t)~(1u << 10);
    b->snap.last_rx_ms[axis] = t_ms;
    b->sat_mask[axis] = (uint8_t)sat;
  }
}

static void* backend_loop(void* arg) {
  struct motion_backend* b = (struct motion_backend*)arg;
  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);

  while (b->running) {
    pthread_mutex_lock(&b->lock);
    const uint64_t t_ms = now_ms();
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      axis_step_profile(b, axis, (double)BACKEND_PERIOD_MS / 1000.0, t_ms);
    }
    pthread_mutex_unlock(&b->lock);

    next.tv_nsec += BACKEND_PERIOD_NS;
    normalize_deadline(&next);
    (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
  }

  return NULL;
}

int myact_motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend) {
  if (!cfg || !out_backend) return -1;

  struct motion_backend* b = (struct motion_backend*)calloc(1, sizeof(*b));
  if (!b) return -1;
  b->cfg = *cfg;
  pthread_mutex_init(&b->lock, NULL);

  b->can_socket = open_can_socket(cfg->can_if);
  if (b->can_socket < 0) {
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

  b->running = 0;
  (void)pthread_join(b->worker, NULL);

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) axis_stop(b, axis);

  close(b->can_socket);
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
    b->mode[axis] = AXIS_TARGET;
    b->target_deg[axis] = targets_deg[axis];
  }
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
    b->mode[axis] = AXIS_MANUAL;
    b->manual_speed_deg_s[axis] = speeds_deg_s[axis];
    b->last_speed_cmd_ms[axis] = t_ms;
  }
  pthread_mutex_unlock(&b->lock);
  return 0;
}

int myact_motion_backend_quick_stop(motion_backend_t* backend, uint32_t axes_mask) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;

  pthread_mutex_lock(&b->lock);
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0) continue;
    axis_stop(b, axis);
  }
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
