/*
  turret_motion_demo.c — standalone PD4-C turret motion test (no TCP server)

  Build:
    gcc -O2 -Wall -Wextra -std=c11 -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L \
      -Idev/include dev/tests/turret_motion_demo.c dev/src/pd4c.c -lm -o turret_motion_demo

  Run:
    ./turret_motion_demo <can_iface> <node_id> [motor_counts_per_rev] [gear_ratio]

  Defaults:
    motor_counts_per_rev = 4096
    gear_ratio           = 100

  This reproduces the same command sequence as the former turret_motion_demo.sh,
  but drives the backend CAN library directly (pd4c) instead of TCP comms.
*/

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pd4c.h"

#define MIN_DEG (-140.0)
#define MAX_DEG (140.0)
#define HOME_DEG (0.0)
#define MAX_LOAD_SPEED_DEG_S (25.0)
#define STEP_DEG (0.1)
#define STEP_INTERVAL_S (2)

#define HOLD_CENTER_S (3)
#define HOLD_LIMIT_S (4)
#define HOME_SETTLE_S (8)

#define MOVE_140_S (5.8)
#define MOVE_280_S (11.4)

typedef struct motion_cfg_s {
  double motor_counts_per_rev;
  double gear_ratio;
  double counts_per_turret_deg;
  uint16_t profile_vel_raw;
} motion_cfg_t;

static volatile sig_atomic_t g_stop = 0;
static pd4c_ctx_t* g_ctx = NULL;
static motion_cfg_t g_cfg;

static void msleep(int ms) {
  struct timespec ts;
  ts.tv_sec = ms / 1000;
  ts.tv_nsec = (long)(ms % 1000) * 1000000L;
  nanosleep(&ts, NULL);
}

static void sleep_s(double sec) {
  if (sec <= 0.0) return;
  int ms = (int)llround(sec * 1000.0);
  if (ms < 0) ms = 0;
  msleep(ms);
}

static int32_t deg_to_counts(const motion_cfg_t* cfg, double deg) {
  double counts = deg * cfg->counts_per_turret_deg;
  if (counts > (double)INT32_MAX) counts = (double)INT32_MAX;
  if (counts < (double)INT32_MIN) counts = (double)INT32_MIN;
  return (int32_t)llround(counts);
}

static int16_t turret_deg_s_to_target_vel_raw(const motion_cfg_t* cfg, double turret_deg_s) {
  double motor_deg_s = turret_deg_s * cfg->gear_ratio;
  if (motor_deg_s > (double)INT16_MAX) motor_deg_s = (double)INT16_MAX;
  if (motor_deg_s < (double)INT16_MIN) motor_deg_s = (double)INT16_MIN;
  return (int16_t)llround(motor_deg_s);
}

static void log_msg(const char* msg) {
  fprintf(stdout, "%s\n", msg);
  fflush(stdout);
}

static int enable_mode(pd4c_ctx_t* ctx, int8_t mode) {
  int rc = pd4c_ds402_enable_operation(ctx, mode, 2000);
  if (rc != 0) {
    fprintf(stderr, "Enable mode %d failed: %s (%d)\n", (int)mode, pd4c_strerror(rc), rc);
  }
  return rc;
}

static int send_speed_deg_s(pd4c_ctx_t* ctx, const motion_cfg_t* cfg, double speed_deg_s) {
  if (enable_mode(ctx, PD4C_MODE_VELOCITY) != 0) return -1;
  int16_t raw = turret_deg_s_to_target_vel_raw(cfg, speed_deg_s);
  int rc = pd4c_rpdo3_send_target_velocity(ctx, raw);
  if (rc != 0) {
    fprintf(stderr, "Velocity command failed (%.3f deg/s -> %d): %s (%d)\n",
            speed_deg_s, (int)raw, pd4c_strerror(rc), rc);
  }
  return rc;
}

static int send_abs_deg(pd4c_ctx_t* ctx, const motion_cfg_t* cfg, double deg) {
  if (enable_mode(ctx, PD4C_MODE_PROFILE_POSITION) != 0) return -1;
  const double clamped = pd4c_deg_clamp(deg, MIN_DEG, MAX_DEG);
  const int32_t target_counts = deg_to_counts(cfg, clamped);
  int rc = pd4c_pp_move(ctx, target_counts, cfg->profile_vel_raw, 0, 2500);
  if (rc != 0) {
    fprintf(stderr, "Absolute move failed (%.3f deg -> %d counts): %s (%d)\n",
            clamped, target_counts, pd4c_strerror(rc), rc);
  }
  return rc;
}

static void safe_stop_and_home(void) {
  if (!g_ctx) return;
  log_msg("Safety stop + home");
  (void)send_speed_deg_s(g_ctx, &g_cfg, 0.0);
  sleep_s(1.0);
  (void)send_abs_deg(g_ctx, &g_cfg, HOME_DEG);
  sleep_s(HOME_SETTLE_S);
}

static void on_interrupt(int signo) {
  (void)signo;
  g_stop = 1;
  safe_stop_and_home();
}

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr,
            "Usage: %s <can_iface> <node_id> [motor_counts_per_rev] [gear_ratio]\n",
            argv[0]);
    return 2;
  }

  const char* ifname = argv[1];
  int node = atoi(argv[2]);
  if (node <= 0 || node > 127) {
    fprintf(stderr, "Invalid node id: %d\n", node);
    return 2;
  }

  g_cfg.motor_counts_per_rev = (argc >= 4) ? atof(argv[3]) : 4096.0;
  g_cfg.gear_ratio = (argc >= 5) ? atof(argv[4]) : 100.0;
  if (g_cfg.motor_counts_per_rev <= 0.0 || g_cfg.gear_ratio <= 0.0) {
    fprintf(stderr, "motor_counts_per_rev and gear_ratio must be > 0\n");
    return 2;
  }

  g_cfg.counts_per_turret_deg = (g_cfg.motor_counts_per_rev * g_cfg.gear_ratio) / 360.0;
  g_cfg.profile_vel_raw = (uint16_t)llround(fabs(MAX_LOAD_SPEED_DEG_S * g_cfg.gear_ratio));

  pd4c_ctx_t ctx;
  memset(&ctx, 0, sizeof(ctx));
  g_ctx = &ctx;

  signal(SIGINT, on_interrupt);
  signal(SIGTERM, on_interrupt);

  int rc = pd4c_open(&ctx, ifname, (uint8_t)node);
  if (rc != 0) {
    fprintf(stderr, "pd4c_open failed: %s (%d)\n", pd4c_strerror(rc), rc);
    return 1;
  }

  rc = pd4c_nmt_start(&ctx);
  if (rc != 0) {
    fprintf(stderr, "pd4c_nmt_start failed: %s (%d)\n", pd4c_strerror(rc), rc);
    pd4c_close(&ctx);
    return 1;
  }

  log_msg("Starting turret motion demo (standalone pd4c)");

  log_msg("Normalize start position to HOME");
  if (send_abs_deg(&ctx, &g_cfg, HOME_DEG) != 0) goto fail;
  sleep_s(HOME_SETTLE_S);

  log_msg("Hold center");
  sleep_s(HOLD_CENTER_S);

  log_msg("Move to MIN at max load speed");
  if (send_speed_deg_s(&ctx, &g_cfg, -MAX_LOAD_SPEED_DEG_S) != 0) goto fail;
  sleep_s(MOVE_140_S);
  if (send_speed_deg_s(&ctx, &g_cfg, 0.0) != 0) goto fail;
  log_msg("Hold at MIN");
  sleep_s(HOLD_LIMIT_S);

  log_msg("Move to MAX at max load speed");
  if (send_speed_deg_s(&ctx, &g_cfg, MAX_LOAD_SPEED_DEG_S) != 0) goto fail;
  sleep_s(MOVE_280_S);
  if (send_speed_deg_s(&ctx, &g_cfg, 0.0) != 0) goto fail;
  log_msg("Hold at MAX");
  sleep_s(HOLD_LIMIT_S);

  log_msg("Return HOME at max load speed");
  if (send_speed_deg_s(&ctx, &g_cfg, -MAX_LOAD_SPEED_DEG_S) != 0) goto fail;
  sleep_s(MOVE_140_S);
  if (send_speed_deg_s(&ctx, &g_cfg, 0.0) != 0) goto fail;
  sleep_s(2.0);

  log_msg("Granularity demo: absolute steps every 2s");
  for (double pos = HOME_DEG; pos <= MAX_DEG + 1e-9; pos += STEP_DEG) {
    if (g_stop) break;
    if (send_abs_deg(&ctx, &g_cfg, pos) != 0) goto fail;
    sleep_s((double)STEP_INTERVAL_S);
  }

  log_msg("Granularity demo done; returning HOME");
  if (send_abs_deg(&ctx, &g_cfg, HOME_DEG) != 0) goto fail;
  sleep_s(HOME_SETTLE_S);

  log_msg("Turret motion demo complete");
  pd4c_close(&ctx);
  return 0;

fail:
  safe_stop_and_home();
  pd4c_close(&ctx);
  return 1;
}
