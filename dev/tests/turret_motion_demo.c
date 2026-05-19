/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Standalone calibrated bench/demo motion tool for PD4-C/PD6-C tuning.
 *
 * Design notes:
 *   This is not the normal multi-threaded stc_server runtime path.
 *   It is a single-node commissioning/demo utility with explicit safety flow:
 *   read SDO diagnostics, optionally apply startup limits, run bounded motion,
 *   and return to safe stop/home behavior on completion or interrupt.
 */


/*
  turret_motion_demo_pd6_calibrated.c
  Deterministic standalone turret demo for Nanotec PD4-C / PD6-C.

  What changed versus the previous revision:
    1) HOME default is now raw 0.
    2) We DO NOT use the physical encoder CPR from the datasheet for 6064/607A scaling.
       We use the drive's active virtual position resolution from 608F whenever available.
       For the PD6-C bench motor this is currently 4096/1, even though the integrated encoder
       hardware is 1024 cpr.
    3) We separate:
         - nominal gearbox ratio (motor -> gearbox output), and
         - extra mechanical ratio (gear train / mast geometry not represented in the drive OD).
       This extra factor is required by your measured mast motion.
    4) We optionally widen 607D software limits at startup so corrected ±140 deg targets fit.
    5) We keep 60A8 / 60A9 / 6091 / 6092 as LOG ONLY by default.
    6) CTRL+C now still performs safety stop + HOME correctly.

  Safe operational philosophy:
    - Stop NanoJ if running.
    - Do NOT rewrite unit-stack objects every run.
    - Use the measured effective total ratio in software.
    - Only auto-write software position limits (607D) when enabled below.
    - Do NOT auto-raise max motor speed (6080) unless you deliberately enable it.

  Build example:
    gcc -O2 -Wall -Wextra -std=c11 -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L \
      -Idev/include dev/tests/turret_motion_demo_pd6_calibrated.c dev/src/pd4c.c -lm \
      -o turret_motion_demo

  Run example:
    ./turret_motion_demo can0 1
    ./turret_motion_demo can0 1 1024 100 0 2.65 0

  CLI:
    ./turret_motion_demo <can_iface> <node_id>
                         [legacy_encoder_cpr_hint]
                         [gear_ratio]
                         [home_raw]
                         [extra_mech_ratio]
                         [drive_units_per_motor_rev_override]

  Notes on CLI:
    - legacy_encoder_cpr_hint is kept only for backwards compatibility and logging.
      If 608F is readable and non-zero, 608F wins for raw 6064/607A scaling.
    - drive_units_per_motor_rev_override=0 means "auto from 608F".
*/

#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <linux/can.h>

#include "pd4c.h"

#define DEMO_MIN_DEG              (-140.0)
#define DEMO_MAX_DEG              (+140.0)
#define DEMO_HOME_DEG             (0.0)
#define DEMO_STEP_DEG             (0.1)
#define DEMO_STEP_INTERVAL_S      (5)
#define DEMO_HOLD_HOME_S          (3)
#define DEMO_HOLD_LIMIT_S         (4)
#define DEMO_MAX_LOAD_SPEED_DEG_S (25.0)

#define DEFAULT_LEGACY_CPR_HINT          (1024.0)
/* Previous simple assumption:
 *   counts_per_deg = 4096 * 100 / 360
 * That matched the gearbox only, but not the full mast kinematics.
 */
#define DEFAULT_GEAR_RATIO               (100.0)
#define DEFAULT_EXTRA_MECH_RATIO         (2.65)   /* empirical starting point from bench observations */
#define DEFAULT_HOME_RAW                 (0)
#define DEFAULT_INVERT_DIR               (false)
#define DEFAULT_DRIVE_UNITS_PER_MOTOR_REV_OVERRIDE (0.0)

#define SDO_POLL_MS               (50)
#define ENABLE_TIMEOUT_MS         (5000)
#define MOVE_TIMEOUT_MS           (45000)

#define AUTO_WRITE_SOFT_LIMITS       (1)

/* Motion-limit startup writes for the demo bench.
 * Set back to 0 if you want pure log-only behavior again.
 */
#define AUTO_WRITE_MOTION_LIMITS     (1)

/* Demo tuning targets in the drive's current raw user units.
 * From the measured/logged behavior:
 *   - 25 deg/s at mast with total_ratio ~= 265 needs about 1104 rpm.
 *   - 1200 gives a little headroom.
 *   - 5000 accel/decel is a sane first step and matches common max accel defaults.
 *
 * Previous behavior (kept here for easy revert/reference):
 *   AUTO_WRITE_MAX_MOTOR_SPEED = 0
 *   no startup writes to 607F/6080/6083/6084/6085/6086/60C5/60C6
 */
#define STARTUP_MAX_PROFILE_VEL_RAW  (1200u)  /* 607F */
#define STARTUP_MAX_MOTOR_SPEED_RAW  (1200u)  /* 6080 */
#define STARTUP_PROFILE_ACCEL_RAW    (5000u)  /* 6083 */
#define STARTUP_PROFILE_DECEL_RAW    (5000u)  /* 6084 */
#define STARTUP_QUICKSTOP_DECEL_RAW  (5000u)  /* 6085 */
#define STARTUP_MAX_ACCEL_RAW        (5000u)  /* 60C5 */
#define STARTUP_MAX_DECEL_RAW        (5000u)  /* 60C6 */
#define STARTUP_MOTION_PROFILE_TYPE  (0u)     /* 6086: 0=trapezoidal, 3=jerk-limited */

#define SOFT_LIMIT_MARGIN_COUNTS   (20000)
#define SOFT_LIMIT_MIN_ABS_COUNTS  (500000)

/* If you deliberately want to try changing the drive's internal notion of gear ratio,
 * enable the block below. This is OFF by default because the demo does not need it.
 */
#define FORCE_WRITE_6091_GEAR_RATIO (0)

static volatile sig_atomic_t g_stop = 0;

typedef struct demo_cfg_s {
  double legacy_encoder_cpr_hint;       /* informational only */
  double gear_ratio;                    /* mechanical engineer guaranteed gearbox ratio */
  double extra_mech_ratio;              /* empirical multiplier from bench observations */
  double total_ratio;                   /* motor rev / mast rev = gear_ratio * extra_mech_ratio */

  double drive_units_per_motor_rev_override; /* 0 => use 608F */
  double drive_units_per_motor_rev;     /* active raw 6064 units per motor revolution */

  double counts_per_deg;                /* raw 6064 counts per mast degree */
  double deg_per_count;

  int32_t home_raw;                     /* raw 6064 that represents HOME */

  bool invert_dir;
  int profile_vel_bits;                 /* 16 or 32 depending on 1601:02 */
  uint32_t max_profile_vel_raw;         /* 607F */
  uint32_t max_motor_speed_raw;         /* 6080 */
  uint32_t profile_vel_current_raw;     /* 6081 */
  uint32_t profile_accel_raw;           /* 6083 */
  uint32_t profile_decel_raw;           /* 6084 */
  uint32_t quickstop_decel_raw;         /* 6085 */
  uint32_t motion_profile_type_raw;     /* 6086 */
  uint32_t max_accel_raw;               /* 60C5 */
  uint32_t max_decel_raw;               /* 60C6 */
  int32_t sw_pos_min_raw;               /* 607D:01 */
  int32_t sw_pos_max_raw;               /* 607D:02 */

  /* OD logging */
  uint32_t od_60a8;
  uint32_t od_60a9;
  uint32_t od_608f_num;
  uint32_t od_608f_den;
  uint32_t od_6091_num;
  uint32_t od_6091_den;
  uint32_t od_6092_num;
  uint32_t od_6092_den;
} demo_cfg_t;

static uint64_t now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

static void msleep(int ms) {
  if (ms <= 0) return;
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

static int ds402_state(uint16_t sw) {
  uint16_t m = sw & 0x006F;
  if (m == 0x0000) return 0;
  if (m == 0x0040) return 1;
  if (m == 0x0021) return 2;
  if (m == 0x0023) return 3;
  if (m == 0x0027) return 4;
  if (m == 0x0007) return 5;
  if (m == 0x000F) return 6;
  if (m == 0x0008) return 7;
  return 99;
}

static const char* ds402_state_str(int st) {
  switch (st) {
    case 0: return "NOT READY";
    case 1: return "SWITCH ON DISABLED";
    case 2: return "READY TO SWITCH ON";
    case 3: return "SWITCHED ON";
    case 4: return "OPERATION ENABLED";
    case 5: return "QUICK STOP ACTIVE";
    case 6: return "FAULT REACTION";
    case 7: return "FAULT";
    default: return "UNKNOWN";
  }
}

static int read_u8(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint8_t* out) {
  return pd4c_sdo_read_u8(ctx, idx, sub, out);
}
static int read_u16(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint16_t* out) {
  return pd4c_sdo_read_u16(ctx, idx, sub, out);
}
static int read_u32(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint32_t* out) {
  return pd4c_sdo_read_u32(ctx, idx, sub, out);
}
static int read_i32(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, int32_t* out) {
  return pd4c_sdo_read_i32(ctx, idx, sub, out);
}

static double raw_to_deg(const demo_cfg_t* cfg, int32_t raw) {
  double rel = (double)(raw - cfg->home_raw);
  double deg = rel * cfg->deg_per_count;
  return cfg->invert_dir ? -deg : deg;
}

static int32_t deg_to_raw(const demo_cfg_t* cfg, double deg) {
  double d = cfg->invert_dir ? -deg : deg;
  double rel_counts = d * cfg->counts_per_deg;
  double raw = (double)cfg->home_raw + rel_counts;
  if (raw > (double)INT32_MAX) raw = (double)INT32_MAX;
  if (raw < (double)INT32_MIN) raw = (double)INT32_MIN;
  return (int32_t)llround(raw);
}

/* 6081 / 606C empirically behave as rpm on this bench setup.
 * Therefore:
 *   mast_deg_s = motor_rpm * 6 / total_ratio
 *   motor_rpm  = mast_deg_s * total_ratio / 6
 */
static uint32_t mast_deg_s_to_profile_vel_rpm_raw(const demo_cfg_t* cfg, double mast_deg_s) {
  double s = fabs(mast_deg_s);
  double motor_rpm = (s * cfg->total_ratio) / 6.0;
  uint32_t raw = (uint32_t)llround(motor_rpm);
  if (cfg->max_motor_speed_raw > 0 && raw > cfg->max_motor_speed_raw) {
    raw = cfg->max_motor_speed_raw;
  }
  return raw;
}

static double profile_vel_rpm_raw_to_mast_deg_s(const demo_cfg_t* cfg, int32_t rpm_raw) {
  return ((double)rpm_raw * 6.0) / cfg->total_ratio;
}

static int detect_profile_vel_bits(pd4c_ctx_t* ctx) {
  uint32_t map = 0;
  if (read_u32(ctx, 0x1601, 0x02, &map) != 0) return 16;
  uint8_t bits = (uint8_t)(map & 0xFFu);
  if (bits == 0x20) return 32;
  if (bits == 0x10) return 16;
  return 16;
}

static void stop_nanoj_if_running(pd4c_ctx_t* ctx) {
  uint32_t nanoj_ctrl = 0, nanoj_stat = 0;
  if (read_u32(ctx, 0x2300, 0x00, &nanoj_ctrl) != 0) return;
  if (read_u32(ctx, 0x2301, 0x00, &nanoj_stat) != 0) return;

  if (((nanoj_ctrl | nanoj_stat) & 1u) == 0u) {
    printf("[init] NanoJ not running (2300=0x%08" PRIX32 ", 2301=0x%08" PRIX32 ")\n",
           nanoj_ctrl, nanoj_stat);
    fflush(stdout);
    return;
  }

  printf("[init] NanoJ appears running (2300=0x%08" PRIX32 ", 2301=0x%08" PRIX32 "). Stopping...\n",
         nanoj_ctrl, nanoj_stat);
  fflush(stdout);

  (void)pd4c_sdo_write_u32(ctx, 0x2300, 0x00, 0u);
  msleep(250);

  (void)read_u32(ctx, 0x2300, 0x00, &nanoj_ctrl);
  (void)read_u32(ctx, 0x2301, 0x00, &nanoj_stat);

  printf("[init] NanoJ after stop: 2300=0x%08" PRIX32 " 2301=0x%08" PRIX32 "\n",
         nanoj_ctrl, nanoj_stat);
  fflush(stdout);
}

static void maybe_write_6091_gear_ratio(pd4c_ctx_t* ctx, double gear_ratio) {
#if FORCE_WRITE_6091_GEAR_RATIO
  uint32_t num = 0, den = 0;
  (void)read_u32(ctx, 0x6091, 0x01, &num);
  (void)read_u32(ctx, 0x6091, 0x02, &den);
  uint32_t want_num = (uint32_t)llround(gear_ratio);
  uint32_t want_den = 1u;
  if (num != want_num || den != want_den) {
    printf("[init] Writing 6091 gear ratio to %" PRIu32 "/%" PRIu32 "\n", want_num, want_den);
    fflush(stdout);
    (void)pd4c_sdo_write_u32(ctx, 0x6091, 0x01, want_num);
    (void)pd4c_sdo_write_u32(ctx, 0x6091, 0x02, want_den);
    msleep(50);
  }
#else
  (void)ctx;
  (void)gear_ratio;
#endif
}

static void read_scaling_objects(pd4c_ctx_t* ctx, demo_cfg_t* cfg) {
  int32_t tmp_i32 = 0;

  cfg->od_60a8 = 0;
  cfg->od_60a9 = 0;
  cfg->od_608f_num = 0;
  cfg->od_608f_den = 0;
  cfg->od_6091_num = 0;
  cfg->od_6091_den = 0;
  cfg->od_6092_num = 0;
  cfg->od_6092_den = 0;
  cfg->max_profile_vel_raw = 0;
  cfg->max_motor_speed_raw = 0;
  cfg->profile_vel_current_raw = 0;
  cfg->profile_accel_raw = 0;
  cfg->profile_decel_raw = 0;
  cfg->quickstop_decel_raw = 0;
  cfg->motion_profile_type_raw = 0;
  cfg->max_accel_raw = 0;
  cfg->max_decel_raw = 0;
  cfg->sw_pos_min_raw = 0;
  cfg->sw_pos_max_raw = 0;

  (void)read_u32(ctx, 0x60A8, 0x00, &cfg->od_60a8);
  (void)read_u32(ctx, 0x60A9, 0x00, &cfg->od_60a9);
  (void)read_u32(ctx, 0x608F, 0x01, &cfg->od_608f_num);
  (void)read_u32(ctx, 0x608F, 0x02, &cfg->od_608f_den);
  (void)read_u32(ctx, 0x6091, 0x01, &cfg->od_6091_num);
  (void)read_u32(ctx, 0x6091, 0x02, &cfg->od_6091_den);
  (void)read_u32(ctx, 0x6092, 0x01, &cfg->od_6092_num);
  (void)read_u32(ctx, 0x6092, 0x02, &cfg->od_6092_den);
  (void)read_u32(ctx, 0x607F, 0x00, &cfg->max_profile_vel_raw);
  (void)read_u32(ctx, 0x6080, 0x00, &cfg->max_motor_speed_raw);
  (void)read_u32(ctx, 0x6081, 0x00, &cfg->profile_vel_current_raw);
  (void)read_u32(ctx, 0x6083, 0x00, &cfg->profile_accel_raw);
  (void)read_u32(ctx, 0x6084, 0x00, &cfg->profile_decel_raw);
  (void)read_u32(ctx, 0x6085, 0x00, &cfg->quickstop_decel_raw);
  (void)read_u32(ctx, 0x6086, 0x00, &cfg->motion_profile_type_raw);
  (void)read_u32(ctx, 0x60C5, 0x00, &cfg->max_accel_raw);
  (void)read_u32(ctx, 0x60C6, 0x00, &cfg->max_decel_raw);

  if (read_i32(ctx, 0x607D, 0x01, &tmp_i32) == 0) cfg->sw_pos_min_raw = tmp_i32;
  if (read_i32(ctx, 0x607D, 0x02, &tmp_i32) == 0) cfg->sw_pos_max_raw = tmp_i32;

  if (cfg->drive_units_per_motor_rev_override > 0.0) {
    cfg->drive_units_per_motor_rev = cfg->drive_units_per_motor_rev_override;
  } else if (cfg->od_608f_num != 0u && cfg->od_608f_den != 0u) {
    cfg->drive_units_per_motor_rev =
      ((double)cfg->od_608f_num) / ((double)cfg->od_608f_den);
  } else {
    cfg->drive_units_per_motor_rev = cfg->legacy_encoder_cpr_hint;
  }

  cfg->total_ratio = cfg->gear_ratio * cfg->extra_mech_ratio;
  cfg->counts_per_deg = (cfg->drive_units_per_motor_rev * cfg->total_ratio) / 360.0;
  cfg->deg_per_count = 1.0 / cfg->counts_per_deg;
}

static void log_scaling(pd4c_ctx_t* ctx, const demo_cfg_t* cfg) {
  int32_t pos = 0;
  int32_t home_off = 0;
  (void)read_i32(ctx, 0x6064, 0x00, &pos);
  (void)read_i32(ctx, 0x607C, 0x00, &home_off);

  printf("[scale] 60A8=0x%08" PRIX32 " 60A9=0x%08" PRIX32
         " 608F=%" PRIu32 "/%" PRIu32 " 6091=%" PRIu32 "/%" PRIu32
         " 6092=%" PRIu32 "/%" PRIu32 " 607C=%" PRId32
         " 607D=[%" PRId32 ", %" PRId32 "]"
         " 607F=%" PRIu32 " 6080=%" PRIu32 " 6081=%" PRIu32
         " 6083=%" PRIu32 " 6084=%" PRIu32 " 6085=%" PRIu32 " 6086=%" PRIu32
         " 60C5=%" PRIu32 " 60C6=%" PRIu32
         " current6064=%" PRId32 " home_raw=%" PRId32 "
",
         cfg->od_60a8, cfg->od_60a9,
         cfg->od_608f_num, cfg->od_608f_den,
         cfg->od_6091_num, cfg->od_6091_den,
         cfg->od_6092_num, cfg->od_6092_den,
         home_off,
         cfg->sw_pos_min_raw, cfg->sw_pos_max_raw,
         cfg->max_profile_vel_raw,
         cfg->max_motor_speed_raw,
         cfg->profile_vel_current_raw,
         cfg->profile_accel_raw,
         cfg->profile_decel_raw,
         cfg->quickstop_decel_raw,
         cfg->motion_profile_type_raw,
         cfg->max_accel_raw,
         cfg->max_decel_raw,
         pos, cfg->home_raw);
  fflush(stdout);

  printf("[kin] legacy_cpr_hint=%.0f active_drive_units_per_motor_rev=%.3f"
         " gear_ratio=%.3f extra_mech_ratio=%.3f total_ratio=%.3f"
         " counts_per_deg=%.6f deg_per_count=%.9f
",
         cfg->legacy_encoder_cpr_hint,
         cfg->drive_units_per_motor_rev,
         cfg->gear_ratio,
         cfg->extra_mech_ratio,
         cfg->total_ratio,
         cfg->counts_per_deg,
         cfg->deg_per_count);
  fflush(stdout);

  int32_t raw_step = deg_to_raw(cfg, DEMO_STEP_DEG) - deg_to_raw(cfg, 0.0);
  int32_t raw_lim = deg_to_raw(cfg, DEMO_MAX_DEG) - cfg->home_raw;
  double mast_speed_cap = profile_vel_rpm_raw_to_mast_deg_s(
      cfg, (int32_t)cfg->max_motor_speed_raw);
  uint32_t want_pv = mast_deg_s_to_profile_vel_rpm_raw(cfg, DEMO_MAX_LOAD_SPEED_DEG_S);
  double want_uncapped = (DEMO_MAX_LOAD_SPEED_DEG_S * cfg->total_ratio) / 6.0;

  printf("[kin] demo_step=%.3f deg -> raw_step=%" PRId32
         " | ±140deg -> raw_limit≈±%" PRId32
         " | requested 25deg/s -> motor_rpm_needed=%.3f capped_profile_vel_raw=%" PRIu32
         " | 6080 cap -> mast_speed_max≈%.3f deg/s
",
         DEMO_STEP_DEG, raw_step, raw_lim, want_uncapped, want_pv, mast_speed_cap);
  fflush(stdout);

  if (cfg->od_608f_num == 4096u && llround(cfg->legacy_encoder_cpr_hint) == 1024ll) {
    printf("[note] Datasheet encoder CPR (1024) is NOT the active 6064/607A scaling here."
           " The drive currently exposes 608F=%" PRIu32 "/%" PRIu32 ", so raw position units are based on %.0f per motor rev.
",
           cfg->od_608f_num, cfg->od_608f_den, cfg->drive_units_per_motor_rev);
    fflush(stdout);
  }

  if (cfg->od_6091_num != 100u || cfg->od_6091_den != 1u) {
    printf("[warn] Drive OD gear ratio 6091 is %" PRIu32 "/%" PRIu32 ", not 100/1."
           " The demo therefore applies gearbox + extra mast kinematics in software.
",
           cfg->od_6091_num, cfg->od_6091_den);
    fflush(stdout);
  }

  uint32_t active_speed_limit = 0;
  if (cfg->max_profile_vel_raw == 0) active_speed_limit = cfg->max_motor_speed_raw;
  else if (cfg->max_motor_speed_raw == 0) active_speed_limit = cfg->max_profile_vel_raw;
  else active_speed_limit = (cfg->max_profile_vel_raw < cfg->max_motor_speed_raw)
                              ? cfg->max_profile_vel_raw : cfg->max_motor_speed_raw;

  if (active_speed_limit > 0 && want_uncapped > (double)active_speed_limit) {
    printf("[warn] Requested mast speed %.1f deg/s needs about %.1f motor rpm,"
           " but active speed limit is %" PRIu32 " (min of 607F=%" PRIu32 ", 6080=%" PRIu32 ")."
           " The demo will be capped.
",
           DEMO_MAX_LOAD_SPEED_DEG_S, want_uncapped,
           active_speed_limit, cfg->max_profile_vel_raw, cfg->max_motor_speed_raw);
    fflush(stdout);
  }
}

static int maybe_widen_soft_limits(pd4c_ctx_t* ctx, demo_cfg_t* cfg) {
  /* Demo guardrail: ensure 607D software limits are large enough for calibrated
   * ±140 deg excursions after applying corrected counts/degree scaling.
   */
#if AUTO_WRITE_SOFT_LIMITS
  int32_t need_abs = deg_to_raw(cfg, DEMO_MAX_DEG) - cfg->home_raw;
  if (need_abs < 0) need_abs = -need_abs;
  int32_t desired_abs = need_abs + SOFT_LIMIT_MARGIN_COUNTS;
  if (desired_abs < SOFT_LIMIT_MIN_ABS_COUNTS) desired_abs = SOFT_LIMIT_MIN_ABS_COUNTS;

  bool too_small = (-desired_abs < cfg->sw_pos_min_raw) || (desired_abs > cfg->sw_pos_max_raw);
  if (!too_small) {
    printf("[limits] Existing 607D already large enough for corrected motion.\n");
    fflush(stdout);
    return 0;
  }

  printf("[limits] Writing 607D software limits from [%" PRId32 ", %" PRId32 "] to [%" PRId32 ", %" PRId32 "]\n",
         cfg->sw_pos_min_raw, cfg->sw_pos_max_raw, -desired_abs, desired_abs);
  fflush(stdout);

  int rc = pd4c_sdo_write_i32(ctx, 0x607D, 0x01, -desired_abs);
  if (rc != 0) return rc;
  rc = pd4c_sdo_write_i32(ctx, 0x607D, 0x02, desired_abs);
  if (rc != 0) return rc;

  msleep(50);
  (void)read_i32(ctx, 0x607D, 0x01, &cfg->sw_pos_min_raw);
  (void)read_i32(ctx, 0x607D, 0x02, &cfg->sw_pos_max_raw);

  printf("[limits] 607D after write = [%" PRId32 ", %" PRId32 "]\n",
         cfg->sw_pos_min_raw, cfg->sw_pos_max_raw);
  fflush(stdout);
#else
  (void)ctx;
  (void)cfg;
#endif
  return 0;
}

static int maybe_write_motion_limits(pd4c_ctx_t* ctx, demo_cfg_t* cfg) {
  /* Demo startup writes are explicit and bounded SDO diagnostics/setup ops,
   * kept separate from real-time PDO motion commands.
   */
#if AUTO_WRITE_MOTION_LIMITS
  int rc = 0;

  printf("[motion] Writing startup motion limits:
");
  printf("         607F=%u 6080=%u 6083=%u 6084=%u 6085=%u 6086=%u 60C5=%u 60C6=%u
",
         STARTUP_MAX_PROFILE_VEL_RAW,
         STARTUP_MAX_MOTOR_SPEED_RAW,
         STARTUP_PROFILE_ACCEL_RAW,
         STARTUP_PROFILE_DECEL_RAW,
         STARTUP_QUICKSTOP_DECEL_RAW,
         STARTUP_MOTION_PROFILE_TYPE,
         STARTUP_MAX_ACCEL_RAW,
         STARTUP_MAX_DECEL_RAW);
  fflush(stdout);

  /* Important: max accel/decel ceilings first, then profile accel/decel. */
  rc = pd4c_sdo_write_u32(ctx, 0x60C5, 0x00, STARTUP_MAX_ACCEL_RAW);
  if (rc != 0) return rc;
  rc = pd4c_sdo_write_u32(ctx, 0x60C6, 0x00, STARTUP_MAX_DECEL_RAW);
  if (rc != 0) return rc;

  rc = pd4c_sdo_write_u32(ctx, 0x607F, 0x00, STARTUP_MAX_PROFILE_VEL_RAW);
  if (rc != 0) return rc;
  rc = pd4c_sdo_write_u32(ctx, 0x6080, 0x00, STARTUP_MAX_MOTOR_SPEED_RAW);
  if (rc != 0) return rc;

  rc = pd4c_sdo_write_u32(ctx, 0x6083, 0x00, STARTUP_PROFILE_ACCEL_RAW);
  if (rc != 0) return rc;
  rc = pd4c_sdo_write_u32(ctx, 0x6084, 0x00, STARTUP_PROFILE_DECEL_RAW);
  if (rc != 0) return rc;
  rc = pd4c_sdo_write_u32(ctx, 0x6085, 0x00, STARTUP_QUICKSTOP_DECEL_RAW);
  if (rc != 0) return rc;

  rc = pd4c_sdo_write_u32(ctx, 0x6086, 0x00, STARTUP_MOTION_PROFILE_TYPE);
  if (rc != 0) return rc;

  msleep(50);

  (void)read_u32(ctx, 0x607F, 0x00, &cfg->max_profile_vel_raw);
  (void)read_u32(ctx, 0x6080, 0x00, &cfg->max_motor_speed_raw);
  (void)read_u32(ctx, 0x6083, 0x00, &cfg->profile_accel_raw);
  (void)read_u32(ctx, 0x6084, 0x00, &cfg->profile_decel_raw);
  (void)read_u32(ctx, 0x6085, 0x00, &cfg->quickstop_decel_raw);
  (void)read_u32(ctx, 0x6086, 0x00, &cfg->motion_profile_type_raw);
  (void)read_u32(ctx, 0x60C5, 0x00, &cfg->max_accel_raw);
  (void)read_u32(ctx, 0x60C6, 0x00, &cfg->max_decel_raw);

  printf("[motion] After write: 607F=%" PRIu32 " 6080=%" PRIu32
         " 6083=%" PRIu32 " 6084=%" PRIu32 " 6085=%" PRIu32
         " 6086=%" PRIu32 " 60C5=%" PRIu32 " 60C6=%" PRIu32 "
",
         cfg->max_profile_vel_raw,
         cfg->max_motor_speed_raw,
         cfg->profile_accel_raw,
         cfg->profile_decel_raw,
         cfg->quickstop_decel_raw,
         cfg->motion_profile_type_raw,
         cfg->max_accel_raw,
         cfg->max_decel_raw);
  fflush(stdout);
#else
  (void)ctx;
  (void)cfg;
#endif
  return 0;
}

static int wait_state_sdo(pd4c_ctx_t* ctx, int desired_state, int timeout_ms, bool abortable) {
  uint64_t start = now_ms();
  while ((int)(now_ms() - start) < timeout_ms) {
    if (abortable && g_stop) return PD4C_ETIMEDOUT;
    uint16_t sw = 0;
    if (read_u16(ctx, 0x6041, 0x00, &sw) == 0) {
      int st = ds402_state(sw);
      if (st == desired_state) return 0;
      if (st == 7) return PD4C_ESTATE;
    }
    msleep(SDO_POLL_MS);
  }
  return PD4C_ETIMEDOUT;
}

static int ds402_enable_operation_sdo(pd4c_ctx_t* ctx, int8_t mode, bool abortable) {
  int rc;
  rc = pd4c_rpdo1_send_control_mode(ctx, 0x0006, mode);
  if (rc != 0) return rc;
  rc = wait_state_sdo(ctx, 2, ENABLE_TIMEOUT_MS, abortable);
  if (rc != 0) return rc;

  rc = pd4c_rpdo1_send_control_mode(ctx, 0x0007, mode);
  if (rc != 0) return rc;
  rc = wait_state_sdo(ctx, 3, ENABLE_TIMEOUT_MS, abortable);
  if (rc != 0) return rc;

  rc = pd4c_rpdo1_send_control_mode(ctx, 0x000F, mode);
  if (rc != 0) return rc;
  rc = wait_state_sdo(ctx, 4, ENABLE_TIMEOUT_MS, abortable);
  return rc;
}

static void log_telem(pd4c_ctx_t* ctx, const demo_cfg_t* cfg, const char* tag) {
  uint16_t sw = 0, err = 0;
  uint8_t mode = 0, err_reg = 0;
  int32_t pos = 0, vel_vl = 0, vel_act = 0, tgt = 0;
  uint32_t pv = 0;
  uint32_t din = 0, dout = 0;

  (void)read_u16(ctx, 0x6041, 0x00, &sw);
  (void)read_u8(ctx, 0x6061, 0x00, &mode);
  (void)read_i32(ctx, 0x6064, 0x00, &pos);
  (void)read_i32(ctx, 0x6044, 0x00, &vel_vl);
  (void)read_i32(ctx, 0x606C, 0x00, &vel_act);
  (void)read_i32(ctx, 0x607A, 0x00, &tgt);
  (void)read_u32(ctx, 0x6081, 0x00, &pv);
  (void)read_u16(ctx, 0x603F, 0x00, &err);
  (void)read_u8(ctx, 0x1001, 0x00, &err_reg);
  (void)read_u32(ctx, 0x60FD, 0x00, &din);
  (void)read_u32(ctx, 0x60FE, 0x01, &dout);

  double mast_vel_deg_s = profile_vel_rpm_raw_to_mast_deg_s(cfg, vel_act);
  double pv_deg_s = profile_vel_rpm_raw_to_mast_deg_s(cfg, (int32_t)pv);

  printf("[%s] sw=0x%04X (%s) mode=%u raw_pos=%" PRId32 " mast_deg=%.3f target_raw=%" PRId32
         " raw_vl=%" PRId32 " raw_606C=%" PRId32 " (~%.3f deg/s)"
         " pv=%" PRIu32 " (~%.3f deg/s)"
         " err=0x%04X err_reg=0x%02X din=0x%08" PRIX32 " dout=0x%08" PRIX32 "\n",
         tag, sw, ds402_state_str(ds402_state(sw)), (unsigned)mode,
         pos, raw_to_deg(cfg, pos), tgt,
         vel_vl, vel_act, mast_vel_deg_s,
         pv, pv_deg_s,
         err, (unsigned)err_reg, din, dout);
  fflush(stdout);
}

static int check_fault_or_emcy(pd4c_ctx_t* ctx, const demo_cfg_t* cfg) {
  uint16_t sw = 0, err = 0;
  uint8_t err_reg = 0;
  (void)read_u16(ctx, 0x6041, 0x00, &sw);
  (void)read_u16(ctx, 0x603F, 0x00, &err);
  (void)read_u8(ctx, 0x1001, 0x00, &err_reg);

  if (ds402_state(sw) == 7 || err != 0 || err_reg != 0) {
    if (err == 0x3100u) {
      fprintf(stderr, "[fault] Drive fault 0x3100 = supply over/undervoltage.\n");
    } else if (err == 0x8210u || err == 0x8220u) {
      fprintf(stderr, "[fault] Drive fault indicates PDO length/mapping mismatch.\n");
    }
    log_telem(ctx, cfg, "fault");
    return PD4C_ESTATE;
  }
  return 0;
}

static int rpdo2_send_pp_adaptive(pd4c_ctx_t* ctx, const demo_cfg_t* cfg,
                                  int32_t target_pos, uint32_t prof_vel_raw) {
  struct can_frame fr;
  memset(&fr, 0, sizeof(fr));
  fr.can_id = ctx->rpdo2_id;

  if (cfg->profile_vel_bits == 32) {
    fr.can_dlc = 8;
    fr.data[0] = (uint8_t)(target_pos & 0xFF);
    fr.data[1] = (uint8_t)((target_pos >> 8) & 0xFF);
    fr.data[2] = (uint8_t)((target_pos >> 16) & 0xFF);
    fr.data[3] = (uint8_t)((target_pos >> 24) & 0xFF);
    fr.data[4] = (uint8_t)(prof_vel_raw & 0xFF);
    fr.data[5] = (uint8_t)((prof_vel_raw >> 8) & 0xFF);
    fr.data[6] = (uint8_t)((prof_vel_raw >> 16) & 0xFF);
    fr.data[7] = (uint8_t)((prof_vel_raw >> 24) & 0xFF);
  } else {
    uint16_t pv16 = (prof_vel_raw > 0xFFFFu) ? 0xFFFFu : (uint16_t)prof_vel_raw;
    fr.can_dlc = 6;
    fr.data[0] = (uint8_t)(target_pos & 0xFF);
    fr.data[1] = (uint8_t)((target_pos >> 8) & 0xFF);
    fr.data[2] = (uint8_t)((target_pos >> 16) & 0xFF);
    fr.data[3] = (uint8_t)((target_pos >> 24) & 0xFF);
    fr.data[4] = (uint8_t)(pv16 & 0xFF);
    fr.data[5] = (uint8_t)((pv16 >> 8) & 0xFF);
  }

  ssize_t n = write(ctx->sock, &fr, sizeof(fr));
  if (n != (ssize_t)sizeof(fr)) return PD4C_EIO;
  return 0;
}

static int wait_target_reached(pd4c_ctx_t* ctx, const demo_cfg_t* cfg,
                               int32_t target_raw, int32_t tol_counts, int timeout_ms,
                               bool abortable) {
  uint64_t start = now_ms();
  uint64_t last_log = 0;

  while ((int)(now_ms() - start) < timeout_ms) {
    if (abortable && g_stop) return PD4C_ETIMEDOUT;
    if (check_fault_or_emcy(ctx, cfg) != 0) return PD4C_ESTATE;

    uint16_t sw = 0;
    int32_t pos = 0;
    (void)read_u16(ctx, 0x6041, 0x00, &sw);
    (void)read_i32(ctx, 0x6064, 0x00, &pos);

    bool trg = (sw & (1u << 10)) != 0u;
    int32_t e = pos - target_raw;
    if (e < 0) e = -e;

    uint64_t t = now_ms();
    if (t - last_log >= 500) {
      log_telem(ctx, cfg, "move");
      last_log = t;
    }

    if (trg && e <= tol_counts) return 0;
    msleep(SDO_POLL_MS);
  }
  return PD4C_ETIMEDOUT;
}

static int pp_move_abs_deg_impl(pd4c_ctx_t* ctx, const demo_cfg_t* cfg,
                                double deg, double speed_deg_s, bool abortable) {
  if (deg < DEMO_MIN_DEG) deg = DEMO_MIN_DEG;
  if (deg > DEMO_MAX_DEG) deg = DEMO_MAX_DEG;

  int rc = ds402_enable_operation_sdo(ctx, PD4C_MODE_PROFILE_POSITION, abortable);
  if (rc != 0) {
    fprintf(stderr, "Enable OP (Profile Position) failed: %s (%d)\n", pd4c_strerror(rc), rc);
    return rc;
  }

  int32_t target_raw = deg_to_raw(cfg, deg);
  uint32_t pv_raw = mast_deg_s_to_profile_vel_rpm_raw(cfg, speed_deg_s);

  fprintf(stdout,
          "[cmd] abs_deg=%.3f -> target_raw=%" PRId32 " profile_vel_raw=%" PRIu32
          " (~%.3f mast_deg/s requested %.3f)\n",
          deg, target_raw, pv_raw,
          profile_vel_rpm_raw_to_mast_deg_s(cfg, (int32_t)pv_raw), speed_deg_s);
  fflush(stdout);

  rc = rpdo2_send_pp_adaptive(ctx, cfg, target_raw, pv_raw);
  if (rc != 0) {
    fprintf(stderr, "RPDO2 send failed: %s (%d)\n", pd4c_strerror(rc), rc);
    return rc;
  }

  rc = pd4c_rpdo1_send_control_mode(ctx, 0x001F, PD4C_MODE_PROFILE_POSITION);
  if (rc != 0) return rc;
  msleep(20);
  rc = pd4c_rpdo1_send_control_mode(ctx, 0x000F, PD4C_MODE_PROFILE_POSITION);
  if (rc != 0) return rc;

  int32_t tol = (int32_t)llround(cfg->counts_per_deg * 0.05); /* ±0.05 deg */
  if (tol < 10) tol = 10;

  rc = wait_target_reached(ctx, cfg, target_raw, tol, MOVE_TIMEOUT_MS, abortable);
  if (rc != 0) {
    fprintf(stderr, "Move to %.3f deg timed out/faulted: %s (%d)\n", deg, pd4c_strerror(rc), rc);
    log_telem(ctx, cfg, "timeout");
    return rc;
  }

  log_telem(ctx, cfg, "reached");
  return 0;
}

static int pp_move_abs_deg(pd4c_ctx_t* ctx, const demo_cfg_t* cfg, double deg, double speed_deg_s) {
  return pp_move_abs_deg_impl(ctx, cfg, deg, speed_deg_s, true);
}

static int pp_move_abs_deg_force(pd4c_ctx_t* ctx, const demo_cfg_t* cfg, double deg, double speed_deg_s) {
  return pp_move_abs_deg_impl(ctx, cfg, deg, speed_deg_s, false);
}

static void safe_stop_and_home(pd4c_ctx_t* ctx, const demo_cfg_t* cfg) {
  /* Safety behavior for demo/bench use: always issue quick-stop first, then
   * drive back to calibrated HOME with abort flag temporarily masked.
   */
  sig_atomic_t stop_saved = g_stop;
  g_stop = 0; /* allow the safety return-home move to complete */

  printf("[safety] Quick stop + HOME(raw=%" PRId32 ")\n", cfg->home_raw);
  fflush(stdout);
  (void)pd4c_ds402_quick_stop(ctx, PD4C_MODE_PROFILE_POSITION);
  msleep(200);
  (void)pp_move_abs_deg_force(ctx, cfg, DEMO_HOME_DEG, DEMO_MAX_LOAD_SPEED_DEG_S);
  sleep_s(1.0);

  g_stop = stop_saved;
}

static void on_signal(int signo) {
  (void)signo;
  /* Async-signal-safe stop flag consumed by main motion loop. */
  g_stop = 1;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr,
      "Usage: %s <can_iface> <node_id>"
      " [legacy_encoder_cpr_hint] [gear_ratio] [home_raw]"
      " [extra_mech_ratio] [drive_units_per_motor_rev_override]\n",
      argv[0]);
    return 2;
  }

  const char* ifname = argv[1];
  int node = atoi(argv[2]);
  if (node <= 0 || node > 127) {
    fprintf(stderr, "Invalid node id: %d\n", node);
    return 2;
  }

  demo_cfg_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.legacy_encoder_cpr_hint = (argc >= 4) ? atof(argv[3]) : DEFAULT_LEGACY_CPR_HINT;
  cfg.gear_ratio = (argc >= 5) ? atof(argv[4]) : DEFAULT_GEAR_RATIO;
  cfg.home_raw = (argc >= 6) ? (int32_t)strtol(argv[5], NULL, 0) : DEFAULT_HOME_RAW;
  cfg.extra_mech_ratio = (argc >= 7) ? atof(argv[6]) : DEFAULT_EXTRA_MECH_RATIO;
  cfg.drive_units_per_motor_rev_override = (argc >= 8) ? atof(argv[7]) : DEFAULT_DRIVE_UNITS_PER_MOTOR_REV_OVERRIDE;
  cfg.invert_dir = DEFAULT_INVERT_DIR;

  if (cfg.legacy_encoder_cpr_hint <= 0.0 || cfg.gear_ratio <= 0.0 || cfg.extra_mech_ratio <= 0.0) {
    fprintf(stderr, "legacy_encoder_cpr_hint, gear_ratio and extra_mech_ratio must be > 0\n");
    return 2;
  }

  /* SIGINT/SIGTERM convert into graceful stop path that still executes
   * safe_stop_and_home before close when possible.
   */
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  pd4c_ctx_t ctx;
  memset(&ctx, 0, sizeof(ctx));

  int rc = pd4c_open(&ctx, ifname, (uint8_t)node);
  if (rc != 0) {
    fprintf(stderr, "pd4c_open failed: %s (%d)\n", pd4c_strerror(rc), rc);
    return 1;
  }

  pd4c_set_sdo_timeout(&ctx, 500);

  rc = pd4c_nmt_start(&ctx);
  if (rc != 0) {
    fprintf(stderr, "pd4c_nmt_start failed: %s (%d)\n", pd4c_strerror(rc), rc);
    pd4c_close(&ctx);
    return 1;
  }

  stop_nanoj_if_running(&ctx);
  maybe_write_6091_gear_ratio(&ctx, cfg.gear_ratio);

  cfg.profile_vel_bits = detect_profile_vel_bits(&ctx);

  read_scaling_objects(&ctx, &cfg);

  printf("[init] iface=%s node=%d legacy_cpr_hint=%.0f gear=%.3f extra=%.3f total=%.3f"
         " drive_units_per_motor_rev=%.3f home_raw=%" PRId32
         " rpdo2_pv_bits=%d 6080(max_speed_raw)=%" PRIu32 "\n",
         ifname, node,
         cfg.legacy_encoder_cpr_hint,
         cfg.gear_ratio, cfg.extra_mech_ratio, cfg.total_ratio,
         cfg.drive_units_per_motor_rev,
         cfg.home_raw,
         cfg.profile_vel_bits, cfg.max_motor_speed_raw);
  fflush(stdout);

  log_scaling(&ctx, &cfg);

  rc = maybe_widen_soft_limits(&ctx, &cfg);
  if (rc != 0) {
    fprintf(stderr, "Failed to update 607D: %s (%d)\n", pd4c_strerror(rc), rc);
    pd4c_close(&ctx);
    return 1;
  }

  rc = maybe_raise_max_motor_speed(&ctx, &cfg);
  if (rc != 0) {
    fprintf(stderr, "Failed to update 6080: %s (%d)\n", pd4c_strerror(rc), rc);
    pd4c_close(&ctx);
    return 1;
  }

  /* re-read after any writes */
  read_scaling_objects(&ctx, &cfg);
  log_telem(&ctx, &cfg, "init");

  safe_stop_and_home(&ctx, &cfg);
  if (g_stop) goto out;

  printf("[demo] Hold HOME for %ds\n", DEMO_HOLD_HOME_S);
  fflush(stdout);
  sleep_s((double)DEMO_HOLD_HOME_S);

  printf("[demo] Move to MIN %.1f deg @ %.1f deg/s\n", DEMO_MIN_DEG, DEMO_MAX_LOAD_SPEED_DEG_S);
  fflush(stdout);
  rc = pp_move_abs_deg(&ctx, &cfg, DEMO_MIN_DEG, DEMO_MAX_LOAD_SPEED_DEG_S);
  if (rc != 0) goto fail;
  printf("[demo] Hold MIN for %ds\n", DEMO_HOLD_LIMIT_S);
  fflush(stdout);
  sleep_s((double)DEMO_HOLD_LIMIT_S);
  if (g_stop) goto out;

  printf("[demo] Move to MAX %.1f deg @ %.1f deg/s\n", DEMO_MAX_DEG, DEMO_MAX_LOAD_SPEED_DEG_S);
  fflush(stdout);
  rc = pp_move_abs_deg(&ctx, &cfg, DEMO_MAX_DEG, DEMO_MAX_LOAD_SPEED_DEG_S);
  if (rc != 0) goto fail;
  printf("[demo] Hold MAX for %ds\n", DEMO_HOLD_LIMIT_S);
  fflush(stdout);
  sleep_s((double)DEMO_HOLD_LIMIT_S);
  if (g_stop) goto out;

  printf("[demo] Return HOME @ %.1f deg/s\n", DEMO_MAX_LOAD_SPEED_DEG_S);
  fflush(stdout);
  rc = pp_move_abs_deg(&ctx, &cfg, DEMO_HOME_DEG, DEMO_MAX_LOAD_SPEED_DEG_S);
  if (rc != 0) goto fail;
  if (g_stop) goto out;

  printf("[demo] Granularity demo: +0.1 deg steps every %ds until +140 deg\n", DEMO_STEP_INTERVAL_S);
  fflush(stdout);
  int steps = (int)llround((DEMO_MAX_DEG - DEMO_HOME_DEG) / DEMO_STEP_DEG);
  for (int i = 0; i <= steps; ++i) {
    if (g_stop) break;
    double pos_deg = DEMO_HOME_DEG + (double)i * DEMO_STEP_DEG;
    rc = pp_move_abs_deg(&ctx, &cfg, pos_deg, DEMO_MAX_LOAD_SPEED_DEG_S);
    if (rc != 0) goto fail;
    sleep_s((double)DEMO_STEP_INTERVAL_S);
  }

  if (!g_stop) {
    printf("[demo] Step loop complete; return HOME and exit\n");
    fflush(stdout);
    (void)pp_move_abs_deg(&ctx, &cfg, DEMO_HOME_DEG, DEMO_MAX_LOAD_SPEED_DEG_S);
  }

out:
  if (g_stop) {
    safe_stop_and_home(&ctx, &cfg);
  }
  pd4c_close(&ctx);
  return 0;

fail:
  fprintf(stderr, "[demo] Failure path; safety stop + home\n");
  safe_stop_and_home(&ctx, &cfg);
  pd4c_close(&ctx);
  return 1;
}
