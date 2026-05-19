/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Internal module implementation for STC runtime support services.
 *
 * Design notes:
 *   Comment-only documentation refactor; runtime behavior is unchanged.
 */

#include "stc_model.h"

#include <limits.h>
#include <math.h>

/* Centralized conversion helpers keep all modules aligned on mechanical scaling
 * (counts_per_rev, gear_ratio, vel_scale) so command generation, telemetry, and
 * diagnostics use identical math and avoid drift across duplicated formulas.
 */
static double safe_cpr(const axis_cfg_t* cfg) {
  return (cfg && cfg->counts_per_rev > 0.0) ? cfg->counts_per_rev : 1.0;
}

static double safe_gr(const axis_cfg_t* cfg) {
  return (cfg && cfg->gear_ratio > 0.0) ? cfg->gear_ratio : 1.0;
}

static double safe_vel_scale(const axis_cfg_t* cfg) {
  return (cfg && cfg->vel_scale != 0.0) ? cfg->vel_scale : 1.0;
}

const char* axis_driver_to_string(stc_axis_driver_t driver) {
  switch (driver) {
    case STC_AXIS_DRIVER_DISABLED: return "disabled";
    case STC_AXIS_DRIVER_PD4C: return "pd4c";
    case STC_AXIS_DRIVER_MYACT_X8_25: return "myact_x8_25";
    case STC_AXIS_DRIVER_SIM: return "sim";
    default: return "unknown";
  }
}

/** Convert encoder counts (motor-domain units) into physical axis degrees. */
double axis_counts_to_deg(const axis_cfg_t* cfg, int32_t counts) {
  const double cpr = safe_cpr(cfg);
  const double gr = safe_gr(cfg);
  return ((double)counts) * 360.0 / (cpr * gr);
}

/** Convert physical axis degrees into encoder counts for command objects. */
int32_t axis_deg_to_counts(const axis_cfg_t* cfg, double deg) {
  const double cpr = safe_cpr(cfg);
  const double gr = safe_gr(cfg);
  double counts = deg * (cpr * gr) / 360.0;
  if (counts > (double)INT32_MAX) counts = (double)INT32_MAX;
  if (counts < (double)INT32_MIN) counts = (double)INT32_MIN;
  return (int32_t)llround(counts);
}

/** Convert backend raw velocity units into physical axis deg/s telemetry. */
double axis_vel_to_deg_s(const axis_cfg_t* cfg, int32_t vel_raw) {
  const double gr = safe_gr(cfg);
  const double vel_scale = safe_vel_scale(cfg);
  const double motor_deg_s = ((double)vel_raw) * vel_scale;
  return motor_deg_s / gr;
}

/** Convert commanded axis deg/s into backend target velocity units with clipping. */
int32_t axis_deg_s_to_target_vel_units(const axis_cfg_t* cfg, double axis_deg_s, int* clipped) {
  if (clipped) *clipped = 0;

  double cmd_deg_s = axis_deg_s;
  const double max_speed = (cfg && cfg->max_speed_deg_s > 0.0) ? cfg->max_speed_deg_s : 0.0;
  if (max_speed > 0.0) {
    if (cmd_deg_s > max_speed) {
      cmd_deg_s = max_speed;
      if (clipped) *clipped = 1;
    }
    if (cmd_deg_s < -max_speed) {
      cmd_deg_s = -max_speed;
      if (clipped) *clipped = 1;
    }
  }

  const double motor_deg_s = cmd_deg_s * safe_gr(cfg);
  const double vel_scale = safe_vel_scale(cfg);
  double raw = motor_deg_s / vel_scale;

  if (raw > (double)INT32_MAX) {
    raw = (double)INT32_MAX;
    if (clipped) *clipped = 1;
  }
  if (raw < (double)INT32_MIN) {
    raw = (double)INT32_MIN;
    if (clipped) *clipped = 1;
  }

  return (int32_t)llround(raw);
}

/** Update one axis telemetry sample and derive acceleration from bounded dt. */
void axis_telem_update(axis_telem_t* telem,
                       const axis_cfg_t* cfg,
                       int32_t pos_raw,
                       int32_t vel_raw,
                       uint16_t status_word,
                       int8_t mode_disp,
                       uint64_t ts_ms) {
  if (!telem) return;

  telem->pos_raw = pos_raw;
  telem->vel_raw = vel_raw;
  telem->status_word = status_word;
  telem->mode_disp = mode_disp;

  const double vel_now = axis_vel_to_deg_s(cfg, vel_raw);
  telem->pos_deg = axis_counts_to_deg(cfg, pos_raw);
  telem->vel_deg_s = vel_now;

  double accel = 0.0;
  if (telem->prev_ts_ms > 0 && ts_ms > telem->prev_ts_ms) {
    const double dt = ((double)(ts_ms - telem->prev_ts_ms)) / 1000.0;
    if (dt > 0.0 && dt <= 1.0) {
      accel = (vel_now - telem->prev_vel_deg_s) / dt;
    }
  }
  telem->accel_deg_s2 = accel;

  telem->prev_vel_deg_s = vel_now;
  telem->prev_ts_ms = ts_ms;
  telem->ts_ms = ts_ms;
}
