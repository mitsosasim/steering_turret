/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Command clamping and saturation accounting for axis motion requests.
 *
 * Design notes:
 *   Safety-sensitive pre-execution limits enforce configured mechanical bounds.
 *   Position and speed are clamped independently so diagnostics can report cause.
 *   Helpers preserve requested vs applied motion deltas for operator traceability.
 */

#include "stc_limits.h"

#include <math.h>

/* Saturation policy: keep runtime commands within configured mechanical limits
 * and report when clipping happened so operators/diagnostics can distinguish
 * requested motion from applied motion.
 */
double stc_clamp(double val, double min, double max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

/** Clamp target position and speed independently; return true if any clamp applied. */
bool stc_clamp_axis_cmd(const axis_cfg_t* cfg,
                        double* target_deg,
                        double* speed_deg_s,
                        bool* target_limited,
                        bool* speed_limited) {
  /* Clamp target/speed independently and report exactly what was clipped. */
  bool limited = false;
  if (!cfg) return false;

  if (target_limited) *target_limited = false;
  if (speed_limited) *speed_limited = false;

  /* Target clamping protects position commands against travel-limit overrun. */
  if (target_deg) {
    const double clamped = stc_clamp(*target_deg, cfg->az_min_deg, cfg->az_max_deg);
    if (fabs(clamped - *target_deg) > 1e-9) {
      *target_deg = clamped;
      limited = true;
      if (target_limited) *target_limited = true;
    }
  }

  /* Speed clamping enforces configured max axis rate in both directions. */
  if (speed_deg_s) {
    const double max_speed = fabs(cfg->max_speed_deg_s);
    const double clamped = stc_clamp(*speed_deg_s, -max_speed, max_speed);
    if (fabs(clamped - *speed_deg_s) > 1e-9) {
      *speed_deg_s = clamped;
      limited = true;
      if (speed_limited) *speed_limited = true;
    }
  }

  return limited;
}


bool stc_compute_effective_delta(double current_deg,
                                double requested_delta_deg,
                                double min_deg,
                                double max_deg,
                                double* requested_abs_deg,
                                double* clamped_abs_deg,
                                double* effective_delta_deg) {
  /* Preserve both requested and applied movement for diagnostics/logging. */
  const double requested_abs = current_deg + requested_delta_deg;
  const double clamped_abs = stc_clamp(requested_abs, min_deg, max_deg);
  const double effective_delta = clamped_abs - current_deg;

  if (requested_abs_deg) *requested_abs_deg = requested_abs;
  if (clamped_abs_deg) *clamped_abs_deg = clamped_abs;
  if (effective_delta_deg) *effective_delta_deg = effective_delta;

  return fabs(clamped_abs - requested_abs) > 1e-9;
}
