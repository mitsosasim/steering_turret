/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Axis command clamping and saturation helper API for command safety checks.
 *
 * Design notes:
 *   Enforces configured position/speed limits and reports when requests were
 *   saturated so diagnostics/telemetry can expose operator-command limiting.
 */

#ifndef STC_LIMITS_H
#define STC_LIMITS_H

#include "stc_model.h"

#include <stdbool.h>

/** Clamp scalar value to inclusive [min,max] bounds. */
double stc_clamp(double val, double min, double max);

bool stc_clamp_axis_cmd(const axis_cfg_t* cfg,
                        double* target_deg,
                        double* speed_deg_s,
                        bool* target_limited,
                        bool* speed_limited);

bool stc_compute_effective_delta(double current_deg,
                                double requested_delta_deg,
                                double min_deg,
                                double max_deg,
                                double* requested_abs_deg,
                                double* clamped_abs_deg,
                                double* effective_delta_deg);

#endif
