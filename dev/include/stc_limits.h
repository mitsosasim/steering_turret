#ifndef STC_LIMITS_H
#define STC_LIMITS_H

#include "stc_model.h"

#include <stdbool.h>

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
