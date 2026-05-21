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

#include "stc_axis_cfg_map.h"

void stc_axis_cfg_from_config(const stc_config_t* cfg, int axis, axis_cfg_t* out_cfg, uint8_t* out_axis_expected) {
  if (!cfg || !out_cfg || axis < 0 || axis >= (int)STC_AXIS_COUNT) return;

  out_cfg->driver = cfg->axis_driver[axis];
  out_cfg->node_id = cfg->axis_node_id[axis];

  out_cfg->min_deg = cfg->axis_min_deg[axis];
  out_cfg->max_deg = cfg->axis_max_deg[axis];
  out_cfg->max_speed_deg_s = cfg->axis_max_speed_deg_s[axis];
  out_cfg->storage_deg = cfg->axis_storage_deg[axis];

  out_cfg->command_sign = cfg->axis_command_sign[axis];
  out_cfg->telemetry_sign = cfg->axis_telemetry_sign[axis];
  out_cfg->zero_physical_deg = cfg->axis_zero_physical_deg[axis];
  out_cfg->zero_raw_deg = cfg->axis_zero_raw_deg[axis];
  out_cfg->zero_on_start = cfg->axis_zero_on_start[axis];

  out_cfg->brake_release_timeout_ms = cfg->axis_brake_release_timeout_ms[axis];
  out_cfg->brake_poll_ms = cfg->axis_brake_poll_ms[axis];
  out_cfg->active_status2_enable = cfg->axis_active_status2_enable[axis];
  out_cfg->active_status2_interval_ms = cfg->axis_active_status2_interval_ms[axis];
  out_cfg->feedback_timeout_ms = cfg->axis_feedback_timeout_ms[axis];

  out_cfg->counts_per_rev = cfg->axis_counts_per_rev[axis];
  out_cfg->gear_ratio = cfg->axis_gear_ratio[axis];
  out_cfg->vel_raw_units = AXIS_VEL_RAW_DEG_PER_SEC;
  out_cfg->vel_scale = cfg->axis_vel_scale[axis];
  out_cfg->az_min_deg = cfg->axis_min_deg[axis];
  out_cfg->az_max_deg = cfg->axis_max_deg[axis];
  out_cfg->vel_raw_mode = cfg->axis_vel_raw_mode[axis];
  out_cfg->pp_profile_vel = cfg->axis_pp_profile_vel[axis];

  if (out_axis_expected) {
    const int enabled = cfg->axis_enabled[axis] != 0;
    const int driver_active = cfg->axis_driver[axis] != STC_AXIS_DRIVER_DISABLED;
    const int node_mapped = cfg->axis_node_id[axis] > 0;
    *out_axis_expected = (enabled && driver_active && node_mapped) ? 1u : 0u;
  }
}
