/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Axis configuration mapping regression tests from INI model into runtime axis_cfg_t.
 *
 * Design notes:
 *   Prevents mismapped axis ownership, driver kind, and limits from silently changing at startup.
 */

#include "stc_axis_cfg_map.h"

#include <assert.h>
#include <string.h>

/* Verifies one-to-one config->runtime mapping so enabled axis ownership and limits remain unchanged across parser/runtime refactors. */
int main(void) {
  stc_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  const int axis = 3;

  cfg.axis_enabled[axis] = 1;
  cfg.axis_driver[axis] = STC_AXIS_DRIVER_MYACT_X8_25;
  cfg.axis_node_id[axis] = 11;
  cfg.axis_command_sign[axis] = -1.0;
  cfg.axis_telemetry_sign[axis] = 1.0;
  cfg.axis_zero_physical_deg[axis] = 12.5;
  cfg.axis_zero_raw_deg[axis] = -4.5;
  cfg.axis_zero_on_start[axis] = 1;
  cfg.axis_counts_per_rev[axis] = 4096.0;
  cfg.axis_gear_ratio[axis] = 123.0;
  cfg.axis_vel_scale[axis] = 0.125;
  cfg.axis_vel_raw_mode[axis] = 2;
  cfg.axis_min_deg[axis] = -85.0;
  cfg.axis_max_deg[axis] = 95.0;
  cfg.axis_max_speed_deg_s[axis] = 77.0;
  cfg.axis_storage_deg[axis] = 16.0;
  cfg.axis_brake_release_timeout_ms[axis] = 4321;
  cfg.axis_brake_poll_ms[axis] = 65;
  cfg.axis_active_status2_enable[axis] = 1;
  cfg.axis_active_status2_interval_ms[axis] = 70;
  cfg.axis_feedback_timeout_ms[axis] = 888;
  cfg.axis_pp_profile_vel[axis] = 321;

  axis_cfg_t out;
  memset(&out, 0, sizeof(out));
  uint8_t expected = 0;

  stc_axis_cfg_from_config(&cfg, axis, &out, &expected);

  assert(out.driver == cfg.axis_driver[axis]);
  assert(out.node_id == cfg.axis_node_id[axis]);
  assert(out.command_sign == cfg.axis_command_sign[axis]);
  assert(out.telemetry_sign == cfg.axis_telemetry_sign[axis]);
  assert(out.zero_physical_deg == cfg.axis_zero_physical_deg[axis]);
  assert(out.zero_raw_deg == cfg.axis_zero_raw_deg[axis]);
  assert(out.zero_on_start == cfg.axis_zero_on_start[axis]);
  assert(out.counts_per_rev == cfg.axis_counts_per_rev[axis]);
  assert(out.gear_ratio == cfg.axis_gear_ratio[axis]);
  assert(out.vel_scale == cfg.axis_vel_scale[axis]);
  assert(out.vel_raw_units == AXIS_VEL_RAW_DEG_PER_SEC);
  assert(out.vel_raw_mode == cfg.axis_vel_raw_mode[axis]);
  assert(out.min_deg == cfg.axis_min_deg[axis]);
  assert(out.max_deg == cfg.axis_max_deg[axis]);
  assert(out.az_min_deg == cfg.axis_min_deg[axis]);
  assert(out.az_max_deg == cfg.axis_max_deg[axis]);
  assert(out.max_speed_deg_s == cfg.axis_max_speed_deg_s[axis]);
  assert(out.storage_deg == cfg.axis_storage_deg[axis]);
  assert(out.brake_release_timeout_ms == cfg.axis_brake_release_timeout_ms[axis]);
  assert(out.brake_poll_ms == cfg.axis_brake_poll_ms[axis]);
  assert(out.active_status2_enable == cfg.axis_active_status2_enable[axis]);
  assert(out.active_status2_interval_ms == cfg.axis_active_status2_interval_ms[axis]);
  assert(out.feedback_timeout_ms == cfg.axis_feedback_timeout_ms[axis]);
  assert(out.pp_profile_vel == cfg.axis_pp_profile_vel[axis]);
  assert(expected == 1u);

  cfg.axis_enabled[axis] = 0;
  stc_axis_cfg_from_config(&cfg, axis, &out, &expected);
  assert(expected == 0u);

  cfg.axis_enabled[axis] = 1;
  cfg.axis_node_id[axis] = 0;
  stc_axis_cfg_from_config(&cfg, axis, &out, &expected);
  assert(expected == 0u);

  cfg.axis_enabled[axis] = 1;
  cfg.axis_driver[axis] = STC_AXIS_DRIVER_DISABLED;
  cfg.axis_node_id[axis] = 11;
  stc_axis_cfg_from_config(&cfg, axis, &out, &expected);
  assert(expected == 0u);

  stc_config_t empty_cfg;
  memset(&empty_cfg, 0, sizeof(empty_cfg));
  uint8_t empty_expected = 1u;
  stc_axis_cfg_from_config(&empty_cfg, 0, &out, &empty_expected);
  assert(empty_expected == 0u);

  return 0;
}
