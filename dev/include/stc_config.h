/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Defines the INI configuration model for STC runtime and axis mappings.
 *
 * Design notes:
 *   Covers network/CAN settings, axis mapping, and turret/SUT/mirror/laser maps.
 *   Configuration is loaded once at startup and copied into runtime structures.
 */

#ifndef STC_CONFIG_H
#define STC_CONFIG_H

#include "stc_model.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STC_MAX_SUT 8
#define STC_MAX_MIRROR 8

/** Parsed configuration container populated from INI and consumed at startup. */
typedef struct stc_config_s {
  /** TCP command socket bind port for CMD/RSP control traffic. */
  int tcp_port;
  char backend[32];
  stc_runtime_backend_t runtime_backend;
  int runtime_backend_explicitly_set;
  char can_if[16];
  int primary_node_id;
  int primary_tx_id;
  int primary_rx_id;
  int primary_motion_tx_id;
  int primary_motion_rx_id;
  double travel_min_deg;
  double travel_max_deg;
  double profile_max_speed_deg_s;
  double profile_max_accel_deg_s2;
  int watchdog_timeout_ms;
  int telemetry_interval_ms;
  int experimental_native_speed_mode;

  /** UDP fallback destination when per-stream address is not separately configured. */
  char tlm_dest_ip[16];
  char tlmh_ip[16];
  char tlml_ip[16];
  int tlmh_port;
  int tlml_port;
  int tlmh_interval_ms;
  int tlml_interval_ms;
  int can_bitrate;
  int can_restart_ms;
  int can_scan_max_node;
  int can_discovery_enabled;
  int can_timeout_ms;
  int startup_feedback_grace_ms;
  int strict_stale_policy;
  int allow_degraded_startup;
  int require_identity_for_pbit;
  int speed_watchdog_ms;
  int speed_stream_period_ms;
  double position_tolerance_deg;
  int position_timeout_ms;
  int position_poll_ms;

  int rt_enabled;
  int rt_strict;
  int rt_sched_policy;
  int can_worker_priority;
  int tlmh_tx_priority;
  int tlml_tx_priority;
  // 0 (default): allow compatibility controller_id 0x01 + legacy ICD IDs; 1: legacy IDs only.
  int controller_id_strict_legacy;

  /** Number of declared logical axes in use (<= STC_AXIS_COUNT). Defaults keep all axes disabled until explicit [AxisN] entries. */
  int num_axes;
  int axis_enabled[STC_AXIS_COUNT];
  /** Per-axis driver selection parsed from aliases (pd4c/nanotec/myact...). */
  stc_axis_driver_t axis_driver[STC_AXIS_COUNT];
  int axis_node_id[STC_AXIS_COUNT];
  double axis_command_sign[STC_AXIS_COUNT];
  double axis_telemetry_sign[STC_AXIS_COUNT];
  double axis_zero_physical_deg[STC_AXIS_COUNT];
  double axis_zero_raw_deg[STC_AXIS_COUNT];
  int axis_zero_on_start[STC_AXIS_COUNT];
  double axis_counts_per_rev[STC_AXIS_COUNT];
  double axis_gear_ratio[STC_AXIS_COUNT];
  double axis_vel_scale[STC_AXIS_COUNT];
  int axis_vel_raw_mode[STC_AXIS_COUNT];
  double axis_min_deg[STC_AXIS_COUNT];
  double axis_max_deg[STC_AXIS_COUNT];
  /** Per-axis speed cap (deg/s) used as motion speed_limit for ABS/REL commands. */
  double axis_max_speed_deg_s[STC_AXIS_COUNT];
  double axis_storage_deg[STC_AXIS_COUNT];
  int axis_brake_release_timeout_ms[STC_AXIS_COUNT];
  int axis_brake_poll_ms[STC_AXIS_COUNT];
  int axis_active_status2_enable[STC_AXIS_COUNT];
  int axis_active_status2_interval_ms[STC_AXIS_COUNT];
  int axis_feedback_timeout_ms[STC_AXIS_COUNT];
  uint16_t axis_pp_profile_vel[STC_AXIS_COUNT];
  uint8_t axis_diag_cat[STC_AXIS_COUNT];
  uint8_t axis_diag_subcat[STC_AXIS_COUNT];
  /** Turret logical mapping: azimuth/elevation axis indexes. */
  int turret_az_axis;
  int turret_el_axis;

  /** Number of explicitly mapped SUT entries (both az/el mapped to real axes). */
  int sut_count;
  int sut_az_axis[STC_MAX_SUT];
  int sut_el_axis[STC_MAX_SUT];
  int sut_mirror_axis[STC_MAX_SUT];
  uint8_t sut_laser_on[STC_MAX_SUT];

  /** Number of explicitly mapped mirror entries (axis mapped to a real axis). */
  int mirror_count;
  int mirror_axis[STC_MAX_MIRROR];
} stc_config_t;

/** Populate cfg with deterministic defaults before applying INI overrides. */
void stc_config_set_defaults(stc_config_t* cfg);
/** Parse INI file into cfg. Called once at startup; cfg is copied into runtime state. */
bool load_config(const char* path, stc_config_t* cfg);

#ifdef __cplusplus
}
#endif

#endif
