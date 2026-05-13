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

typedef struct stc_config_s {
  int tcp_port;
  char backend[32];
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

  char tlm_dest_ip[16];
  int tlmh_port;
  int tlml_port;
  int tlmh_interval_ms;
  int tlml_interval_ms;
  int can_scan_max_node;
  int can_timeout_ms;
  int strict_stale_policy;

  int rt_enabled;
  int rt_strict;
  int rt_sched_policy;
  int can_worker_priority;
  int tlmh_tx_priority;
  int tlml_tx_priority;
  // 0 (default): allow compatibility controller_id 0x01 + legacy ICD IDs; 1: legacy IDs only.
  int controller_id_strict_legacy;

  int num_axes;
  int axis_node_id[STC_AXIS_COUNT];
  double axis_counts_per_rev[STC_AXIS_COUNT];
  double axis_gear_ratio[STC_AXIS_COUNT];
  double axis_min_deg[STC_AXIS_COUNT];
  double axis_max_deg[STC_AXIS_COUNT];
  double axis_max_speed_deg_s[STC_AXIS_COUNT];
  double axis_storage_deg[STC_AXIS_COUNT];
  uint16_t axis_pp_profile_vel[STC_AXIS_COUNT];
  uint8_t axis_diag_cat[STC_AXIS_COUNT];
  uint8_t axis_diag_subcat[STC_AXIS_COUNT];
  int8_t motor_status_map[8];

  int turret_az_axis;
  int turret_el_axis;

  int sut_count;
  int sut_az_axis[STC_MAX_SUT];
  int sut_el_axis[STC_MAX_SUT];
  uint8_t sut_laser_on[STC_MAX_SUT];

  int mirror_count;
  int mirror_axis[STC_MAX_MIRROR];
} stc_config_t;

void stc_config_set_defaults(stc_config_t* cfg);
bool load_config(const char* path, stc_config_t* cfg);

#ifdef __cplusplus
}
#endif

#endif
