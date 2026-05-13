#ifndef STC_MOTION_BACKEND_H
#define STC_MOTION_BACKEND_H

#include "stc_model.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct motion_backend motion_backend_t;

typedef struct {
  uint8_t supports_native_a2_speed;
  uint8_t supports_native_absolute_position;
} motion_backend_capabilities_t;

typedef struct {
  const char* can_if;
  char backend[32];
  uint8_t primary_node_id;
  uint32_t primary_tx_id;
  uint32_t primary_rx_id;
  uint32_t primary_motion_tx_id;
  uint32_t primary_motion_rx_id;
  double travel_min_deg;
  double travel_max_deg;
  double profile_max_speed_deg_s;
  double profile_max_accel_deg_s2;
  int telemetry_interval_ms;
  int experimental_native_speed_mode;
  int can_bitrate;
  int can_restart_ms;
  int can_scan_max_node;
  int speed_watchdog_ms;
  int ds402_timeout_ms;
  int pp_sdo_diag;
  uint32_t can_timeout_ms;
  axis_cfg_t axis_cfg[STC_AXIS_COUNT];
  uint8_t axis_node_id[STC_AXIS_COUNT];
} motion_backend_cfg_t;

typedef struct {
  uint8_t backend_mode;      /* diagnostic only */
  uint8_t transport_status;  /* diagnostic only */
  motion_backend_capabilities_t capabilities;
  int can_ok;
  uint8_t axis_available[STC_AXIS_COUNT];
  uint8_t can_discovered_nodes[16];
  uint8_t can_connected_devices;
  uint8_t can_discovered_devices;
  uint64_t last_rx_ms[STC_AXIS_COUNT];

  int32_t pos_raw[STC_AXIS_COUNT];
  int32_t vel_raw[STC_AXIS_COUNT];
  uint16_t status_word[STC_AXIS_COUNT];
  int8_t mode_display[STC_AXIS_COUNT];
  uint8_t fault_active[STC_AXIS_COUNT];
  uint32_t command_age_ms[STC_AXIS_COUNT];
  uint8_t cmd_ack_no_motion[STC_AXIS_COUNT];
  uint32_t diag_ext[STC_AXIS_COUNT]; /* backend-specific optional diagnostic detail */

  uint8_t hb_state;
  int have_hb;
  uint16_t emcy_code;
  uint8_t emcy_reg;
  int have_emcy;
  uint64_t ts_tpdo1_ms;
  uint64_t ts_tpdo2_ms;
  uint64_t ts_tpdo3_ms;
  uint64_t ts_hb_ms;
  uint64_t ts_emcy_ms;
} motion_snapshot_t;

int motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend);
void motion_backend_stop(motion_backend_t* backend);
int motion_backend_discover(motion_backend_t* backend);
int motion_backend_apply_pp_move(motion_backend_t* backend, uint32_t axes_mask, const double targets_deg[STC_AXIS_COUNT]);
int motion_backend_apply_speed(motion_backend_t* backend, uint32_t axes_mask, const double speeds_deg_s[STC_AXIS_COUNT]);
int motion_backend_quick_stop(motion_backend_t* backend, uint32_t axes_mask);
int motion_backend_clear_faults(motion_backend_t* backend, uint32_t axes_mask);
int motion_backend_set_laser(motion_backend_t* backend, uint8_t laser_id, uint8_t on_off);
int motion_backend_get_snapshot(motion_backend_t* backend, motion_snapshot_t* snapshot);

#ifdef __cplusplus
}
#endif

#endif
