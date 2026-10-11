/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Defines the generic motion backend interface consumed by stc_server.
 *
 * Design notes:
 *   Exposes one abstraction for real CANopen hardware and simulator backends.
 *   The CAN worker thread owns backend lifetime and all backend API calls.
 */

#ifndef STC_MOTION_BACKEND_H
#define STC_MOTION_BACKEND_H

#include "stc_model.h"
#include "stc_actions.h"

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque backend handle implemented by the motion backend module. */
typedef struct motion_backend motion_backend_t;

/** Backend feature flags advertised in snapshots to gate optional command paths. */
typedef struct {
  uint8_t supports_native_a2_speed;
  uint8_t supports_native_absolute_position;
} motion_backend_capabilities_t;

/**
 * Static backend configuration copied from parsed INI config into runtime.
 * Ownership: caller-owned during start; backend copies required values.
 */
/**
 * Runtime backend configuration consumed by motion_backend_start().
 *
 * Exposes transport details (CAN iface/IDs), motion limits, DS402/watchdog timing,
 * and per-axis mapping copied from parsed configuration.
 *
 * Ownership/threading: stc_server builds this once, then the CAN worker thread
 * passes it to motion_backend_start() and remains the sole caller of backend APIs.
 */
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
  int can_discovery_enabled;
  int speed_watchdog_ms;
  double position_tolerance_deg;
  int position_timeout_ms;
  int position_poll_ms;
  int require_identity_for_pbit;
  int ds402_timeout_ms;
  int pp_sdo_diag;
  uint32_t can_timeout_ms;
  int axis_enabled[STC_AXIS_COUNT];
  axis_cfg_t axis_cfg[STC_AXIS_COUNT];
  uint8_t axis_node_id[STC_AXIS_COUNT];
} motion_backend_cfg_t;

/**
 * Snapshot view of backend connectivity, diagnostics, and last known axis telemetry.
 * Filled by motion_backend_get_snapshot() for state-machine/telemetry publishing.
 */
typedef struct {
  uint8_t backend_mode;      /* diagnostic only */
  uint8_t transport_status;  /* diagnostic only */
  motion_backend_capabilities_t capabilities;
  int can_ok;
  /* Startup inventory source for PBIT (directly reported by backend startup inventory). */
  uint8_t expected_motors;
  uint8_t connected_motors;
  uint8_t axis_configured[STC_AXIS_COUNT];
  uint8_t axis_available[STC_AXIS_COUNT];
  uint8_t can_discovered_nodes[16];
  uint8_t can_connected_devices;
  uint8_t can_discovered_devices;
  uint64_t last_rx_ms[STC_AXIS_COUNT];
  uint8_t axis_fault_active[STC_AXIS_COUNT];
  uint8_t axis_connected[STC_AXIS_COUNT];
  uint8_t axis_stale[STC_AXIS_COUNT];
  uint8_t axis_moving[STC_AXIS_COUNT];
  stc_axis_health_t axis_health[STC_AXIS_COUNT];

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

typedef struct {
  stc_axis_driver_t driver;
  uint8_t enabled;
  uint8_t connected;
  uint8_t available;
  uint8_t stale;
  int32_t last_error;
} motion_axis_inventory_t;


/** Count enabled hardware CAN axes per driver from static config (dry-run helper for tests). */
static inline void hardware_backend_count_configured_axes(const motion_backend_cfg_t* cfg,
                                                          int* pd4c_count,
                                                          int* myact_count) {
  int pd4c = 0;
  int myact = 0;
  if (cfg) {
    for (size_t axis = 0; axis < STC_AXIS_COUNT; ++axis) {
      if (!cfg->axis_enabled[axis]) continue;
      if (cfg->axis_cfg[axis].driver == STC_AXIS_DRIVER_PD4C) pd4c++;
      else if (cfg->axis_cfg[axis].driver == STC_AXIS_DRIVER_MYACT_X8_25) myact++;
    }
  }
  if (pd4c_count) *pd4c_count = pd4c;
  if (myact_count) *myact_count = myact;
}

/** Create and initialize backend instance. Called by CAN worker at startup. */
/** Create backend instance from cfg; called once by CAN worker during startup. */
int motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend);
/** Stop backend activity and release resources; called by CAN worker during shutdown. */
void motion_backend_stop(motion_backend_t* backend);
/** Probe transport/device availability and refresh discovered/connected diagnostics. */
int motion_backend_discover(motion_backend_t* backend);
/** Apply profile-position targets (degrees) on axes selected by axes_mask. */
int motion_backend_apply_pp_move(motion_backend_t* backend, uint32_t axes_mask, const double targets_deg[STC_AXIS_COUNT]);
/** Apply mixed per-axis motion kinds via one API used by action execution path (native backend commands, no protocol fallback translation). */
int motion_backend_apply_motion(motion_backend_t* backend, uint32_t axes_mask, const stc_axis_motion_kind_t kind[STC_AXIS_COUNT], const double values_deg_or_dps[STC_AXIS_COUNT], const double speed_limit_deg_s[STC_AXIS_COUNT]);
/** Apply profile-velocity/stream speed targets (deg/s) on selected axes. */
int motion_backend_apply_speed(motion_backend_t* backend, uint32_t axes_mask, const double speeds_deg_s[STC_AXIS_COUNT]);
/** Request DS402 quick-stop (or simulator equivalent) on selected axes. */
int motion_backend_quick_stop(motion_backend_t* backend, uint32_t axes_mask);
/** Attempt per-axis fault reset/clear sequence on selected axes. */
int motion_backend_clear_faults(motion_backend_t* backend, uint32_t axes_mask);
/** Forward laser command for backend implementations that expose laser control. */
int motion_backend_set_laser(motion_backend_t* backend, uint8_t laser_id, uint8_t on_off);
/** Copy latest backend snapshot for telemetry/diagnostics consumption by server. */
int motion_backend_get_snapshot(motion_backend_t* backend, motion_snapshot_t* snapshot);
int motion_backend_get_axis_inventory(motion_backend_t* backend, motion_axis_inventory_t out_axes[STC_AXIS_COUNT]);

/* Dedicated MYACT backend entry points (used when backend=myact_x8_25). */
int myact_motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend);
void myact_motion_backend_stop(motion_backend_t* backend);
int myact_motion_backend_discover(motion_backend_t* backend);
int myact_motion_backend_apply_pp_move(motion_backend_t* backend, uint32_t axes_mask, const double targets_deg[STC_AXIS_COUNT]);
int myact_motion_backend_apply_motion(motion_backend_t* backend, uint32_t axes_mask, const stc_axis_motion_kind_t kind[STC_AXIS_COUNT], const double values_deg_or_dps[STC_AXIS_COUNT], const double speed_limit_deg_s[STC_AXIS_COUNT]);
int myact_motion_backend_apply_speed(motion_backend_t* backend, uint32_t axes_mask, const double speeds_deg_s[STC_AXIS_COUNT]);
int myact_motion_backend_quick_stop(motion_backend_t* backend, uint32_t axes_mask);
int myact_motion_backend_clear_faults(motion_backend_t* backend, uint32_t axes_mask);
int myact_motion_backend_set_laser(motion_backend_t* backend, uint8_t laser_id, uint8_t on_off);
int myact_motion_backend_get_snapshot(motion_backend_t* backend, motion_snapshot_t* snapshot);

#ifdef __cplusplus
}
#endif

#endif
