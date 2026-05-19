#ifndef STC_MODEL_H
#define STC_MODEL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STC_AXIS_COUNT 18u

typedef enum axis_vel_raw_units_e {
  AXIS_VEL_RAW_DEG_PER_SEC = 0
} axis_vel_raw_units_t;

typedef enum stc_axis_driver_e {
  STC_AXIS_DRIVER_DISABLED = 0,
  STC_AXIS_DRIVER_PD4C,
  STC_AXIS_DRIVER_MYACT_X8_25,
  STC_AXIS_DRIVER_SIM
} stc_axis_driver_t;

const char* axis_driver_to_string(stc_axis_driver_t driver);

typedef struct axis_cfg_s {
  stc_axis_driver_t driver;
  int node_id;

  double min_deg;
  double max_deg;
  double max_speed_deg_s;
  double storage_deg;

  double command_sign;
  double telemetry_sign;
  double zero_physical_deg;
  double zero_raw_deg;
  int zero_on_start;

  int brake_release_timeout_ms;
  int brake_poll_ms;
  int active_status2_enable;
  int active_status2_interval_ms;
  int feedback_timeout_ms;

  /* PD4C-only fields. */
  double counts_per_rev;
  double gear_ratio;
  axis_vel_raw_units_t vel_raw_units;
  double vel_scale;
  double az_min_deg;
  double az_max_deg;
  int vel_raw_mode;
  uint16_t pp_profile_vel;
} axis_cfg_t;

typedef struct axis_telem_s {
  int32_t pos_raw;
  int32_t vel_raw;
  uint16_t status_word;
  int8_t mode_disp;

  double pos_deg;
  double vel_deg_s;
  double accel_deg_s2;

  uint64_t ts_ms;
  double prev_vel_deg_s;
  uint64_t prev_ts_ms;
} axis_telem_t;

typedef struct axis_stream_s {
  bool enabled;
  double desired_speed_deg_s;
  uint64_t last_cmd_ms;
} axis_stream_t;

double axis_counts_to_deg(const axis_cfg_t* cfg, int32_t counts);
int32_t axis_deg_to_counts(const axis_cfg_t* cfg, double deg);

double axis_vel_to_deg_s(const axis_cfg_t* cfg, int32_t vel_raw);
int32_t axis_deg_s_to_target_vel_units(const axis_cfg_t* cfg, double axis_deg_s, int* clipped);

void axis_telem_update(axis_telem_t* telem,
                       const axis_cfg_t* cfg,
                       int32_t pos_raw,
                       int32_t vel_raw,
                       uint16_t status_word,
                       int8_t mode_disp,
                       uint64_t ts_ms);

#ifdef __cplusplus
}
#endif

#endif
