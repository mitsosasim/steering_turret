#ifndef STC_MODEL_H
#define STC_MODEL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STC_AXIS_COUNT 18u

typedef enum axis_vel_raw_units_e {
  AXIS_VEL_RAW_DEG_PER_SEC = 0,
  AXIS_VEL_RAW_RPM = 1,
  AXIS_VEL_RAW_COUNTS_PER_SEC = 2
} axis_vel_raw_units_t;

typedef struct axis_cfg_s {
  double counts_per_rev;
  double gear_ratio;
  axis_vel_raw_units_t vel_raw_units;
  double vel_scale;
  double az_min_deg;
  double az_max_deg;
  double max_speed_deg_s;
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
                       uint64_t ts_ms);

#ifdef __cplusplus
}
#endif

#endif
