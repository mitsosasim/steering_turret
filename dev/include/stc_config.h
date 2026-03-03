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
  char mcast_ip[16];
  int mcast_tlmh_port;
  int mcast_tlml_port;
  int tlmh_interval_ms;
  int tlml_interval_ms;

  int num_axes;
  int axis_node_id[STC_AXIS_COUNT];
  double axis_counts_per_rev[STC_AXIS_COUNT];
  double axis_gear_ratio[STC_AXIS_COUNT];
  int axis_vel_is_rpm[STC_AXIS_COUNT];
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

  int mirror_count;
  int mirror_axis[STC_MAX_MIRROR];
} stc_config_t;

void stc_config_set_defaults(stc_config_t* cfg);
bool load_config(const char* path, stc_config_t* cfg);

#ifdef __cplusplus
}
#endif

#endif
