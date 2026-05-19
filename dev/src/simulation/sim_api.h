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

#ifndef STC_SIM_API_H
#define STC_SIM_API_H

#include "stc_model.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  SIM_MODE_DISABLED = 0,
  SIM_MODE_PP = 1,
  SIM_MODE_PV = 2
} sim_axis_mode_t;

void sim_init(const axis_cfg_t* cfg, size_t axis_count);
void sim_shutdown(void);
void sim_update(double dt_s);

void sim_set_position_target(size_t axis, double target_deg);
void sim_set_velocity_target(size_t axis, double speed_deg_s);
void sim_quick_stop(size_t axis);
void sim_reset_axis(size_t axis);

int32_t sim_get_pos_counts(size_t axis);
int32_t sim_get_vel_raw(size_t axis);
uint16_t sim_get_status_word(size_t axis);
int8_t sim_get_mode_display(size_t axis);
uint64_t sim_get_last_rx_ms(size_t axis);

#ifdef __cplusplus
}
#endif

#endif
