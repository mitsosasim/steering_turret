/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Simulation-side axis API used by the STC simulator backend implementation.
 *
 * Design notes:
 *   Mirrors motion backend behaviors (PP/PV, quick-stop, reset, telemetry reads)
 *   without requiring real CAN devices. Deterministic for development/testing
 *   workflows, but not a cycle-accurate hardware timing model.
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

/** Initialize simulator state from axis configuration copied at backend start. */
void sim_init(const axis_cfg_t* cfg, size_t axis_count);
/** Tear down simulator state; no external transport resources are owned. */
void sim_shutdown(void);
/** Advance internal motor models one integrator step (deterministic dev behavior). */
/** One integrator tick for all simulated axes; updates state heartbeat timestamps. */
void sim_update(double dt_s);

/** Set profile-position style target in degrees for one axis. */
void sim_set_position_target(size_t axis, double target_deg);
/** Set profile-velocity style target in deg/s for one axis. */
void sim_set_velocity_target(size_t axis, double speed_deg_s);
/** Stop one simulated axis immediately (quick-stop equivalent). */
void sim_quick_stop(size_t axis);
/** Reset one axis model state to neutral origin. */
void sim_reset_axis(size_t axis);

/** Read latest simulated position converted to raw counts for backend snapshot use. */
int32_t sim_get_pos_counts(size_t axis);
/** Read latest simulated velocity converted to driver raw units. */
int32_t sim_get_vel_raw(size_t axis);
/** Read DS402-like statusword exposed by simulator for protocol compatibility. */
uint16_t sim_get_status_word(size_t axis);
/** Read mode display value aligned with PD4-C operation-mode constants. */
int8_t sim_get_mode_display(size_t axis);
/** Read last simulated telemetry timestamp in monotonic milliseconds. */
uint64_t sim_get_last_rx_ms(size_t axis);

#ifdef __cplusplus
}
#endif

#endif
