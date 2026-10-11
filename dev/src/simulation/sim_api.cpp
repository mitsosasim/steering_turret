/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Implements a lightweight simulated axis plant used by STC backend emulation.
 *
 * Design notes:
 *   Mirrors backend-facing motion semantics (position/velocity/stop/reset) with
 *   deterministic development behavior and no CAN dependency; not intended as a
 *   real hardware timing or bus-contention model.
 */

#include "sim_api.h"

#include "MotorModel.h"
#include "pd4c.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <time.h>

namespace {
constexpr size_t kAxisCount = STC_AXIS_COUNT;
constexpr uint16_t kStatusOperationEnabled = 0x1237u;

/* Per-axis simulator ownership: plant model, static config, and protocol-facing mode/timestamp state. */
struct SimAxis {
  MotorModel model{};
  axis_cfg_t cfg{};
  sim_axis_mode_t mode{SIM_MODE_DISABLED};
  uint64_t last_rx_ms{0};
};

/* Global simulator state and lock shared by the C API; mutex serializes all axis reads/writes. */
struct SimState {
  std::array<SimAxis, kAxisCount> axes{};
  bool initialized{false};
  std::mutex mtx;
} g_sim;

static double safe(double v, double fallback) {
  return (v > 0.0) ? v : fallback;
}

static uint64_t now_ms_steady() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000ULL + static_cast<uint64_t>(ts.tv_nsec / 1000000ULL);
}

static int32_t vel_deg_s_to_raw(const axis_cfg_t* cfg, double axis_deg_s) {
  const double gr = safe(cfg ? cfg->gear_ratio : 0.0, 1.0);
  const double scale = (cfg && cfg->vel_scale != 0.0) ? cfg->vel_scale : 1.0;

  const double motor_deg_s = axis_deg_s * gr;
  double raw = motor_deg_s / scale;

  if (raw > static_cast<double>(INT32_MAX)) raw = static_cast<double>(INT32_MAX);
  if (raw < static_cast<double>(INT32_MIN)) raw = static_cast<double>(INT32_MIN);
  return static_cast<int32_t>(std::llround(raw));
}

static MotorModel::Limits limits_from_cfg(const axis_cfg_t* cfg) {
  MotorModel::Limits limits;
  limits.minPosition = cfg ? cfg->az_min_deg : -180.0;
  limits.maxPosition = cfg ? cfg->az_max_deg : 180.0;
  limits.maxVelocity = safe(cfg ? cfg->max_speed_deg_s : 0.0, 45.0);
  limits.maxAcceleration = limits.maxVelocity * 4.0;
  return limits;
}

static bool axis_valid(size_t axis) {
  return axis < kAxisCount;
}
} // namespace

extern "C" {

void sim_init(const axis_cfg_t* cfg, size_t axis_count) {
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  const size_t n = (axis_count < kAxisCount) ? axis_count : kAxisCount;
  for (size_t i = 0; i < kAxisCount; ++i) {
    if (i < n && cfg) {
      g_sim.axes[i].cfg = cfg[i];
      g_sim.axes[i].model.setLimits(limits_from_cfg(&cfg[i]));
    } else {
      axis_cfg_t dflt{};
      dflt.counts_per_rev = 4096.0;
      dflt.gear_ratio = 1.0;
      dflt.vel_raw_units = AXIS_VEL_RAW_DEG_PER_SEC;
      dflt.vel_scale = 1.0;
      dflt.az_min_deg = -180.0;
      dflt.az_max_deg = 180.0;
      dflt.max_speed_deg_s = 45.0;
      dflt.pp_profile_vel = 100;
      g_sim.axes[i].cfg = dflt;
      g_sim.axes[i].model.setLimits(limits_from_cfg(&dflt));
    }
    g_sim.axes[i].mode = SIM_MODE_DISABLED;
    g_sim.axes[i].model.reset(0.0, 0.0);
    g_sim.axes[i].last_rx_ms = now_ms_steady();
  }
  g_sim.initialized = true;
}

void sim_shutdown(void) {
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  g_sim.initialized = false;
}

void sim_update(double /*dt_s*/) {
  /* Integrator tick: advance all axes and publish a fresh rx timestamp heartbeat. */
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  if (!g_sim.initialized) return;
  const uint64_t now = now_ms_steady();
  for (size_t i = 0; i < kAxisCount; ++i) {
    (void)g_sim.axes[i].model.step();
    g_sim.axes[i].last_rx_ms = now;
  }
}

void sim_set_position_target(size_t axis, double target_deg) {
  /* Profile-position mode: target position is tracked by motor model over time. */
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  if (!g_sim.initialized || !axis_valid(axis)) return;
  g_sim.axes[axis].mode = SIM_MODE_PP;
  g_sim.axes[axis].model.setPositionTarget(target_deg);
}

void sim_set_velocity_target(size_t axis, double speed_deg_s) {
  /* Velocity mode: command persistent speed target until quick-stop/reset/override. */
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  if (!g_sim.initialized || !axis_valid(axis)) return;
  g_sim.axes[axis].mode = SIM_MODE_PV;
  g_sim.axes[axis].model.setVelocityTarget(speed_deg_s);
}

void sim_quick_stop(size_t axis) {
  /* Quick-stop emulation: preserve position, force instantaneous zero velocity. */
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  if (!g_sim.initialized || !axis_valid(axis)) return;
  auto st = g_sim.axes[axis].model.state();
  g_sim.axes[axis].model.reset(st.position, 0.0);
  g_sim.axes[axis].mode = SIM_MODE_DISABLED;
}

void sim_reset_axis(size_t axis) {
  /* Reset emulation: return axis model to origin and clear active mode. */
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  if (!g_sim.initialized || !axis_valid(axis)) return;
  g_sim.axes[axis].model.reset(0.0, 0.0);
  g_sim.axes[axis].mode = SIM_MODE_DISABLED;
}

int32_t sim_get_pos_counts(size_t axis) {
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  if (!g_sim.initialized || !axis_valid(axis)) return 0;
  auto st = g_sim.axes[axis].model.state();
  return axis_deg_to_counts(&g_sim.axes[axis].cfg, st.position);
}

int32_t sim_get_vel_raw(size_t axis) {
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  if (!g_sim.initialized || !axis_valid(axis)) return 0;
  auto st = g_sim.axes[axis].model.state();
  return vel_deg_s_to_raw(&g_sim.axes[axis].cfg, st.velocity);
}

uint16_t sim_get_status_word(size_t axis) {
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  if (!g_sim.initialized || !axis_valid(axis)) return 0;
  (void)axis;
  return kStatusOperationEnabled;
}

int8_t sim_get_mode_display(size_t axis) {
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  if (!g_sim.initialized || !axis_valid(axis)) return 0;
  if (g_sim.axes[axis].mode == SIM_MODE_PP) return static_cast<int8_t>(PD4C_MODE_PROFILE_POSITION);
  if (g_sim.axes[axis].mode == SIM_MODE_PV) return static_cast<int8_t>(PD4C_MODE_PROFILE_VELOCITY);
  return 0;
}

uint64_t sim_get_last_rx_ms(size_t axis) {
  std::lock_guard<std::mutex> lock(g_sim.mtx);
  if (!g_sim.initialized || !axis_valid(axis)) return 0;
  return g_sim.axes[axis].last_rx_ms;
}

} // extern "C"
