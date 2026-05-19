/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Implements the motion_backend API using an in-process simulator backend.
 *
 * Design notes:
 *   Presents the same backend contract that stc_server uses for real CAN, so
 *   command/state/telemetry paths can run unchanged in development environments.
 *   Deterministic enough for functional testing, but not a hardware timing model.
 */

#include "motion_backend.h"

#include "MotorModel.h"
#include "pd4c.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>

namespace {
struct SimAxis {
  MotorModel model{};
  axis_cfg_t cfg{};
  bool has_position_target{false};
  bool has_speed_target{false};
  double speed_target_deg_s{0.0};
  uint64_t last_speed_cmd_ms{0};
  uint64_t last_rx_ms{0};
  int8_t mode_display{0};
};


static uint64_t now_ms() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static double safe(double v, double fallback) { return (v > 0.0) ? v : fallback; }

static MotorModel::Limits limits_from_cfg(const axis_cfg_t& cfg) {
  MotorModel::Limits limits;
  limits.minPosition = cfg.az_min_deg;
  limits.maxPosition = cfg.az_max_deg;
  limits.maxVelocity = safe(cfg.max_speed_deg_s, 45.0);
  limits.maxAcceleration = limits.maxVelocity * 4.0;
  return limits;
}

static int32_t vel_deg_s_to_raw(const axis_cfg_t& cfg, double axis_deg_s) {
  const double gr = safe(cfg.gear_ratio, 1.0);
  const double scale = (cfg.vel_scale != 0.0) ? cfg.vel_scale : 1.0;
  const double motor_deg_s = axis_deg_s * gr;
  double raw = motor_deg_s / scale;
  if (raw > static_cast<double>(INT32_MAX)) raw = static_cast<double>(INT32_MAX);
  if (raw < static_cast<double>(INT32_MIN)) raw = static_cast<double>(INT32_MIN);
  return static_cast<int32_t>(std::llround(raw));
}

}  // namespace

struct motion_backend {
  motion_backend_cfg_t cfg{};
  std::array<SimAxis, STC_AXIS_COUNT> axes{};
  motion_snapshot_t snap{};
  std::mutex mtx;
  std::thread integrator;
  std::atomic<bool> running{false};
};





static void integrator_loop(motion_backend* b) {
  using namespace std::chrono;
  constexpr auto period = milliseconds(10);
  auto next = steady_clock::now() + period;
  /* Fixed-period integration loop approximating backend telemetry cadence. */
  while (b->running.load()) {
    {
      std::lock_guard<std::mutex> lock(b->mtx);
      uint64_t t_ms = now_ms();
      for (size_t axis = 0; axis < STC_AXIS_COUNT; ++axis) {
        auto& a = b->axes[axis];
        /* Match real backend speed-stream watchdog behavior when commands stop. */
        if (a.has_speed_target && b->cfg.speed_watchdog_ms > 0 &&
            (t_ms - a.last_speed_cmd_ms) > static_cast<uint64_t>(b->cfg.speed_watchdog_ms)) {
          a.has_speed_target = false;
          a.speed_target_deg_s = 0.0;
          a.model.setVelocityTarget(0.0);
          a.mode_display = 0;
        }
        if (!a.has_position_target && !a.has_speed_target) continue;
        /* Plant propagation for active PP/PV modes; values feed snapshot telemetry. */
        auto st = a.model.step();
        b->snap.pos_raw[axis] = axis_deg_to_counts(&a.cfg, st.position);
        b->snap.vel_raw[axis] = vel_deg_s_to_raw(a.cfg, st.velocity);
        b->snap.status_word[axis] = 0x1237u;
        b->snap.mode_display[axis] = a.mode_display;
        b->snap.last_rx_ms[axis] = t_ms;
      }
    }
    std::this_thread::sleep_until(next);
    next += period;
  }
}

extern "C" {

int sim_motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend) {
  if (!cfg || !out_backend) return -1;
  motion_backend* b = new motion_backend();
  b->cfg = *cfg;
  const uint64_t t_ms = now_ms();
  for (size_t axis = 0; axis < STC_AXIS_COUNT; ++axis) {
    b->axes[axis].cfg = cfg->axis_cfg[axis];
    b->axes[axis].model.setLimits(limits_from_cfg(cfg->axis_cfg[axis]));
    b->axes[axis].model.reset(0.0, 0.0);
    b->axes[axis].last_rx_ms = t_ms;
    b->snap.axis_available[axis] = 1;
    b->snap.last_rx_ms[axis] = t_ms;
    b->snap.status_word[axis] = 0x1237u;
  }
  b->snap.can_ok = 1;
  b->snap.capabilities.supports_native_a2_speed = 1;
  b->snap.capabilities.supports_native_absolute_position = 1;
  b->snap.backend_mode = 3;
  b->snap.transport_status = 1;
  b->snap.can_connected_devices = STC_AXIS_COUNT;
  b->running.store(true);
  b->integrator = std::thread(integrator_loop, b);
  *out_backend = b;
  return 0;
}

void sim_motion_backend_stop(motion_backend_t* backend) {
  motion_backend* b = reinterpret_cast<motion_backend*>(backend);
  if (!b) return;
  b->running.store(false);
  if (b->integrator.joinable()) b->integrator.join();
  delete b;
}

int sim_motion_backend_discover(motion_backend_t* backend) {
  motion_backend* b = reinterpret_cast<motion_backend*>(backend);
  if (!b) return -1;
  std::lock_guard<std::mutex> lock(b->mtx);
  b->snap.can_discovered_devices = STC_AXIS_COUNT;
  for (size_t i = 0; i < 16; ++i) b->snap.can_discovered_nodes[i] = static_cast<uint8_t>(i + 1);
  return STC_AXIS_COUNT;
}

int sim_motion_backend_apply_pp_move(motion_backend_t* backend, uint32_t axes_mask, const double targets_deg[STC_AXIS_COUNT]) {
  motion_backend* b = reinterpret_cast<motion_backend*>(backend);
  if (!b || !targets_deg) return -1;
  /* Profile-position path mirrors motion_backend_apply_pp_move contract. */
  std::lock_guard<std::mutex> lock(b->mtx);
  for (size_t axis = 0; axis < STC_AXIS_COUNT; ++axis) {
    if ((axes_mask & (1u << axis)) == 0) continue;
    b->axes[axis].has_position_target = true;
    b->axes[axis].has_speed_target = false;
    b->axes[axis].model.setPositionTarget(targets_deg[axis]);
    b->axes[axis].mode_display = static_cast<int8_t>(PD4C_MODE_PROFILE_POSITION);
    b->snap.mode_display[axis] = b->axes[axis].mode_display;
  }
  return 0;
}

int sim_motion_backend_apply_speed(motion_backend_t* backend, uint32_t axes_mask, const double speeds_deg_s[STC_AXIS_COUNT]) {
  motion_backend* b = reinterpret_cast<motion_backend*>(backend);
  if (!b || !speeds_deg_s) return -1;
  const uint64_t t_ms = now_ms();
  /* Velocity mode path mirrors motion_backend_apply_speed and refreshes watchdog time. */
  std::lock_guard<std::mutex> lock(b->mtx);
  for (size_t axis = 0; axis < STC_AXIS_COUNT; ++axis) {
    if ((axes_mask & (1u << axis)) == 0) continue;
    b->axes[axis].has_position_target = false;
    b->axes[axis].has_speed_target = true;
    b->axes[axis].speed_target_deg_s = speeds_deg_s[axis];
    b->axes[axis].last_speed_cmd_ms = t_ms;
    b->axes[axis].model.setVelocityTarget(speeds_deg_s[axis]);
    b->axes[axis].mode_display = static_cast<int8_t>(PD4C_MODE_PROFILE_VELOCITY);
    b->snap.mode_display[axis] = b->axes[axis].mode_display;
  }
  return 0;
}

int sim_motion_backend_quick_stop(motion_backend_t* backend, uint32_t axes_mask) {
  motion_backend* b = reinterpret_cast<motion_backend*>(backend);
  if (!b) return -1;
  /* Quick-stop mirrors DS402 intent: hold position and zero commanded velocity. */
  std::lock_guard<std::mutex> lock(b->mtx);
  for (size_t axis = 0; axis < STC_AXIS_COUNT; ++axis) {
    if ((axes_mask & (1u << axis)) == 0) continue;
    auto st = b->axes[axis].model.state();
    b->axes[axis].model.reset(st.position, 0.0);
    b->axes[axis].has_position_target = false;
    b->axes[axis].has_speed_target = false;
    b->axes[axis].mode_display = 0;
    b->snap.mode_display[axis] = 0;
    b->snap.vel_raw[axis] = 0;
  }
  return 0;
}


int sim_motion_backend_clear_faults(motion_backend_t* backend, uint32_t axes_mask) {
  motion_backend* b = reinterpret_cast<motion_backend*>(backend);
  if (!b) return -1;
  /* Fault clear is modeled by restoring nominal statusword in snapshot. */
  std::lock_guard<std::mutex> lock(b->mtx);
  for (size_t axis = 0; axis < STC_AXIS_COUNT; ++axis) {
    if ((axes_mask & (1u << axis)) == 0) continue;
    b->snap.status_word[axis] = 0x1237u;
  }
  return 0;
}

int sim_motion_backend_set_laser(motion_backend_t* backend, uint8_t laser_id, uint8_t on_off) {
  motion_backend* b = reinterpret_cast<motion_backend*>(backend);
  if (!b) return -1;
  std::lock_guard<std::mutex> lock(b->mtx);
  (void)laser_id;
  (void)on_off;
  return 0;
}

int sim_motion_backend_get_snapshot(motion_backend_t* backend, motion_snapshot_t* snapshot) {
  motion_backend* b = reinterpret_cast<motion_backend*>(backend);
  if (!b || !snapshot) return -1;
  /* Snapshot copy emulates backend polling: caller receives consistent cached state. */
  std::lock_guard<std::mutex> lock(b->mtx);
  *snapshot = b->snap;
  return 0;
}

}  // extern "C"
