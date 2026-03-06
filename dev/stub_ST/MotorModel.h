// Lightweight motor model with limited velocity and acceleration.
#pragma once

#include <algorithm>
#include <chrono>
#include <mutex>

class MotorModel {
public:
    // Instantaneous motor state exposed to callers.
    struct State {
        double position{0.0};
        double velocity{0.0};
        double acceleration{0.0};
    };

    // Physical constraints used by the integrator.
    struct Limits {
        double maxVelocity{45.0};     // unit per second
        double maxAcceleration{90.0}; // unit per second^2
        double minPosition{-180.0};
        double maxPosition{180.0};
    };

    MotorModel() = default;
    explicit MotorModel(const Limits& limits);

    // Updates motion limits and clamps current state/target into the new bounds.
    void setLimits(const Limits& limits);
    // Reinitializes model state and switches back to position mode.
    void reset(double position = 0.0, double velocity = 0.0);

    // Requests absolute position tracking.
    void setPositionTarget(double position);
    // Requests direct velocity tracking.
    void setVelocityTarget(double velocity);

    // Returns current state without integrating.
    State state() const;
    // Integrates one time step and returns updated state.
    State step();

private:
    // Position mode drives toward an absolute target, velocity mode tracks a signed rate.
    enum class ControlMode { Position, Velocity };

    mutable std::mutex mutex_;
    Limits limits_{};
    ControlMode mode_{ControlMode::Position};
    double target_{0.0};
    double position_{0.0};
    double velocity_{0.0};
    double acceleration_{0.0};
    std::chrono::steady_clock::time_point lastStepTime_{};
    bool hasLastStepTime_{false};
};
