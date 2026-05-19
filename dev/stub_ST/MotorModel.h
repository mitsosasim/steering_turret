/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Simulated single-axis motor dynamics model used by the ST stub.
 *
 * Design notes:
 *   Supports position-target and velocity-target modes with bounded acceleration,
 *   velocity, and position limits; integrated in fixed simulator time steps.
 */

// Lightweight motor model with limited velocity and acceleration.
#pragma once

#include <algorithm>
#include <chrono>
#include <mutex>

/** @brief Lightweight axis plant model for turret/SUT simulation. */
class MotorModel {
public:
    /** @brief Instantaneous motor kinematic state in axis units (degrees, deg/s, deg/s^2). */
    struct State {
        double position{0.0};
        double velocity{0.0};
        double acceleration{0.0};
    };

    /** @brief Axis limits enforced during commanding and integration. */
    struct Limits {
        double maxVelocity{45.0};     // unit per second
        double maxAcceleration{90.0}; // unit per second^2
        double minPosition{-180.0};
        double maxPosition{180.0};
    };

    MotorModel() = default;
    explicit MotorModel(const Limits& limits);

    /** @brief Update constraints and clamp stored state/targets to the new range. */
    void setLimits(const Limits& limits);
    /** @brief Reset model state and return control mode to absolute-position tracking. */
    void reset(double position = 0.0, double velocity = 0.0);

    /** @brief Set absolute position target (position mode). */
    void setPositionTarget(double position);
    /** @brief Set signed velocity target (velocity mode). */
    void setVelocityTarget(double velocity);

    /** @brief Read state snapshot without advancing integration. */
    State state() const;
    /** @brief Integrate one step using elapsed monotonic time and return new state. */
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
