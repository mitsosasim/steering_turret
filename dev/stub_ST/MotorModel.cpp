/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   simple motor dynamics model used by simulator paths.
 *
 * Design notes:
 *   favors deterministic bench behavior over a detailed hardware model.
 *   Integrates axis kinematics in degrees/deg/s/deg/s^2 from monotonic clock deltas.
 *   Applies acceleration, velocity, and position limits as simulator safety gates.
 */

// MotorModel.cpp - simple kinematic motor simulator.
#include "MotorModel.h"

#include <algorithm>
#include <cmath>

namespace {
// Clamps a signed value to +/-limit while preserving sign.
double clampWithSign(double value, double limit) {
    if (limit <= 0.0) {
        return 0.0;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}
} // namespace

MotorModel::MotorModel(const Limits& limits) : limits_(limits) {
    // Accept swapped min/max values in config and normalize them.
    if (limits_.minPosition > limits_.maxPosition) {
        std::swap(limits_.minPosition, limits_.maxPosition);
    }
    position_ = std::clamp(position_, limits_.minPosition, limits_.maxPosition);
    target_ = std::clamp(target_, limits_.minPosition, limits_.maxPosition);
}

void MotorModel::setLimits(const Limits& limits) {
    std::lock_guard<std::mutex> lock(mutex_);
    limits_ = limits;
    // Keep limits robust even if config accidentally inverts range.
    if (limits_.minPosition > limits_.maxPosition) {
        std::swap(limits_.minPosition, limits_.maxPosition);
    }
    position_ = std::clamp(position_, limits_.minPosition, limits_.maxPosition);
    target_ = std::clamp(target_, limits_.minPosition, limits_.maxPosition);
}

void MotorModel::reset(double position, double velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    position_ = std::clamp(position, limits_.minPosition, limits_.maxPosition);
    velocity_ = velocity;
    acceleration_ = 0.0;
    target_ = position_;
    mode_ = ControlMode::Position;
    hasLastStepTime_ = false;
}

void MotorModel::setPositionTarget(double position) {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = ControlMode::Position;
    target_ = std::clamp(position, limits_.minPosition, limits_.maxPosition);
}

void MotorModel::setVelocityTarget(double velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = ControlMode::Velocity;
    target_ = clampWithSign(velocity, limits_.maxVelocity);
}

MotorModel::State MotorModel::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {position_, velocity_, acceleration_};
}

MotorModel::State MotorModel::step() {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasLastStepTime_) {
        // First tick initializes the time base without moving the model.
        lastStepTime_ = now;
        hasLastStepTime_ = true;
        acceleration_ = 0.0;
        return {position_, velocity_, acceleration_};
    }

    double dtSeconds = std::chrono::duration<double>(now - lastStepTime_).count();
    lastStepTime_ = now;
    if (dtSeconds <= 0.0) {
        // Prevent divisions by zero and unstable integration when timer resolution is coarse.
        dtSeconds = 0.001;
    }

    double desiredVelocity = 0.0;
    double previousPositionError = 0.0;
    if (mode_ == ControlMode::Position) {
        const double error = target_ - position_;
        previousPositionError = error;
        if (std::abs(error) < 1e-4 && std::abs(velocity_) < 1e-3) {
            // Snap to the exact target once close enough to avoid endless micro-oscillations.
            position_ = target_;
            velocity_ = 0.0;
            acceleration_ = 0.0;
            return {position_, velocity_, acceleration_};
        }

        // Position mode uses a trapezoidal profile:
        // accelerate up to max velocity, cruise, then brake when the remaining distance
        // equals the stopping distance at current speed.
        const double direction = (error >= 0.0) ? 1.0 : -1.0;
        const double distance = std::abs(error);
        const double accel = std::max(limits_.maxAcceleration, 1e-9);
        const double speedTowardTarget = std::max(0.0, velocity_ * direction);
        const double stoppingDistance = (speedTowardTarget * speedTowardTarget) / (2.0 * accel);
        const bool shouldBrake = distance <= stoppingDistance;
        desiredVelocity = shouldBrake ? 0.0 : (direction * limits_.maxVelocity);
    } else { // Velocity control
        desiredVelocity = target_;
    }

    const double maxDelta = limits_.maxAcceleration * dtSeconds;
    // Apply acceleration limit as a bounded change in velocity per step.
    const double velDelta = clampWithSign(desiredVelocity - velocity_, maxDelta);
    acceleration_ = velDelta / dtSeconds;
    velocity_ += velDelta;
    position_ += velocity_ * dtSeconds;
    if (mode_ == ControlMode::Position) {
        const double newError = target_ - position_;
        if ((previousPositionError > 0.0 && newError <= 0.0) ||
            (previousPositionError < 0.0 && newError >= 0.0)) {
            // If integration passes the target between two ticks, stop exactly on target
            // to avoid oscillatory sign flips in the following steps.
            position_ = target_;
            velocity_ = 0.0;
            acceleration_ = 0.0;
        }
    }
    // Clamp to mechanical limits; stop when hitting bounds.
    if (position_ < limits_.minPosition) {
        position_ = limits_.minPosition;
        if (velocity_ < 0.0) {
            velocity_ = 0.0;
            acceleration_ = 0.0;
        }
    } else if (position_ > limits_.maxPosition) {
        position_ = limits_.maxPosition;
        if (velocity_ > 0.0) {
            velocity_ = 0.0;
            acceleration_ = 0.0;
        }
    }
    return {position_, velocity_, acceleration_};
}
