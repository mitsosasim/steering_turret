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

// TurretSimulator.h - Az/El turret model with SUT axes.
#pragma once

#include "MotorModel.h"
#include "StubStateMachine.h"
#include "st_com.h"

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

class TurretSimulator {
public:
    // mainAxes: ST azimuth/elevation limits. sutAxes: 8 SUTs * 2 axes.
    TurretSimulator(const std::array<MotorModel::Limits, 2>& mainAxes,
                    const std::array<MotorModel::Limits, 16>& sutAxes);
    ~TurretSimulator();

    // Starts/stops the periodic integrator thread.
    void start();
    void stop();

    // Main turret command interface (absolute position or velocity targets).
    void setMainAzPosition(float az);
    void setMainAzVelocity(float azRate);
    void setMainElPosition(float el);
    void setMainElVelocity(float elRate);

    // SUT command interface. sutId is 1..8.
    void setSutAzPosition(size_t sutId, float az);
    void setSutAzVelocity(size_t sutId, float azRate);
    void setSutElPosition(size_t sutId, float el);
    void setSutElVelocity(size_t sutId, float elRate);

   
    // Builds high/low telemetry payloads from current simulated state.
    st_protocol::TelemetryHPayload buildTelemetryH(const StubStateMachine& fsm) const;
    st_protocol::TelemetryLPayload buildTelemetryL(const StubStateMachine& fsm) const;
    // Snapshot getters used by command handlers and telemetry builders.
    std::array<float, 2> mainPosition() const;
    std::array<float, 16> sutPositions() const;

    // Returns simulator start timestamp for uptime calculations.
    std::chrono::steady_clock::time_point startTime() const;

private:
    // Periodically advances all motor models.
    void integrateLoop();
    std::array<float, 2> mainVelocity() const;
    std::array<float, 2> mainAcceleration() const;
    std::array<float, 16> sutVelocities() const;
    std::array<float, 16> sutAccelerations() const;

private:
    MotorModel stAz_;
    MotorModel stEl_;
    std::array<MotorModel, 16> sutAxes_;

    std::array<uint8_t, 9> memorySlots_{}; // index 1..8 used for SUTs
    float cameraHfov_{45.0f};
    bool joystickEnabled_{false};
    uint8_t trackSource_{0};

    std::chrono::steady_clock::time_point startTime_{std::chrono::steady_clock::now()};
    std::atomic<bool> running_{false};
    std::thread integratorThread_;
    mutable std::mutex mutex_;
};
