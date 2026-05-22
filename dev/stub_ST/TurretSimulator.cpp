/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   legacy turret simulator behavior and telemetry generation.
 *
 * Design notes:
 *   combines state logic and motor models to emulate ST-side motion responses.
 *   Owns integrator-thread lifecycle and telemetry snapshot composition for stub runtime.
 *   Assumes functional protocol timing fidelity rather than hardware-real-time fidelity.
 */

// TurretSimulator.cpp - kinematic simulation and telemetry builder.
#include "TurretSimulator.h"

#include "mod_LOG.h"

#include <cstring>

namespace {
constexpr size_t kSutCount = 8;
constexpr size_t kAxesPerSut = 2;
// Integrator tick period used for all axis updates.
constexpr double kIntegrationPeriodSeconds = 0.005;
} // namespace

TurretSimulator::TurretSimulator(const std::array<MotorModel::Limits, 2>& mainAxes,
                                 const std::array<MotorModel::Limits, 16>& sutAxes)
    : stAz_(mainAxes[0]),
      stEl_(mainAxes[1]) {
    // Initialize per-SUT axis limits from configuration/defaults.
    for (size_t i = 0; i < sutAxes_.size(); ++i) {
        sutAxes_[i].setLimits(sutAxes[i]);
    }
}

TurretSimulator::~TurretSimulator() {
    stop();
}

void TurretSimulator::start() {
    if (running_.exchange(true)) {
        // Already running.
        return;
    }
    integratorThread_ = std::thread(&TurretSimulator::integrateLoop, this);
}

void TurretSimulator::stop() {
    if (!running_.exchange(false)) {
        // Already stopped.
        return;
    }
    if (integratorThread_.joinable()) {
        integratorThread_.join();
    }
}

void TurretSimulator::setMainAzPosition(float az) {
    stAz_.setPositionTarget(static_cast<double>(az));
}

void TurretSimulator::setMainAzVelocity(float azRate) {
    stAz_.setVelocityTarget(static_cast<double>(azRate));
}

void TurretSimulator::setMainElPosition(float el) {
    stEl_.setPositionTarget(static_cast<double>(el));
}

void TurretSimulator::setMainElVelocity(float elRate) {
    stEl_.setVelocityTarget(static_cast<double>(elRate));
}

void TurretSimulator::setSutAzPosition(size_t sutId, float az) {
    if (sutId < 1 || sutId > kSutCount) {
        // Ignore invalid SUT indices to keep command path resilient.
        return;
    }
    const size_t idx = (sutId - 1U) * kAxesPerSut;
    sutAxes_[idx].setPositionTarget(static_cast<double>(az));
}

void TurretSimulator::setSutAzVelocity(size_t sutId, float azRate) {
    if (sutId < 1 || sutId > kSutCount) {
        return;
    }
    const size_t idx = (sutId - 1U) * kAxesPerSut;
    sutAxes_[idx].setVelocityTarget(static_cast<double>(azRate));
}

void TurretSimulator::setSutElPosition(size_t sutId, float el) {
    if (sutId < 1 || sutId > kSutCount) {
        return;
    }
    const size_t idx = (sutId - 1U) * kAxesPerSut + 1;
    sutAxes_[idx].setPositionTarget(static_cast<double>(el));
}

void TurretSimulator::setSutElVelocity(size_t sutId, float elRate) {
    if (sutId < 1 || sutId > kSutCount) {
        return;
    }
    const size_t idx = (sutId - 1U) * kAxesPerSut + 1;
    sutAxes_[idx].setVelocityTarget(static_cast<double>(elRate));
}

std::chrono::steady_clock::time_point TurretSimulator::startTime() const {
    return startTime_;
}

st_protocol::TelemetryHPayload
TurretSimulator::buildTelemetryH(const StubStateMachine& /*fsm*/) const {
    st_protocol::TelemetryHPayload payload{};
    // Use wall-clock epoch timestamp to match ICD message expectations.
    payload.timestamp = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());

    const auto stPos = mainPosition();
    const auto stVel = mainVelocity();
    const auto stAcc = mainAcceleration();
    const auto sutPos = sutPositions();
    const auto sutVel = sutVelocities();
    const auto sutAcc = sutAccelerations();

    // Flatten [ST axes][all SUT axes] into protocol arrays.
    std::memcpy(payload.position.data(), stPos.data(), sizeof(float) * stPos.size());
    std::memcpy(payload.position.data() + stPos.size(), sutPos.data(), sizeof(float) * sutPos.size());

    std::memcpy(payload.rates.data(), stVel.data(), sizeof(float) * stVel.size());
    std::memcpy(payload.rates.data() + stVel.size(), sutVel.data(), sizeof(float) * sutVel.size());

    std::memcpy(payload.acceleration.data(), stAcc.data(), sizeof(float) * stAcc.size());
    std::memcpy(payload.acceleration.data() + stAcc.size(), sutAcc.data(), sizeof(float) * sutAcc.size());
    return payload;
}

st_protocol::TelemetryLPayload
TurretSimulator::buildTelemetryL(const StubStateMachine& fsm) const {
    st_protocol::TelemetryLPayload payload{};
    payload.timestamp = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());

    // Export current FSM state as protocol state byte.
    payload.state = static_cast<uint8_t>(fsm.state());

    const auto now = std::chrono::steady_clock::now();
    payload.uptime = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::seconds>(now - startTime_).count());

    // Environmental values are fixed placeholders in the stub.
    payload.temperature = 25.0f;
    payload.humidity = 50.0f;
    payload.pressure = 1013.0f;

    const auto stPos = mainPosition();
    payload.geoPos.fill(0.0);
    payload.orientationAz = static_cast<double>(stPos[0]);

    payload.motorStatus.fill(0);
    payload.laserStatus.fill(0);
    payload.levelX = 0.0f;
    payload.levelY = 0.0f;
    payload.numErrors = 0;
    payload.errorCodes.clear();

    return payload;
}

void TurretSimulator::integrateLoop() {
    using namespace std::chrono_literals;
    const auto tick = std::chrono::duration<double>(kIntegrationPeriodSeconds);
    while (running_) {
        // Advance all axis models once per tick.
        stAz_.step();
        stEl_.step();
        for (auto& motor : sutAxes_) {
            motor.step();
        }
        std::this_thread::sleep_for(tick);
    }
}

std::array<float, 2> TurretSimulator::mainPosition() const {
    const auto az = stAz_.state();
    const auto el = stEl_.state();
    return {static_cast<float>(az.position), static_cast<float>(el.position)};
}

std::array<float, 2> TurretSimulator::mainVelocity() const {
    const auto az = stAz_.state();
    const auto el = stEl_.state();
    return {static_cast<float>(az.velocity), static_cast<float>(el.velocity)};
}

std::array<float, 2> TurretSimulator::mainAcceleration() const {
    const auto az = stAz_.state();
    const auto el = stEl_.state();
    return {static_cast<float>(az.acceleration), static_cast<float>(el.acceleration)};
}

std::array<float, 16> TurretSimulator::sutPositions() const {
    std::array<float, 16> result{};
    for (size_t i = 0; i < result.size(); ++i) {
        result[i] = static_cast<float>(sutAxes_[i].state().position);
    }
    return result;
}

std::array<float, 16> TurretSimulator::sutVelocities() const {
    std::array<float, 16> result{};
    for (size_t i = 0; i < result.size(); ++i) {
        result[i] = static_cast<float>(sutAxes_[i].state().velocity);
    }
    return result;
}

std::array<float, 16> TurretSimulator::sutAccelerations() const {
    std::array<float, 16> result{};
    for (size_t i = 0; i < result.size(); ++i) {
        result[i] = static_cast<float>(sutAxes_[i].state().acceleration);
    }
    return result;
}
