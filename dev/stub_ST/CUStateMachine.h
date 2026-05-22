/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   CU-side state model used by the ST communication stub.
 *
 * Design notes:
 *   Encodes operational/maintenance/failure states, validates transitions,
 *   and exposes callback hooks for observing state changes in integration tests.
 */

#pragma once

#include <mutex>
#include <string>
#include <functional>

/** @brief Control-Unit (CU) state machine model used by the stub simulator. */
class CUStateMachine {
public:
    /** @brief CU runtime states including operational submodes and fail substates. */
    enum class State {
        Off,
        Init,
        Safe,
        FailIdle,
        FailIbitRun,
        MaintenanceIdle,
        MaintenanceIbitRun,
        MaintenanceSelfLevelling,
        OperationalManual,
        OperationalCalibration,
        OperationalAuto
    };

    /** @brief Operational control modes allowed while in operational states. */
    enum class OperationalMode {
        Manual,
        Calibration,
        Automatic
    };

    /** @brief Construct state machine in default startup state. */
    CUStateMachine();

    /** @brief Thread-safe read of current CU state for protocol responders. */
    State state() const;
    OperationalMode operationalMode() const;
    std::string stateName() const;

    /** @brief Transition API family used by legacy command handlers to request state changes. */
    void powerOn();
    void powerOff();
    void reportPbit(bool ok);
    void reportCbit(bool ok);
    void armOperational();
    void goSafe();
    void goMaintenance();
    void startMaintenanceIbit();
    void finishMaintenanceIbit(bool ok);
    void startSelfLevelling();
    void finishSelfLevelling(bool ok);
    void setOperationalMode(OperationalMode mode);
    void signalFailure(const std::string& reason);
    void startFailureIbit();
    void finishFailureIbit(bool ok);
    /** @brief Register callback invoked after each accepted state transition. */
    void setStateChangeCallback(std::function<void(State)> cb);

private:
    void transitionTo(State newState, const std::string& cause);
    bool isOperational(State s) const;
    static const char* toString(State s);
    static const char* toString(OperationalMode mode);

private:
    mutable std::mutex mutex_;
    State state_;
    OperationalMode opMode_;
    std::function<void(State)> stateChangeCallback_;
};
