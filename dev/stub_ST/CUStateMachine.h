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

#pragma once

#include <mutex>
#include <string>
#include <functional>

/**
 * @brief Models the turret state machine from the provided diagram
 *        (OFF -> INIT -> SAFE -> OPERATIONAL/MAINTENANCE/FAIL).
 *
 *  This class is thread-safe for concurrent commands thanks to the internal mutex.
 */
class CUStateMachine {
public:
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

    enum class OperationalMode {
        Manual,
        Calibration,
        Automatic
    };

    CUStateMachine();

    State state() const;
    OperationalMode operationalMode() const;
    std::string stateName() const;

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
