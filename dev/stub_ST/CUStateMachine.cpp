/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   control-unit state logic for the legacy ST stub simulator.
 *
 * Design notes:
 *   emulates controller-side state transitions for simulator-driven protocol testing.
 *   Owns only legacy stub runtime state semantics and not production CU behavior.
 *   Transition APIs are mutex-guarded for thread-safe command/telemetry observers.
 */

#include "CUStateMachine.h"

#include "mod_LOG.h"

CUStateMachine::CUStateMachine()
    : state_(State::Off),
      opMode_(OperationalMode::Manual) {}

CUStateMachine::State CUStateMachine::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

CUStateMachine::OperationalMode CUStateMachine::operationalMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return opMode_;
}

std::string CUStateMachine::stateName() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return toString(state_);
}

void CUStateMachine::powerOn() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::Off) {
        transitionTo(State::Init, "POWER ON");
    } else {
        mod_LOG::warning(std::string("[FSM] Ignoro POWER ON: stato attuale ") + toString(state_));
    }
}

void CUStateMachine::powerOff() {
    std::lock_guard<std::mutex> lock(mutex_);
    transitionTo(State::Off, "POWER OFF");
}

void CUStateMachine::reportPbit(bool ok) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::Init) {
        mod_LOG::warning(std::string("[FSM] Il PBIT è valido solo in INIT. Stato attuale: ") + toString(state_));
        return;
    }
    if (ok) {
        transitionTo(State::Safe, "PBIT OK");
    } else {
        transitionTo(State::FailIdle, "PBIT NOT OK");
    }
}

void CUStateMachine::reportCbit(bool ok) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::Safe && !isOperational(state_)) {
        mod_LOG::warning(std::string("[FSM] Il CBIT è previsto solo in SAFE/OPERATIONAL. Stato attuale: ") +
                         toString(state_));
        return;
    }
    if (ok) {
        mod_LOG::info(std::string("[FSM] CBIT OK in stato ") + toString(state_));
    } else {
        transitionTo(State::FailIdle, "CBIT NOT OK");
    }
}

void CUStateMachine::armOperational() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::Safe) {
        mod_LOG::warning(std::string("[FSM] L'ARM è consentito solo in SAFE. Stato attuale: ") + toString(state_));
        return;
    }
    transitionTo(State::OperationalManual, "ARM (MAN mode)");
    opMode_ = OperationalMode::Manual;
}

void CUStateMachine::goSafe() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::Off) {
        mod_LOG::info("[FSM] Già in OFF, SAFE ignorato.");
        return;
    }
    transitionTo(State::Safe, "SAFE command");
}

void CUStateMachine::goMaintenance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::Safe && !isOperational(state_)) {
        mod_LOG::warning(std::string("[FSM] GO MAINT valido solo da SAFE/OPERATIONAL. Stato attuale: ") +
                         toString(state_));
        return;
    }
    transitionTo(State::MaintenanceIdle, "GO MAINT");
}

void CUStateMachine::startMaintenanceIbit() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::MaintenanceIdle) {
        mod_LOG::warning(std::string("[FSM] IBIT di manutenzione parte solo da MAINT/IDLE. Stato attuale: ") +
                         toString(state_));
        return;
    }
    transitionTo(State::MaintenanceIbitRun, "IBIT RUN");
}

void CUStateMachine::finishMaintenanceIbit(bool ok) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::MaintenanceIbitRun) {
        mod_LOG::warning(std::string("[FSM] Nessun IBIT di manutenzione in corso. Stato attuale: ") +
                         toString(state_));
        return;
    }
    if (ok) {
        transitionTo(State::MaintenanceIdle, "IBIT OK");
    } else {
        transitionTo(State::FailIdle, "IBIT NOT OK (maintenance)");
    }
}

void CUStateMachine::startSelfLevelling() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::MaintenanceIdle) {
        mod_LOG::warning(std::string("[FSM] La self-levelling parte solo da MAINT/IDLE. Stato attuale: ") +
                         toString(state_));
        return;
    }
    transitionTo(State::MaintenanceSelfLevelling, "Self Levelling");
}

void CUStateMachine::finishSelfLevelling(bool ok) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::MaintenanceSelfLevelling) {
        mod_LOG::warning(std::string("[FSM] Nessuna self-levelling in corso. Stato attuale: ") +
                         toString(state_));
        return;
    }
    if (ok) {
        transitionTo(State::MaintenanceIdle, "Self Levelling OK");
    } else {
        transitionTo(State::FailIdle, "Self Levelling Error");
    }
}

void CUStateMachine::setOperationalMode(OperationalMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isOperational(state_)) {
        mod_LOG::warning(std::string("[FSM] Il cambio mode è disponibile solo in OPERATIONAL. Stato attuale: ") +
                         toString(state_));
        return;
    }
    State target = State::OperationalManual;
    switch (mode) {
    case OperationalMode::Manual:
        target = State::OperationalManual;
        break;
    case OperationalMode::Calibration:
        target = State::OperationalCalibration;
        break;
    case OperationalMode::Automatic:
        target = State::OperationalAuto;
        break;
    }
    opMode_ = mode;
    transitionTo(target, "Set Mode -> " + std::string(toString(mode)));
}

void CUStateMachine::signalFailure(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    transitionTo(State::FailIdle, "FAIL: " + reason);
}

void CUStateMachine::startFailureIbit() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::FailIdle) {
        mod_LOG::warning(std::string("[FSM] L'IBIT di fail parte solo da FAIL/IDLE. Stato attuale: ") +
                         toString(state_));
        return;
    }
    transitionTo(State::FailIbitRun, "FAIL IBIT RUN");
}

void CUStateMachine::finishFailureIbit(bool ok) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::FailIbitRun) {
        mod_LOG::warning(std::string("[FSM] Nessun FAIL IBIT in corso. Stato attuale: ") +
                         toString(state_));
        return;
    }
    if (ok) {
        transitionTo(State::Safe, "FAIL -> SAFE (IBIT OK)");
    } else {
        transitionTo(State::FailIdle, "IBIT NOT OK (fail loop)");
    }
}

void CUStateMachine::transitionTo(State newState, const std::string& cause) {
    if (state_ == newState) {
        mod_LOG::info(std::string("[FSM] Rimango in ") + toString(state_) + " (" + cause + ")");
        return;
    }
    mod_LOG::info(std::string("[FSM] ") + toString(state_) + " -> " + toString(newState) + " [" + cause + "]");
    auto cb = stateChangeCallback_;
    state_ = newState;
    if (isOperational(newState)) {
        switch (newState) {
        case State::OperationalManual:
            opMode_ = OperationalMode::Manual;
            break;
        case State::OperationalCalibration:
            opMode_ = OperationalMode::Calibration;
            break;
        case State::OperationalAuto:
            opMode_ = OperationalMode::Automatic;
            break;
        default:
            break;
        }
    }
    if (cb) {
        cb(newState);
    }
}

bool CUStateMachine::isOperational(State s) const {
    return s == State::OperationalManual ||
           s == State::OperationalCalibration ||
           s == State::OperationalAuto;
}

const char* CUStateMachine::toString(State s) {
    switch (s) {
    case State::Off:
        return "OFF";
    case State::Init:
        return "INIT";
    case State::Safe:
        return "SAFE";
    case State::FailIdle:
        return "FAIL/IDLE";
    case State::FailIbitRun:
        return "FAIL/IBIT RUN";
    case State::MaintenanceIdle:
        return "MAINT/IDLE";
    case State::MaintenanceIbitRun:
        return "MAINT/IBIT RUN";
    case State::MaintenanceSelfLevelling:
        return "MAINT/SELF LEVELLING";
    case State::OperationalManual:
        return "OPER/MANUAL";
    case State::OperationalCalibration:
        return "OPER/CAL";
    case State::OperationalAuto:
        return "OPER/AUTO";
    default:
        return "UNKNOWN";
    }
}

const char* CUStateMachine::toString(OperationalMode mode) {
    switch (mode) {
    case OperationalMode::Manual:
        return "MANUAL";
    case OperationalMode::Calibration:
        return "CAL";
    case OperationalMode::Automatic:
        return "AUTO";
    default:
        return "UNKNOWN";
    }
}

void CUStateMachine::setStateChangeCallback(std::function<void(State)> cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    stateChangeCallback_ = std::move(cb);
}
