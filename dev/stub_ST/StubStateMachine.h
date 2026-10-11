/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Simplified ST-side operational state machine for the communication stub.
 *
 * Design notes:
 *   Mirrors INIT/SAFE/OPER/FAIL semantics from the runtime server, with compact
 *   transitions for self-levelling, IBIT, and storage command emulation.
 */

// StubStateMachine.h - Simple FSM for turret stub aligned to CU ICD states.
#pragma once

#include <cstdint>
#include <mutex>
#include <string>

/** @brief Stub-local state machine used to gate accepted simulated commands. */
class StubStateMachine {
public:
    /** @brief ST-visible states encoded into telemetry and command gating logic. */
    /* Main legacy stub state enum owned by command gating and telemetry status reporting. */
    enum class State : uint8_t {
        Init = 0,
        InitDone = 1,
        Safe = 2,
        Oper = 3,
        SelfLevelling = 4,
        IbitRunning = 5,
        Store = 6,
        Fail = 0xFF
    };

    State state() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

    std::string stateName() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return toString(state_);
    }

    // Transition ownership stays inside this class; callers request changes through guarded helpers.
    void setState(State s) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = s;
    }

    bool startSelfLevelling() {
        std::lock_guard<std::mutex> lock(mutex_);
        // Commands are rejected when the machine is in FAIL.
        if (!canCommand()) {
            return false;
        }
        if (state_ == State::Safe || state_ == State::Oper) {
            state_ = State::SelfLevelling;
            return true;
        }
        return false;
    }

    void stopSelfLevelling() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == State::SelfLevelling) {
            state_ = State::Safe;
        }
    }

    bool startIbit() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!canCommand()) {
            return false;
        }
        if (state_ == State::Safe || state_ == State::Oper) {
            state_ = State::IbitRunning;
            return true;
        }
        return false;
    }

    void stopIbit() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == State::IbitRunning) {
            state_ = State::Safe;
        }
    }

    bool goOper() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!canCommand()) {
            return false;
        }
        state_ = State::Oper;
        return true;
    }

    void goSafe() {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = State::Safe;
    }

    bool goStore() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!canCommand()) {
            return false;
        }
        state_ = State::Store;
        return true;
    }

    void fail() {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = State::Fail;
    }

    bool isOperational() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_ == State::Oper;
    }

private:
    // Central gate used by state-changing commands.
    bool canCommand() const { return state_ != State::Fail; }

    // Human-readable state labels used in logs/diagnostics.
    static const char* toString(State s) {
        switch (s) {
        case State::Init: return "INIT";
        case State::InitDone: return "INIT DONE";
        case State::Safe: return "SAFE";
        case State::Oper: return "OPER";
        case State::SelfLevelling: return "SELF LEVELLING";
        case State::IbitRunning: return "IBIT RUNNING";
        case State::Store: return "STORE";
        case State::Fail: return "FAIL";
        default: return "UNKNOWN";
        }
    }

    mutable std::mutex mutex_;
    State state_{State::Init};
};
