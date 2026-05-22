/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Runtime state-machine implementation for SAFE/OPER/FAIL control gating.
 *
 * Design notes:
 *   Centralizes legal transitions for startup, safety, and fault events.
 *   Motion/stream permissions are explicit helpers used by command validation.
 *   Safety-critical decisions are isolated here for deterministic testing.
 */

#include "stc_state.h"

/* STC operational state machine:
 * - SAFE: movement/streaming inhibited while control and diagnostics remain active.
 * - OPER: nominal motion-enabled state for accepted command execution.
 * - FAIL: critical-fault containment state requiring explicit recovery path.
 * Invalid transitions return failure so callers never silently change state.
 */

/** Resolve one event-driven transition; returns -1 on invalid transition. */
int stc_state_transition(stc_state_t current, stc_event_t evt, stc_state_t* next) {
  /* All legal transitions are centralized here to prevent split policy logic. */
  if (!next) return -1;
  *next = current;

  if (evt == EVT_FAULT_CRITICAL) {
    *next = STC_STATE_FAIL;
    return 0;
  }

  if ((evt == EVT_ENTER_SAFE && current == STC_STATE_SAFE) ||
      (evt == EVT_ENTER_OPER && current == STC_STATE_OPER)) {
    return 0;
  }

  switch (current) {
    case STC_STATE_INIT:
      if (evt == EVT_PBIT_OK) {
        *next = STC_STATE_INIT_DONE;
        return 0;
      }
      if (evt == EVT_PBIT_FAIL) {
        *next = STC_STATE_FAIL;
        return 0;
      }
      break;
    case STC_STATE_INIT_DONE:
      if (evt == EVT_STORE) {
        *next = STC_STATE_STORE;
        return 0;
      }
      if (evt == EVT_ENTER_SAFE) {
        *next = STC_STATE_SAFE;
        return 0;
      }
      break;
    case STC_STATE_SAFE:
      if (evt == EVT_ENTER_OPER) {
        *next = STC_STATE_OPER;
        return 0;
      }
      if (evt == EVT_SELF_LEV) {
        *next = STC_STATE_SELF_LEVELLING;
        return 0;
      }
      if (evt == EVT_IBIT) {
        *next = STC_STATE_IBIT_RUNNING;
        return 0;
      }
      if (evt == EVT_STORE) {
        *next = STC_STATE_STORE;
        return 0;
      }
      break;
    case STC_STATE_OPER:
      if (evt == EVT_STORE) {
        *next = STC_STATE_STORE;
        return 0;
      }
      if (evt == EVT_ENTER_SAFE) {
        *next = STC_STATE_SAFE;
        return 0;
      }
      if (evt == EVT_SELF_LEV) {
        *next = STC_STATE_SELF_LEVELLING;
        return 0;
      }
      if (evt == EVT_IBIT) {
        *next = STC_STATE_IBIT_RUNNING;
        return 0;
      }
      break;
    case STC_STATE_SELF_LEVELLING:
      if (evt == EVT_ENTER_SAFE) {
        *next = STC_STATE_SAFE;
        return 0;
      }
      if (evt == EVT_SELF_LEV_DONE) {
        *next = STC_STATE_OPER;
        return 0;
      }
      break;
    case STC_STATE_STORE:
      if (evt == EVT_ENTER_SAFE || evt == EVT_STORE_OK) {
        *next = STC_STATE_SAFE;
        return 0;
      }
      if (evt == EVT_STORE_FAIL) {
        *next = STC_STATE_FAIL;
        return 0;
      }
      break;
    case STC_STATE_IBIT_RUNNING:
      if (evt == EVT_ENTER_SAFE) {
        *next = STC_STATE_SAFE;
        return 0;
      }
      if (evt == EVT_IBIT_COMPLETE_OK) {
        *next = STC_STATE_OPER;
        return 0;
      }
      if (evt == EVT_IBIT_COMPLETE_FAIL) {
        *next = STC_STATE_FAIL;
        return 0;
      }
      break;
    case STC_STATE_FAIL:
      if (evt == EVT_PBIT_OK) {
        *next = STC_STATE_INIT_DONE;
        return 0;
      }
      if (evt == EVT_IBIT) {
        *next = STC_STATE_IBIT_RUNNING;
        return 0;
      }
      break;
    default:
      break;
  }

  return -1;
}

/** Motion commands are permitted only in OPER state. */
int stc_state_can_move(stc_state_t state) {
  /* Primary motion gate consumed by command-validation/execution paths. */
  return state == STC_STATE_OPER;
}

/** Streamed-speed control is permitted only in OPER state. */
int stc_state_can_stream(stc_state_t state) {
  return state == STC_STATE_OPER;
}

const char* stc_state_name(stc_state_t state) {
  switch (state) {
    case STC_STATE_INIT: return "INIT";
    case STC_STATE_INIT_DONE: return "INIT_DONE";
    case STC_STATE_SAFE: return "SAFE";
    case STC_STATE_OPER: return "OPER";
    case STC_STATE_SELF_LEVELLING: return "SELF_LEVELLING";
    case STC_STATE_IBIT_RUNNING: return "IBIT_RUNNING";
    case STC_STATE_STORE: return "STORE";
    case STC_STATE_FAIL: return "FAIL";
    default: return "UNKNOWN";
  }
}
