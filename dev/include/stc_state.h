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

#ifndef STC_STATE_H
#define STC_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  STC_STATE_INIT = 0,
  STC_STATE_INIT_DONE,
  STC_STATE_SAFE,
  STC_STATE_OPER,
  STC_STATE_SELF_LEVELLING,
  STC_STATE_IBIT_RUNNING,
  STC_STATE_STORE,
  STC_STATE_FAIL
} stc_state_t;

typedef enum {
  EVT_PBIT_OK = 0,
  EVT_PBIT_FAIL,
  EVT_IBIT_COMPLETE_OK,
  EVT_IBIT_COMPLETE_FAIL,
  EVT_STORE_OK,
  EVT_STORE_FAIL,
  EVT_ENTER_SAFE,
  EVT_ENTER_OPER,
  EVT_SELF_LEV,
  EVT_SELF_LEV_DONE,
  EVT_IBIT,
  EVT_STORE,
  EVT_FAULT_CRITICAL
} stc_event_t;

#define EVT_IBIT_OK EVT_IBIT_COMPLETE_OK
#define EVT_IBIT_FAIL EVT_IBIT_COMPLETE_FAIL

int stc_state_transition(stc_state_t current, stc_event_t evt, stc_state_t* next);
int stc_state_can_move(stc_state_t state);
int stc_state_can_stream(stc_state_t state);

#ifdef __cplusplus
}
#endif

#endif
