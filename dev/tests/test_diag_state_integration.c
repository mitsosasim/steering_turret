/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Integration tests linking diagnostics outcomes to state-machine transitions.
 *
 * Design notes:
 *   Protects SAFE/FAIL behavior so runtime gating follows reported health conditions.
 */

#include "stc_diag.h"
#include "stc_state.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef NDEBUG
#define TEST_ASSERT_OR_ABORT(cond) do { if (!(cond)) { fprintf(stderr, "CHECK failed: %s\n", #cond); abort(); } } while (0)
#else
#define TEST_ASSERT_OR_ABORT(cond) assert(cond)
#endif

static stc_state_t apply(stc_state_t current, stc_event_t evt) {
  stc_state_t next = STC_STATE_INIT;
  TEST_ASSERT_OR_ABORT(stc_state_transition(current, evt, &next) == 0);
  return next;
}

int main(void) {
  stc_state_t state = STC_STATE_INIT;

  stc_diag_set_pbit_result(true);
  state = apply(state, run_pbit_startup() ? EVT_PBIT_OK : EVT_PBIT_FAIL);
  assert(state == STC_STATE_INIT_DONE);

  state = apply(state, EVT_ENTER_SAFE);
  state = apply(state, EVT_ENTER_OPER);
  assert(state == STC_STATE_OPER);

  int fake_shared = 0;
  shared_data_t* shared = (shared_data_t*)&fake_shared;

  stc_diag_set_cbit_critical(shared, false);
  assert(run_cbit_cycle(shared) == STC_DIAG_HEALTH_OK);

  stc_diag_set_cbit_critical(shared, true);
  if (run_cbit_cycle(shared) == STC_DIAG_HEALTH_CRITICAL_FAULT) {
    state = apply(state, EVT_FAULT_CRITICAL);
  }
  assert(state == STC_STATE_FAIL);

  state = apply(state, EVT_IBIT);
  assert(state == STC_STATE_IBIT_RUNNING);

  stc_diag_set_ibit_result(true);
  state = apply(state, run_ibit_sequence(shared) ? EVT_IBIT_COMPLETE_OK : EVT_IBIT_COMPLETE_FAIL);
  assert(state == STC_STATE_OPER);

  state = apply(state, EVT_IBIT);
  stc_diag_set_ibit_result(false);
  state = apply(state, run_ibit_sequence(shared) ? EVT_IBIT_COMPLETE_OK : EVT_IBIT_COMPLETE_FAIL);
  assert(state == STC_STATE_FAIL);

  puts("test_diag_state_integration: PASS");
  return 0;
}
