#include "stc_state.h"
#include "stc_faults.h"
#include "stc_diag.h"

#include <assert.h>
#include <stdio.h>

static void expect_transition(stc_state_t from, stc_event_t evt, stc_state_t to) {
  stc_state_t next;
  assert(stc_state_transition(from, evt, &next) == 0);
  assert(next == to);
}

static void expect_invalid(stc_state_t from, stc_event_t evt) {
  stc_state_t next = STC_STATE_INIT;
  assert(stc_state_transition(from, evt, &next) != 0);
}

int main(void) {
  expect_transition(STC_STATE_INIT, EVT_PBIT_OK, STC_STATE_INIT_DONE);
  expect_transition(STC_STATE_INIT, EVT_PBIT_FAIL, STC_STATE_FAIL);
  expect_transition(STC_STATE_INIT_DONE, EVT_STORE, STC_STATE_STORE);
  expect_transition(STC_STATE_INIT_DONE, EVT_ENTER_SAFE, STC_STATE_SAFE);
  expect_transition(STC_STATE_SAFE, EVT_ENTER_OPER, STC_STATE_OPER);
  expect_transition(STC_STATE_SAFE, EVT_SELF_LEV, STC_STATE_SELF_LEVELLING);
  expect_transition(STC_STATE_SAFE, EVT_IBIT, STC_STATE_IBIT_RUNNING);
  expect_transition(STC_STATE_SAFE, EVT_STORE, STC_STATE_STORE);
  expect_transition(STC_STATE_OPER, EVT_STORE, STC_STATE_STORE);
  expect_transition(STC_STATE_OPER, EVT_ENTER_SAFE, STC_STATE_SAFE);
  expect_transition(STC_STATE_OPER, EVT_SELF_LEV, STC_STATE_SELF_LEVELLING);
  expect_transition(STC_STATE_OPER, EVT_IBIT, STC_STATE_IBIT_RUNNING);
  expect_transition(STC_STATE_SELF_LEVELLING, EVT_ENTER_SAFE, STC_STATE_SAFE);
  expect_transition(STC_STATE_IBIT_RUNNING, EVT_ENTER_SAFE, STC_STATE_SAFE);
  expect_transition(STC_STATE_IBIT_RUNNING, EVT_IBIT_COMPLETE_OK, STC_STATE_OPER);
  expect_transition(STC_STATE_IBIT_RUNNING, EVT_IBIT_COMPLETE_FAIL, STC_STATE_FAIL);
  expect_transition(STC_STATE_SAFE, EVT_IBIT, STC_STATE_IBIT_RUNNING);
  expect_transition(STC_STATE_IBIT_RUNNING, EVT_IBIT_COMPLETE_OK, STC_STATE_OPER);
  expect_transition(STC_STATE_OPER, EVT_IBIT, STC_STATE_IBIT_RUNNING);
  expect_transition(STC_STATE_IBIT_RUNNING, EVT_IBIT_COMPLETE_FAIL, STC_STATE_FAIL);
  expect_transition(STC_STATE_STORE, EVT_STORE_OK, STC_STATE_SAFE);
  expect_transition(STC_STATE_STORE, EVT_STORE_FAIL, STC_STATE_FAIL);
  expect_transition(STC_STATE_FAIL, EVT_IBIT, STC_STATE_IBIT_RUNNING);

  for (int s = STC_STATE_INIT; s <= STC_STATE_FAIL; ++s) {
    expect_transition((stc_state_t)s, EVT_FAULT_CRITICAL, STC_STATE_FAIL);
  }

  expect_invalid(STC_STATE_INIT, EVT_ENTER_OPER);
  expect_invalid(STC_STATE_INIT_DONE, EVT_IBIT);
  expect_invalid(STC_STATE_FAIL, EVT_ENTER_SAFE);

  assert(stc_state_can_move(STC_STATE_OPER));
  assert(!stc_state_can_move(STC_STATE_SAFE));
  assert(stc_state_can_stream(STC_STATE_OPER));
  assert(!stc_state_can_stream(STC_STATE_STORE));

  assert(stc_detect_ds402_fault(0x0008));
  assert(stc_detect_ds402_fault(0xFFFF));
  assert(!stc_detect_ds402_fault(0x0000));

  assert(!stc_detect_can_timeout(1000, 10, 1000));
  assert(stc_detect_can_timeout(1010, 10, 1000));
  assert(!stc_detect_can_timeout(99, 100, 10));

  stc_diag_set_pbit_result(true);
  assert(run_pbit_startup());
  expect_transition(STC_STATE_INIT, run_pbit_startup() ? EVT_PBIT_OK : EVT_PBIT_FAIL, STC_STATE_INIT_DONE);
  stc_diag_set_pbit_result(false);
  assert(!run_pbit_startup());
  expect_transition(STC_STATE_INIT, run_pbit_startup() ? EVT_PBIT_OK : EVT_PBIT_FAIL, STC_STATE_FAIL);

  int dummy = 0;
  shared_data_t* shared = (shared_data_t*)&dummy;
  stc_diag_set_cbit_critical(shared, false);
  assert(run_cbit_cycle(shared) == STC_DIAG_HEALTH_OK);
  stc_diag_set_cbit_critical(shared, true);
  assert(run_cbit_cycle(shared) == STC_DIAG_HEALTH_CRITICAL_FAULT);

  stc_diag_set_ibit_result(true);
  assert(run_ibit_sequence(shared));
  expect_transition(STC_STATE_IBIT_RUNNING,
                    run_ibit_sequence(shared) ? EVT_IBIT_COMPLETE_OK : EVT_IBIT_COMPLETE_FAIL,
                    STC_STATE_OPER);
  stc_diag_set_ibit_result(false);
  assert(!run_ibit_sequence(shared));
  expect_transition(STC_STATE_IBIT_RUNNING,
                    run_ibit_sequence(shared) ? EVT_IBIT_COMPLETE_OK : EVT_IBIT_COMPLETE_FAIL,
                    STC_STATE_FAIL);

  puts("stc_state_test: PASS");
  return 0;
}
