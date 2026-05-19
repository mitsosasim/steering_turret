/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Action queue FIFO, stream-speed coalescing, and action-builder regression tests.
 *
 * Design notes:
 *   Protects command-thread to CAN-worker contract: ordering, wakeup signaling, and freshest speed semantics.
 */

#include "stc_actions.h"

#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void assert_true(int cond, const char* msg) {
  if (!cond) {
    fprintf(stderr, "FAIL: %s\n", msg);
    exit(1);
  }
}

/* Verifies FIFO ordering so command execution order from TCP thread is preserved by CAN worker. */
static void test_queue_fifo(void) {
  actionq_t q;
  actionq_init(&q);

  stc_action_t a0;
  stc_action_make_simple(&a0, ACTION_RESET, 0u);
  stc_action_t a1;
  stc_action_make_set_state(&a1, 3);

  assert_true(actionq_push(&q, &a0) == 0, "push a0");
  assert_true(actionq_push(&q, &a1) == 0, "push a1");

  stc_action_t out;
  assert_true(actionq_try_peek(&q, &out) == 1, "peek a0");
  assert_true(out.type == ACTION_RESET, "a0 order");
  assert_true(actionq_drop_head(&q) == 1, "drop a0");

  assert_true(actionq_try_peek(&q, &out) == 1, "peek a1");
  assert_true(out.type == ACTION_SET_STATE, "a1 order");
  assert_true(out.u.new_state == 3, "a1 payload");
}


/* Verifies stream-speed coalescing and eventfd wakeup to prevent stale speed replay or missed worker wakeups. */
static void test_stream_speed_coalescing_and_wakeup(void) {
  actionq_t q;
  actionq_init(&q);

  stc_action_t act;
  stc_action_make_simple(&act, ACTION_RESET, 0x1u);
  assert_true(actionq_push(&q, &act) == 0, "push reset");

  double speed0[STC_AXIS_COUNT] = {0};
  speed0[0] = 1.0;
  stc_action_make_stream_speed(&act, 0x1u, speed0);
  assert_true(actionq_push(&q, &act) == 0, "push speed0");

  double speed1[STC_AXIS_COUNT] = {0};
  speed1[0] = 2.0;
  stc_action_make_stream_speed(&act, 0x1u, speed1);
  assert_true(actionq_push(&q, &act) == 0, "push speed1");

  double target[STC_AXIS_COUNT] = {0};
  target[0] = 5.0;
  stc_action_make_pp_move(&act, 0x1u, target);
  assert_true(actionq_push(&q, &act) == 0, "push pp");

  double speed2[STC_AXIS_COUNT] = {0};
  speed2[0] = 3.0;
  stc_action_make_stream_speed(&act, 0x1u, speed2);
  assert_true(actionq_push(&q, &act) == 0, "push speed2");

  stc_action_t out;
  assert_true(actionq_try_peek(&q, &out) == 1, "peek coalesced reset");
  assert_true(out.type == ACTION_RESET, "reset still first");
  assert_true(actionq_drop_head(&q) == 1, "drop reset");

  assert_true(actionq_try_peek(&q, &out) == 1, "peek coalesced pp");
  assert_true(out.type == ACTION_PP_MOVE, "pp remains before latest speed");
  assert_true(actionq_drop_head(&q) == 1, "drop pp");

  assert_true(actionq_try_peek(&q, &out) == 1, "peek coalesced speed");
  assert_true(out.type == ACTION_STREAM_SPEED, "latest speed kept");
  assert_true(out.u.speed_deg_s[0] == 3.0, "latest speed value kept");
  assert_true(actionq_drop_head(&q) == 1, "drop speed");
  assert_true(actionq_try_peek(&q, &out) == 0, "queue empty after coalesced sequence");

  int wake_fd = actionq_wakeup_fd(&q);
  assert_true(wake_fd >= 0, "eventfd available");

  struct pollfd pfd = {.fd = wake_fd, .events = POLLIN, .revents = 0};
  assert_true(poll(&pfd, 1, 0) == 1, "wakeup fd readable after pushes");
  actionq_consume_wakeup(&q);
  pfd.revents = 0;
  assert_true(poll(&pfd, 1, 0) == 0, "wakeup fd drained");
}

/* Verifies action builder helpers populate type/mask/payload fields expected by runtime dispatch. */
static void test_action_builders(void) {
  stc_action_t act;
  double target_deg[STC_AXIS_COUNT] = {0};
  double speed_deg_s[STC_AXIS_COUNT] = {0};

  target_deg[0] = 12.5;
  target_deg[2] = -3.25;
  stc_action_make_pp_move(&act, (1u << 0) | (1u << 2), target_deg);
  assert_true(act.type == ACTION_PP_MOVE, "pp type");
  assert_true(act.axes_mask == ((1u << 0) | (1u << 2)), "pp mask");
  assert_true(act.u.target_deg[0] == 12.5, "pp axis0");
  assert_true(act.u.target_deg[2] == -3.25, "pp axis2");

  speed_deg_s[1] = 4.0;
  stc_action_make_stream_speed(&act, (1u << 1), speed_deg_s);
  assert_true(act.type == ACTION_STREAM_SPEED, "speed type");
  assert_true(act.axes_mask == (1u << 1), "speed mask");
  assert_true(act.u.speed_deg_s[1] == 4.0, "speed axis1");
}

int main(void) {
  test_queue_fifo();
  test_action_builders();
  test_stream_speed_coalescing_and_wakeup();
  printf("OK\n");
  return 0;
}
