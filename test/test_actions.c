#include "stc_actions.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void assert_true(int cond, const char* msg) {
  if (!cond) {
    fprintf(stderr, "FAIL: %s\n", msg);
    exit(1);
  }
}

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
  printf("OK\n");
  return 0;
}
