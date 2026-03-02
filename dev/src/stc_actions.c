#include "stc_actions.h"

#include <string.h>

void actionq_init(actionq_t* q) {
  memset(q, 0, sizeof(*q));
  pthread_mutex_init(&q->mtx, NULL);
  pthread_cond_init(&q->cv, NULL);
}

int actionq_push(actionq_t* q, const stc_action_t* action) {
  pthread_mutex_lock(&q->mtx);
  if (q->count >= STC_ACTIONQ_CAPACITY) {
    pthread_mutex_unlock(&q->mtx);
    return -1;
  }
  q->items[q->tail] = *action;
  q->tail = (q->tail + 1) % STC_ACTIONQ_CAPACITY;
  q->count++;
  pthread_cond_signal(&q->cv);
  pthread_mutex_unlock(&q->mtx);
  return 0;
}

int actionq_try_peek(actionq_t* q, stc_action_t* out) {
  int ok = 0;
  pthread_mutex_lock(&q->mtx);
  if (q->count > 0) {
    *out = q->items[q->head];
    ok = 1;
  }
  pthread_mutex_unlock(&q->mtx);
  return ok;
}

int actionq_drop_head(actionq_t* q) {
  int ok = 0;
  pthread_mutex_lock(&q->mtx);
  if (q->count > 0) {
    q->head = (q->head + 1) % STC_ACTIONQ_CAPACITY;
    q->count--;
    ok = 1;
  }
  pthread_mutex_unlock(&q->mtx);
  return ok;
}

void stc_action_make_simple(stc_action_t* action, stc_action_type_t type, uint32_t axes_mask) {
  memset(action, 0, sizeof(*action));
  action->type = type;
  action->axes_mask = axes_mask;
}

void stc_action_make_set_state(stc_action_t* action, int8_t new_state) {
  stc_action_make_simple(action, ACTION_SET_STATE, 0u);
  action->u.new_state = new_state;
}

void stc_action_make_pp_move(stc_action_t* action, uint32_t axes_mask, const double target_deg[STC_AXIS_COUNT]) {
  stc_action_make_simple(action, ACTION_PP_MOVE, axes_mask);
  memcpy(action->u.target_deg, target_deg, sizeof(action->u.target_deg));
}

void stc_action_make_stream_speed(stc_action_t* action, uint32_t axes_mask, const double speed_deg_s[STC_AXIS_COUNT]) {
  stc_action_make_simple(action, ACTION_STREAM_SPEED, axes_mask);
  memcpy(action->u.speed_deg_s, speed_deg_s, sizeof(action->u.speed_deg_s));
}
