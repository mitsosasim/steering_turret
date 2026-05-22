/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Action queue implementation for TCP-to-worker handoff.
 *
 * Design notes:
 *   Thread-safe ring buffer with eventfd wakeups for producer/consumer handoff.
 *   TCP-side validation owns frame/protocol checks before enqueue.
 *   Worker thread owns backend execution after dequeue.
 *   Stream-speed commands use keep-latest coalescing to drop stale intents.
 */

#include "stc_actions.h"

#include <errno.h>
#include <string.h>
#include <sys/eventfd.h>
#include <unistd.h>

/* Queue is producer/consumer between TCP command thread and CAN worker.
 * eventfd wakeups integrate queue activity with poll/epoll without busy loops.
 */
/* Wakeup side only; queue signaling does not transfer backend execution ownership. */
static void actionq_notify_locked(actionq_t* q) {
  if (q->wake_fd < 0) return;
  const uint64_t one = 1;
  ssize_t wr = write(q->wake_fd, &one, sizeof(one));
  if (wr < 0 && errno != EAGAIN) {
    (void)wr;
  }
}

static size_t actionq_filter_locked(actionq_t* q, stc_action_type_t type_to_drop) {
  /* Coalesce stale stream-speed commands so worker always executes latest rate intent. */
  size_t kept = 0;
  stc_action_t compacted[STC_ACTIONQ_CAPACITY];

  for (size_t i = 0; i < q->count; i++) {
    size_t idx = (q->head + i) % STC_ACTIONQ_CAPACITY;
    if (q->items[idx].type == type_to_drop) {
      continue;
    }
    compacted[kept++] = q->items[idx];
  }

  if (kept == q->count) {
    return 0;
  }

  for (size_t i = 0; i < kept; i++) {
    q->items[i] = compacted[i];
  }
  q->head = 0;
  q->tail = kept % STC_ACTIONQ_CAPACITY;
  q->count = kept;
  return 1;
}

void actionq_init(actionq_t* q) {
  memset(q, 0, sizeof(*q));
  q->wake_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  pthread_mutex_init(&q->mtx, NULL);
  pthread_cond_init(&q->cv, NULL);
}

int actionq_push(actionq_t* q, const stc_action_t* action) {
  /* Important semantics:
   * - success => action accepted into queue for deferred worker execution
   * - success does NOT imply backend completion
   */
  pthread_mutex_lock(&q->mtx);

  if (action->type == ACTION_STREAM_SPEED) {
    /* Stream-speed is keep-latest: stale speed intents are dropped/coalesced. */
    (void)actionq_filter_locked(q, ACTION_STREAM_SPEED);
  }

  if (q->count >= STC_ACTIONQ_CAPACITY) {
    pthread_mutex_unlock(&q->mtx);
    return -1;
  }
  q->items[q->tail] = *action;
  q->tail = (q->tail + 1) % STC_ACTIONQ_CAPACITY;
  q->count++;
  actionq_notify_locked(q);
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

int actionq_wakeup_fd(const actionq_t* q) {
  return q->wake_fd;
}

void actionq_consume_wakeup(actionq_t* q) {
  if (q->wake_fd < 0) return;
  uint64_t ignored;
  while (read(q->wake_fd, &ignored, sizeof(ignored)) == sizeof(ignored)) {
  }
}

size_t actionq_count(actionq_t* q) {
  size_t count = 0;
  pthread_mutex_lock(&q->mtx);
  count = q->count;
  pthread_mutex_unlock(&q->mtx);
  return count;
}

/** Build queue action for opcodes that only require type + axes mask. */
void stc_action_make_simple(stc_action_t* action, stc_action_type_t type, uint32_t axes_mask) {
  memset(action, 0, sizeof(*action));
  action->type = type;
  action->axes_mask = axes_mask;
}

/** Build explicit state transition request consumed by worker state gate. */
void stc_action_make_set_state(stc_action_t* action, int8_t new_state) {
  stc_action_make_simple(action, ACTION_SET_STATE, 0u);
  action->u.new_state = new_state;
}

/** Build profile-position move request (absolute targets, degrees). */
void stc_action_make_pp_move(stc_action_t* action, uint32_t axes_mask, const double target_deg[STC_AXIS_COUNT]) {
  stc_action_make_simple(action, ACTION_PP_MOVE, axes_mask);
  memcpy(action->u.target_deg, target_deg, sizeof(action->u.target_deg));
}


/** Build mixed-motion command carrying per-axis mode/value/speed limits. */
void stc_action_make_motion(stc_action_t* action, uint32_t axes_mask, const stc_axis_motion_kind_t kind[STC_AXIS_COUNT], const double value_deg_or_dps[STC_AXIS_COUNT], const double speed_limit_deg_s[STC_AXIS_COUNT]) {
  stc_action_make_simple(action, ACTION_MOTION, axes_mask);
  memset(&action->u.motion, 0, sizeof(action->u.motion));
  memcpy(action->u.motion.kind, kind, sizeof(action->u.motion.kind));
  memcpy(action->u.motion.value_deg_or_dps, value_deg_or_dps, sizeof(action->u.motion.value_deg_or_dps));
  memcpy(action->u.motion.speed_limit_deg_s, speed_limit_deg_s, sizeof(action->u.motion.speed_limit_deg_s));
}

/** Build streamed-speed request; queue semantics coalesce to newest command. */
void stc_action_make_stream_speed(stc_action_t* action, uint32_t axes_mask, const double speed_deg_s[STC_AXIS_COUNT]) {
  stc_action_make_simple(action, ACTION_STREAM_SPEED, axes_mask);
  memcpy(action->u.speed_deg_s, speed_deg_s, sizeof(action->u.speed_deg_s));
}

/** Build logical-laser output toggle action for backend dispatch. */
void stc_action_make_set_laser(stc_action_t* action, uint8_t laser_id, uint8_t on_off) {
  stc_action_make_simple(action, ACTION_SET_LASER, 0u);
  action->u.laser.laser_id = laser_id;
  action->u.laser.on_off = on_off;
}
