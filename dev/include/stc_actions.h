/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Declares the internal action/command queue exchanged between runtime threads.
 *
 * Design notes:
 *   TCP command thread enqueues actions, CAN worker dequeues and executes them.
 *   Stream-speed actions are coalesced to keep freshest rate command per axis.
 *   eventfd wakeup is used so CAN worker can sleep without busy-waiting.
 */

#ifndef STC_ACTIONS_H
#define STC_ACTIONS_H

#include "stc_model.h"

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>

/** Action opcode consumed by the CAN worker execution loop. */
typedef enum {
  ACTION_SET_STATE = 0,
  ACTION_PP_MOVE,
  ACTION_MOTION,
  ACTION_STREAM_SPEED,
  ACTION_STOP,
  ACTION_QUICK_STOP,
  ACTION_ENTER_STORE,
  ACTION_RESET,
  ACTION_FACTORY,
  ACTION_SHUTDOWN,
  ACTION_CAN_REDISCOVER,
  ACTION_CLEAR_ERRORS,
  ACTION_SET_LASER
} stc_action_type_t;

typedef enum {
  STC_AXIS_MOTION_ABS = 0,
  STC_AXIS_MOTION_REL = 1,
  STC_AXIS_MOTION_SPEED = 2
} stc_axis_motion_kind_t;

/** One queued action plus optional axis payload data. */
typedef struct {
  uint32_t axes_mask;
  uint64_t obs_tcp_rx_ns;
  uint64_t obs_queued_ns;
  union {
    struct {
      stc_axis_motion_kind_t kind[STC_AXIS_COUNT];
      double value_deg_or_dps[STC_AXIS_COUNT];
      double speed_limit_deg_s[STC_AXIS_COUNT];
    } motion;
    double target_deg[STC_AXIS_COUNT];
    double speed_deg_s[STC_AXIS_COUNT];
    int8_t new_state;
    struct {
      uint8_t laser_id;
      uint8_t on_off;
    } laser;
  } u;
  stc_action_type_t type;
} stc_action_t;

#define STC_ACTIONQ_CAPACITY 1000

typedef struct {
  stc_action_t items[STC_ACTIONQ_CAPACITY];
  size_t head;
  size_t tail;
  size_t count;
  int wake_fd;
  pthread_mutex_t mtx;
  pthread_cond_t cv;
} actionq_t;

/** Initialize queue state and create eventfd wakeup handle for CAN worker poll loop. */
void actionq_init(actionq_t* q);
/** Push action into queue (or coalesce stream-speed actions); wakes worker via eventfd.
 *  Queue admission means "valid + queued", not execution success.
 */
int actionq_push(actionq_t* q, const stc_action_t* action);
/** Peek current head action without removal; returns non-zero if queue is empty. */
int actionq_try_peek(actionq_t* q, stc_action_t* out);
/** Drop current head action after execution. */
int actionq_drop_head(actionq_t* q);
/** Return eventfd used by reactor/epoll to wake on new queued actions. */
int actionq_wakeup_fd(const actionq_t* q);
/** Drain pending eventfd counter after wakeup notification is observed. */
void actionq_consume_wakeup(actionq_t* q);
/** Snapshot queue depth for diagnostics/telemetry. */
size_t actionq_count(actionq_t* q);

/** Build action with opcode and axis mask only. */
void stc_action_make_simple(stc_action_t* action, stc_action_type_t type, uint32_t axes_mask);
/** Build state-transition action emitted by command thread. */
void stc_action_make_set_state(stc_action_t* action, int8_t new_state);
/** Build profile-position action in degrees for selected axes. */
void stc_action_make_pp_move(stc_action_t* action, uint32_t axes_mask, const double target_deg[STC_AXIS_COUNT]);
/** Build mixed absolute/relative/speed motion action with optional speed limits. */
void stc_action_make_motion(stc_action_t* action, uint32_t axes_mask, const stc_axis_motion_kind_t kind[STC_AXIS_COUNT], const double value_deg_or_dps[STC_AXIS_COUNT], const double speed_limit_deg_s[STC_AXIS_COUNT]);
/** Build stream-speed action subject to coalescing semantics. */
void stc_action_make_stream_speed(stc_action_t* action, uint32_t axes_mask, const double speed_deg_s[STC_AXIS_COUNT]);
/** Build laser toggle action targeting one logical laser output. */
void stc_action_make_set_laser(stc_action_t* action, uint8_t laser_id, uint8_t on_off);

#endif
