#ifndef STC_ACTIONS_H
#define STC_ACTIONS_H

#include "stc_model.h"

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  ACTION_SET_STATE = 0,
  ACTION_PP_MOVE,
  ACTION_STREAM_SPEED,
  ACTION_STOP,
  ACTION_QUICK_STOP,
  ACTION_ENTER_STORE,
  ACTION_RESET,
  ACTION_FACTORY,
  ACTION_SHUTDOWN
} stc_action_type_t;

typedef struct {
  uint32_t axes_mask;
  union {
    double target_deg[STC_AXIS_COUNT];
    double speed_deg_s[STC_AXIS_COUNT];
    int8_t new_state;
  } u;
  stc_action_type_t type;
} stc_action_t;

#define STC_ACTIONQ_CAPACITY 1000

typedef struct {
  stc_action_t items[STC_ACTIONQ_CAPACITY];
  size_t head;
  size_t tail;
  size_t count;
  pthread_mutex_t mtx;
  pthread_cond_t cv;
} actionq_t;

void actionq_init(actionq_t* q);
int actionq_push(actionq_t* q, const stc_action_t* action);
int actionq_try_peek(actionq_t* q, stc_action_t* out);
int actionq_drop_head(actionq_t* q);

void stc_action_make_simple(stc_action_t* action, stc_action_type_t type, uint32_t axes_mask);
void stc_action_make_set_state(stc_action_t* action, int8_t new_state);
void stc_action_make_pp_move(stc_action_t* action, uint32_t axes_mask, const double target_deg[STC_AXIS_COUNT]);
void stc_action_make_stream_speed(stc_action_t* action, uint32_t axes_mask, const double speed_deg_s[STC_AXIS_COUNT]);

#endif
