/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Stress/concurrency regression tests for CAN worker loop and shared-state synchronization.
 *
 * Design notes:
 *   Protects against races between command ingestion, worker execution, and telemetry publication paths.
 */

#define STC_TEST_HOOKS 1
#define main stc_server_main
#include "../src/stc_server.c"
#undef main

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

typedef struct {
  shared_t* sh;
  actionq_t* q;
} stress_ctx_t;

static void check_pthread(int rc, const char* what) {
  if (rc != 0) {
    fprintf(stderr, "%s failed rc=%d\n", what, rc);
    abort();
  }
}

static void init_backend_cfg_for_test(stc_config_t* cfg) {
  memset(cfg, 0, sizeof(*cfg));
  snprintf(cfg->backend, sizeof(cfg->backend), "%s", "simulator");
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    cfg->axis_enabled[axis] = 0;
    cfg->axis_driver[axis] = STC_AXIS_DRIVER_DISABLED;
  }
}

static void test_runtime_backend_and_driver_selection(void) {
  stc_config_t cfg;
  stc_runtime_backend_t runtime_backend = STC_RUNTIME_BACKEND_SIMULATOR;

  init_backend_cfg_for_test(&cfg);
  snprintf(cfg.backend, sizeof(cfg.backend), "%s", "can");
  cfg.axis_enabled[0] = 1;
  cfg.axis_driver[0] = STC_AXIS_DRIVER_PD4C;
  assert(select_runtime_backend(&cfg, &runtime_backend) == 0);
  assert(runtime_backend == STC_RUNTIME_BACKEND_HARDWARE_CAN);
  assert(cfg.axis_driver[0] == STC_AXIS_DRIVER_PD4C);

  init_backend_cfg_for_test(&cfg);
  snprintf(cfg.backend, sizeof(cfg.backend), "%s", "can");
  cfg.axis_enabled[1] = 1;
  cfg.axis_driver[1] = STC_AXIS_DRIVER_MYACT_X8_25;
  assert(select_runtime_backend(&cfg, &runtime_backend) == 0);
  assert(runtime_backend == STC_RUNTIME_BACKEND_HARDWARE_CAN);
  assert(cfg.axis_driver[1] == STC_AXIS_DRIVER_MYACT_X8_25);

  init_backend_cfg_for_test(&cfg);
  snprintf(cfg.backend, sizeof(cfg.backend), "%s", "can");
  cfg.axis_enabled[2] = 0;
  cfg.axis_driver[2] = STC_AXIS_DRIVER_DISABLED;
  assert(select_runtime_backend(&cfg, &runtime_backend) == 0);
  assert(runtime_backend == STC_RUNTIME_BACKEND_HARDWARE_CAN);
  assert(cfg.axis_driver[2] == STC_AXIS_DRIVER_DISABLED);

  init_backend_cfg_for_test(&cfg);
  snprintf(cfg.backend, sizeof(cfg.backend), "%s", "myactuator");
  cfg.axis_enabled[0] = 1;
  cfg.axis_driver[0] = STC_AXIS_DRIVER_MYACT_X8_25;
  assert(select_runtime_backend(&cfg, &runtime_backend) != 0);

  init_backend_cfg_for_test(&cfg);
  snprintf(cfg.backend, sizeof(cfg.backend), "%s", "pd4c");
  cfg.axis_enabled[0] = 1;
  cfg.axis_driver[0] = STC_AXIS_DRIVER_PD4C;
  assert(select_runtime_backend(&cfg, &runtime_backend) != 0);
}

static void* action_producer_thread(void* arg) {
  stress_ctx_t* ctx = (stress_ctx_t*)arg;
  for (int i = 0; i < 20000; i++) {
    stc_action_t act;
    if ((i % 11) == 0) {
      stc_action_make_simple(&act, ACTION_RESET, 0xFFFFFFFFu);
    } else if ((i % 3) == 0) {
      double target_deg[STC_AXIS_COUNT] = {0};
      for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) target_deg[axis] = (double)((i + (int)axis) % 360);
      stc_action_make_pp_move(&act, 0xFFFFFFFFu, target_deg);
    } else {
      double speed_deg_s[STC_AXIS_COUNT] = {0};
      for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) speed_deg_s[axis] = (double)((i + (int)axis) % 20) - 10.0;
      stc_action_make_stream_speed(&act, 0xFFFFFFFFu, speed_deg_s);
    }

    while (actionq_push(ctx->q, &act) != 0) {
      usleep(50);
    }

    pthread_mutex_lock(&ctx->sh->mtx);
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      ctx->sh->d.stream[axis].enabled = 1;
      ctx->sh->d.stream[axis].desired_speed_deg_s = (double)((i + (int)axis) % 30) - 15.0;
      ctx->sh->d.stream[axis].last_cmd_ms = mono_ms();
    }
    pthread_mutex_unlock(&ctx->sh->mtx);
  }
  return NULL;
}

static void* telemetry_updater_thread(void* arg) {
  shared_t* sh = (shared_t*)arg;
  for (int i = 0; i < 30000; i++) {
    pthread_mutex_lock(&sh->mtx);
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      sh->d.telem[axis].status_word = (uint16_t)((i + (int)axis) & 0xFFFF);
      sh->d.last_rx_ms[axis] = mono_ms();
      sh->d.cfg[axis].max_speed_deg_s = 120.0 + (double)(i % 10);
      sh->d.ds402_timeout_ms = 100 + (i % 5);
      sh->d.can_timeout_ms = 100 + (uint32_t)(i % 5);
      sh->d.speed_watchdog_ms = 100 + (i % 20);
    }
    pthread_mutex_unlock(&sh->mtx);
    usleep(50);
  }
  return NULL;
}

int main(void) {
  test_runtime_backend_and_driver_selection();
  shared_t sh;
  memset(&sh, 0, sizeof(sh));
  pthread_mutex_init(&sh.mtx, NULL);

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    sh.d.axis_node_id[axis] = 0;
    sh.d.cfg[axis].counts_per_rev = 4096;
    sh.d.cfg[axis].gear_ratio = 1.0;
    sh.d.cfg[axis].pp_profile_vel = 100;
    sh.d.cfg[axis].vel_raw_units = AXIS_VEL_RAW_DEG_PER_SEC;
    sh.d.last_rx_ms[axis] = mono_ms();
  }
  sh.d.st_state = STC_STATE_OPER;
  sh.d.speed_watchdog_ms = 250;
  sh.d.ds402_timeout_ms = 200;
  sh.d.can_timeout_ms = 250;

  actionq_t q;
  actionq_init(&q);

  can_worker_arg_t warg = {
      .sh = &sh,
      .q = &q,
      .can_if = "vcan0",
  };

  atomic_store(&g_stop, 0);

  pthread_t can_thread;
  pthread_t producer_thread;
  pthread_t telemetry_thread;

  check_pthread(pthread_create(&can_thread, NULL, can_worker_thread, &warg), "pthread_create(can)");

  usleep(20000);
  assert(atomic_load(&g_can_worker_action_dispatch_iterations) == 0);

  stc_action_t single_action;
  stc_action_make_simple(&single_action, ACTION_RESET, 0xFFFFFFFFu);
  assert(actionq_push(&q, &single_action) == 0);

  const uint64_t consume_deadline = mono_ms() + 500;
  while (actionq_count(&q) != 0 && mono_ms() < consume_deadline) {
    usleep(1000);
  }
  assert(actionq_count(&q) == 0);
  assert(atomic_load(&g_can_worker_action_dispatch_iterations) >= 1);

  stress_ctx_t stress_ctx = {.sh = &sh, .q = &q};
  check_pthread(pthread_create(&producer_thread, NULL, action_producer_thread, &stress_ctx), "pthread_create(producer)");
  check_pthread(pthread_create(&telemetry_thread, NULL, telemetry_updater_thread, &sh), "pthread_create(telemetry)");

  check_pthread(pthread_join(producer_thread, NULL), "pthread_join(producer)");
  check_pthread(pthread_join(telemetry_thread, NULL), "pthread_join(telemetry)");

  atomic_store(&g_stop, 1);
  check_pthread(pthread_join(can_thread, NULL), "pthread_join(can)");

  pthread_mutex_destroy(&sh.mtx);
  printf("can_worker concurrency stress completed\n");
  return 0;
}
