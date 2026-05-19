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

#define _POSIX_C_SOURCE 200809L

#include "stc_log.h"

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

enum {
  LOGGER_THREADS = 6,
  LOG_ITERS_PER_THREAD = 4000,
  RECONFIG_ITERS = 2500,
};

static atomic_int g_start_flag = 0;

static void tiny_sleep(void) {
  const struct timespec ts = {.tv_sec = 0, .tv_nsec = 100000L};
  nanosleep(&ts, NULL);
}

static void check_pthread(int rc, const char* what) {
  if (rc != 0) {
    fprintf(stderr, "%s failed rc=%d\n", what, rc);
    abort();
  }
}

static void* logger_thread(void* arg) {
  intptr_t tid = (intptr_t)arg;
  while (atomic_load_explicit(&g_start_flag, memory_order_acquire) == 0) {
    tiny_sleep();
  }

  for (int i = 0; i < LOG_ITERS_PER_THREAD; i++) {
    stc_log(STC_LOG_LEVEL_INFO, "logger=%ld iter=%d", (long)tid, i);
    if ((i % 11) == 0) {
      stc_log(STC_LOG_LEVEL_DEBUG, "debug logger=%ld iter=%d", (long)tid, i);
    }
  }

  return NULL;
}

static void* reconfig_thread(void* arg) {
  (void)arg;
  const char* path = "test_log_concurrency.log";

  while (atomic_load_explicit(&g_start_flag, memory_order_acquire) == 0) {
    tiny_sleep();
  }

  for (int i = 0; i < RECONFIG_ITERS; i++) {
    switch (i % 3) {
      case 0:
        stc_log_set_level(((i % 7) == 0) ? STC_LOG_LEVEL_INFO : STC_LOG_LEVEL_ERROR);
        break;
      case 1:
        stc_log_close();
        break;
      default:
        stc_log_init(path, ((i % 7) == 0) ? STC_LOG_LEVEL_INFO : STC_LOG_LEVEL_ERROR);
        break;
    }
  }

  return NULL;
}

int main(void) {
  const char* path = "test_log_concurrency.log";
  remove(path);

  stc_log_init(path, STC_LOG_LEVEL_ERROR);

  pthread_t log_threads[LOGGER_THREADS];
  pthread_t cfg_thread;

  for (intptr_t i = 0; i < LOGGER_THREADS; i++) {
    check_pthread(pthread_create(&log_threads[i], NULL, logger_thread, (void*)i), "pthread_create(logger)");
  }
  check_pthread(pthread_create(&cfg_thread, NULL, reconfig_thread, NULL), "pthread_create(reconfig)");

  atomic_store_explicit(&g_start_flag, 1, memory_order_release);

  for (int i = 0; i < LOGGER_THREADS; i++) {
    check_pthread(pthread_join(log_threads[i], NULL), "pthread_join(logger)");
  }
  check_pthread(pthread_join(cfg_thread, NULL), "pthread_join(reconfig)");

  stc_log_init(path, STC_LOG_LEVEL_INFO);
  stc_log(STC_LOG_LEVEL_INFO, "final message");
  stc_log_close();

  FILE* fp = fopen(path, "r");
  assert(fp != NULL);

  int lines = 0;
  int ch;
  while ((ch = fgetc(fp)) != EOF) {
    if (ch == '\n') lines++;
  }
  fclose(fp);

  assert(lines > 0);
  printf("log concurrency stress completed with %d lines\n", lines);
  return 0;
}
