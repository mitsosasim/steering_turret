#define _POSIX_C_SOURCE 200809L

#include "stc_log.h"

#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static FILE* g_log_file = NULL;
static stc_log_level_t g_log_level = STC_LOG_LEVEL_INFO;
static pthread_mutex_t g_log_mutex = PTHREAD_MUTEX_INITIALIZER;

static const char* level_tag(stc_log_level_t level) {
  switch (level) {
    case STC_LOG_LEVEL_ERROR: return "ERROR";
    case STC_LOG_LEVEL_WARN: return "WARN ";
    case STC_LOG_LEVEL_INFO: return "INFO ";
    case STC_LOG_LEVEL_DEBUG: return "DEBUG";
    default: return "UNKWN";
  }
}

static void format_timestamp(char* out, size_t out_sz) {
  if (!out || out_sz == 0) {
    return;
  }

  struct timespec ts_now;
  clock_gettime(CLOCK_REALTIME, &ts_now);

  struct tm tm_now;
  localtime_r(&ts_now.tv_sec, &tm_now);

  char base[32];
  if (strftime(base, sizeof(base), "%Y-%m-%d %H:%M:%S", &tm_now) == 0) {
    snprintf(out, out_sz, "0000-00-00 00:00:00.000");
    return;
  }

  long millis = ts_now.tv_nsec / 1000000L;
  if (millis < 0) {
    millis = 0;
  }
  if (millis > 999) {
    millis = 999;
  }

  snprintf(out, out_sz, "%s.%03ld", base, millis);
}

void stc_log_init(const char* filename, stc_log_level_t level) {
  pthread_mutex_lock(&g_log_mutex);

  g_log_level = level;

  if (g_log_file) {
    fclose(g_log_file);
    g_log_file = NULL;
  }

  if (filename && filename[0] != '\0') {
    g_log_file = fopen(filename, "a");
    if (g_log_file) {
      setvbuf(g_log_file, NULL, _IOLBF, 0);
    }
  }

  pthread_mutex_unlock(&g_log_mutex);
}

void stc_log_close(void) {
  pthread_mutex_lock(&g_log_mutex);

  if (g_log_file) {
    fclose(g_log_file);
    g_log_file = NULL;
  }

  pthread_mutex_unlock(&g_log_mutex);
}

void stc_log_set_level(stc_log_level_t level) {
  pthread_mutex_lock(&g_log_mutex);
  g_log_level = level;
  pthread_mutex_unlock(&g_log_mutex);
}

void stc_vlog(stc_log_level_t level, const char* fmt, va_list ap) {
  char ts[64];
  format_timestamp(ts, sizeof(ts));

  char message[1024];
  va_list ap_copy;
  va_copy(ap_copy, ap);
  vsnprintf(message, sizeof(message), fmt, ap_copy);
  va_end(ap_copy);

  pthread_mutex_lock(&g_log_mutex);

  if (level > g_log_level) {
    pthread_mutex_unlock(&g_log_mutex);
    return;
  }

  fprintf(stderr, "%s | %s | %s\n", ts, level_tag(level), message);
  if (g_log_file) {
    fprintf(g_log_file, "%s | %s | %s\n", ts, level_tag(level), message);
  }

  pthread_mutex_unlock(&g_log_mutex);
}

void stc_log(stc_log_level_t level, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  stc_vlog(level, fmt, ap);
  va_end(ap);
}
