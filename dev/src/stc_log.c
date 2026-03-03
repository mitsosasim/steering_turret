#define _POSIX_C_SOURCE 200809L

#include "stc_log.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

static FILE* g_log_file = NULL;
static stc_log_level_t g_log_level = STC_LOG_LEVEL_INFO;

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
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);

  struct tm tm_now;
  localtime_r(&ts.tv_sec, &tm_now);

  const long millis = ts.tv_nsec / 1000000L;
  snprintf(out,
           out_sz,
           "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
           tm_now.tm_year + 1900,
           tm_now.tm_mon + 1,
           tm_now.tm_mday,
           tm_now.tm_hour,
           tm_now.tm_min,
           tm_now.tm_sec,
           millis);
}

void stc_log_init(const char* filename, stc_log_level_t level) {
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
}

void stc_log_close(void) {
  if (g_log_file) {
    fclose(g_log_file);
    g_log_file = NULL;
  }
}

void stc_log_set_level(stc_log_level_t level) {
  g_log_level = level;
}

void stc_vlog(stc_log_level_t level, const char* fmt, va_list ap) {
  if (level > g_log_level) return;

  char ts[32];
  format_timestamp(ts, sizeof(ts));

  char message[1024];
  va_list ap_copy;
  va_copy(ap_copy, ap);
  vsnprintf(message, sizeof(message), fmt, ap_copy);
  va_end(ap_copy);

  fprintf(stderr, "%s | %s | %s\n", ts, level_tag(level), message);
  if (g_log_file) {
    fprintf(g_log_file, "%s | %s | %s\n", ts, level_tag(level), message);
  }
}

void stc_log(stc_log_level_t level, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  stc_vlog(level, fmt, ap);
  va_end(ap);
}
