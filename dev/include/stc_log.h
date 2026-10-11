/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Thread-safe process logger API used by STC runtime and test utilities.
 *
 * Design notes:
 *   Intended for concurrent use from TCP/CAN/telemetry threads and unit/integration
 *   tests; implementation internally synchronizes output serialization.
 */

#pragma once

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Logger functions are thread-safe and internally synchronized. */
typedef enum {
  STC_LOG_LEVEL_ERROR = 0,
  STC_LOG_LEVEL_WARN = 1,
  STC_LOG_LEVEL_INFO = 2,
  STC_LOG_LEVEL_DEBUG = 3,
  STC_LOG_LEVEL_TRACE = 4
} stc_log_level_t;

void stc_log_init(const char* filename, stc_log_level_t level);
void stc_log_close(void);
void stc_log_set_level(stc_log_level_t level);
int stc_log_level_from_string(const char* value, stc_log_level_t* out_level);
void stc_log(stc_log_level_t level, const char* fmt, ...);
void stc_vlog(stc_log_level_t level, const char* fmt, va_list ap);

#define STC_LOG_ERROR(...) stc_log(STC_LOG_LEVEL_ERROR, __VA_ARGS__)
#define STC_LOG_WARN(...)  stc_log(STC_LOG_LEVEL_WARN, __VA_ARGS__)
#define STC_LOG_INFO(...)  stc_log(STC_LOG_LEVEL_INFO, __VA_ARGS__)
#define STC_LOG_DEBUG(...) stc_log(STC_LOG_LEVEL_DEBUG, __VA_ARGS__)
#define STC_LOG_TRACE(...) stc_log(STC_LOG_LEVEL_TRACE, __VA_ARGS__)

#ifdef __cplusplus
}
#endif
