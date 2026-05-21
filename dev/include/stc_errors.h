#pragma once

#include "stc_icd.h"

#ifdef __cplusplus
extern "C" {
#endif

const char* stc_error_code_name(stc_nack_err_t code);

/* Motion pre-validation specific aliases (kept distinct at call sites). */
#define STC_ERR_INVALID_AXIS STC_ERR_NOT_CONFIGURED
#define STC_ERR_AXIS_FAULT STC_ERR_GENERIC
#define STC_ERR_AXIS_STALE STC_ERR_TIMEOUT

#ifdef __cplusplus
}
#endif
