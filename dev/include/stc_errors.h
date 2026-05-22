/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Shared naming and alias layer for STC ICD error codes used across runtime and tooling.
 *
 * Design notes:
 *   Bridges wire-level NACK codes and local motion-validation aliases.
 *   Keeps compatibility naming centralized without owning packet parsing behavior.
 *   String names are consumed by CLI/GUI surfaces for operator-visible diagnostics.
 */

#pragma once

#include "stc_icd.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Stringifies ICD-visible NACK codes for operator-facing logs and UI clients. */
const char* stc_error_code_name(stc_nack_err_t code);

/* Motion pre-validation specific aliases (kept distinct at call sites). */
#define STC_ERR_INVALID_AXIS STC_ERR_NOT_CONFIGURED
#define STC_ERR_AXIS_FAULT STC_ERR_GENERIC
#define STC_ERR_AXIS_STALE STC_ERR_TIMEOUT

#ifdef __cplusplus
}
#endif
