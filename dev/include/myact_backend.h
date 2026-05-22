/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Helper declarations shared by the MYACT backend implementation.
 *
 * Design notes:
 *   Startup/discovery and transport-drain helpers only; no state ownership or
 *   worker policy lives here.
 *   Called from backend-owned startup/maintenance paths, not motion-loop hot paths.
 *   Assumes MYACT transport/protocol contexts are already initialized by caller.
 *   Caller controls threading and sequencing around asynchronous reply draining.
 */

#ifndef STC_MYACT_BACKEND_H
#define STC_MYACT_BACKEND_H

#include "motion_backend.h"
#include "x8_25.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bench/startup probe helper; validates one axis path outside motion-loop control flow. */
int myact_backend_discover_axis(motion_backend_t* backend, size_t axis, const char* can_if, uint8_t node_id);
/* Disable and drain vendor asynchronous replies before normal command sequencing. */
int myact_backend_quiesce_active_replies(myact_ctx_t* ctx, int drain_ms);

#ifdef __cplusplus
}
#endif

#endif
