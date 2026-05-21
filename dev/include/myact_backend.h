#ifndef STC_MYACT_BACKEND_H
#define STC_MYACT_BACKEND_H

#include "motion_backend.h"
#include "x8_25.h"

#ifdef __cplusplus
extern "C" {
#endif

int myact_backend_discover_axis(motion_backend_t* backend, size_t axis, const char* can_if, uint8_t node_id);
int myact_backend_quiesce_active_replies(myact_ctx_t* ctx, int drain_ms);

#ifdef __cplusplus
}
#endif

#endif
