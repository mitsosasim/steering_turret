#define _POSIX_C_SOURCE 200809L

#include "motion_backend.h"

#include "pd4c.h"
#include "stc_log.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

typedef enum {
  MTR_MODE_NONE = 0,
  MTR_MODE_PV = PD4C_MODE_PROFILE_VELOCITY,
  MTR_MODE_PP = PD4C_MODE_PROFILE_POSITION
} mtr_mode_t;

struct motion_backend {
  motion_backend_cfg_t cfg;
  pd4c_ctx_t ctx[STC_AXIS_COUNT];
  int axis_connected[STC_AXIS_COUNT];
  int axis_enabled[STC_AXIS_COUNT];
  mtr_mode_t axis_mode[STC_AXIS_COUNT];
  motion_snapshot_t snap;
};

static uint64_t mono_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static int ensure_enabled(struct motion_backend* b, size_t axis, mtr_mode_t mode) {
  if (b->axis_enabled[axis] && b->axis_mode[axis] == mode) return PD4C_OK;
  int rc = pd4c_ds402_enable_operation(&b->ctx[axis], (int8_t)mode, b->cfg.ds402_timeout_ms);
  if (rc == PD4C_OK) {
    b->axis_enabled[axis] = 1;
    b->axis_mode[axis] = mode;
  }
  return rc;
}

static int verify_node_responsive(pd4c_ctx_t* ctx, uint32_t timeout_ms) {
  if (!ctx) return 0;
  uint32_t vendor_id = 0;
  int effective_timeout_ms = (timeout_ms > 0 && timeout_ms <= INT32_MAX) ? (int)timeout_ms : 500;
  pd4c_set_sdo_timeout(ctx, effective_timeout_ms);
  return pd4c_sdo_read_u32(ctx, 0x1018u, 0x01u, &vendor_id) == PD4C_OK;
}

int motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend) {
  if (!cfg || !out_backend) return -1;
  struct motion_backend* b = (struct motion_backend*)calloc(1, sizeof(*b));
  if (!b) return -1;
  b->cfg = *cfg;
  STC_LOG_INFO("CAN backend assumes interface '%s' is preconfigured externally (manual canup/systemd); skipping interface setup commands.",
    (cfg->can_if && cfg->can_if[0]) ? cfg->can_if : "(unset)");

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    uint8_t node_id = cfg->axis_node_id[axis];
    if (!node_id) continue;
    int rc = pd4c_open(&b->ctx[axis], cfg->can_if, node_id);
    if (rc != PD4C_OK) continue;

    (void)pd4c_nmt_start(&b->ctx[axis]);
    if (!verify_node_responsive(&b->ctx[axis], cfg->can_timeout_ms)) {
      STC_LOG_WARN("CAN node %u (axis %zu) opened but failed liveness verification; closing context.", node_id, axis);
      pd4c_close(&b->ctx[axis]);
      continue;
    }

    b->axis_connected[axis] = 1;
    b->snap.axis_available[axis] = 1;
    b->snap.can_connected_devices++;
    b->snap.last_rx_ms[axis] = mono_ms();
  }
  b->snap.can_ok = 1;
  *out_backend = b;
  return 0;
}

void motion_backend_stop(motion_backend_t* backend) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (b->axis_connected[axis]) pd4c_close(&b->ctx[axis]);
  }
  free(b);
}

int motion_backend_discover(motion_backend_t* backend) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;
  int discovered_count = 0;
  int scan_max = b->cfg.can_scan_max_node;
  if (scan_max < 1) scan_max = 1;
  if (scan_max > 127) scan_max = 127;
  memset(b->snap.can_discovered_nodes, 0, sizeof(b->snap.can_discovered_nodes));

  for (int node = 1; node <= scan_max; node++) {
    pd4c_ctx_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    if (pd4c_open(&tmp, b->cfg.can_if, (uint8_t)node) != PD4C_OK) continue;
    (void)pd4c_nmt_start(&tmp);
    pd4c_telemetry_t t;
    memset(&t, 0, sizeof(t));
    int present = 0;
    for (int k = 0; k < 2; k++) {
      if (pd4c_poll(&tmp, 40) == PD4C_OK) {
        t = *pd4c_telem(&tmp);
        if (t.have_hb || t.have_tpdo1 || t.have_tpdo2 || t.have_tpdo3) { present = 1; break; }
      }
    }
    if (present && discovered_count < 16) b->snap.can_discovered_nodes[discovered_count++] = (uint8_t)node;
    pd4c_close(&tmp);
  }
  b->snap.can_discovered_devices = (uint8_t)discovered_count;
  return discovered_count;
}

int motion_backend_apply_pp_move(motion_backend_t* backend, uint32_t axes_mask, const double targets_deg[STC_AXIS_COUNT]) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !targets_deg) return -1;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0 || !b->axis_connected[axis]) continue;
    if (ensure_enabled(b, axis, MTR_MODE_PP) != PD4C_OK) continue;
    int32_t target_counts = axis_deg_to_counts(&b->cfg.axis_cfg[axis], targets_deg[axis]);
    uint16_t pv = b->cfg.axis_cfg[axis].pp_profile_vel;
    (void)pd4c_pp_move(&b->ctx[axis], target_counts, pv, 0, b->cfg.ds402_timeout_ms);
  }
  return 0;
}

int motion_backend_apply_speed(motion_backend_t* backend, uint32_t axes_mask, const double speeds_deg_s[STC_AXIS_COUNT]) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !speeds_deg_s) return -1;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0 || !b->axis_connected[axis]) continue;
    if (ensure_enabled(b, axis, MTR_MODE_PV) != PD4C_OK) continue;
    int clipped = 0;
    int32_t tv_raw = axis_deg_s_to_target_vel_units(&b->cfg.axis_cfg[axis], speeds_deg_s[axis], &clipped);
    if (tv_raw > INT16_MAX) tv_raw = INT16_MAX;
    if (tv_raw < INT16_MIN) tv_raw = INT16_MIN;
    (void)pd4c_rpdo3_send_target_velocity(&b->ctx[axis], (int16_t)tv_raw);
  }
  return 0;
}

int motion_backend_quick_stop(motion_backend_t* backend, uint32_t axes_mask) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0 || !b->axis_connected[axis]) continue;
    (void)pd4c_ds402_quick_stop(&b->ctx[axis], (int8_t)b->axis_mode[axis]);
    b->axis_enabled[axis] = 0;
  }
  return 0;
}

int motion_backend_get_snapshot(motion_backend_t* backend, motion_snapshot_t* snapshot) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !snapshot) return -1;
  uint64_t now = mono_ms();
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!b->axis_connected[axis]) continue;
    if (pd4c_poll(&b->ctx[axis], 0) == PD4C_OK) {
      const pd4c_telemetry_t* t = pd4c_telem(&b->ctx[axis]);
      b->snap.pos_raw[axis] = t->pos_actual;
      b->snap.vel_raw[axis] = t->vel_actual;
      b->snap.status_word[axis] = t->statusword;
      b->snap.mode_display[axis] = t->mode_display;
      b->snap.last_rx_ms[axis] = now;
      if (axis == 0) {
        b->snap.hb_state = (uint8_t)t->hb_state;
        b->snap.have_hb = t->have_hb ? 1 : 0;
        b->snap.have_emcy = t->have_emcy ? 1 : 0;
        b->snap.emcy_code = t->emcy_error_code;
        b->snap.emcy_reg = t->emcy_error_reg;
        b->snap.ts_tpdo1_ms = t->ts_tpdo1_ms;
        b->snap.ts_tpdo2_ms = t->ts_tpdo2_ms;
        b->snap.ts_tpdo3_ms = t->ts_tpdo3_ms;
        b->snap.ts_hb_ms = t->ts_hb_ms;
        b->snap.ts_emcy_ms = t->ts_emcy_ms;
      }
    }
  }
  *snapshot = b->snap;
  return 0;
}
