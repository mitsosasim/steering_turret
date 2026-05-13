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
  /* RPDO3 maps to 0x6042 (Target Velocity), which PD4-C defines for mode 2. */
  MTR_MODE_PV = PD4C_MODE_VELOCITY,
  MTR_MODE_PP = PD4C_MODE_PROFILE_POSITION
} mtr_mode_t;

struct motion_backend {
  motion_backend_cfg_t cfg;
  struct {
    int responsive;
    int identity_ok;
    int identity_rc;
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision;
    uint32_t serial;
  } node_discovery[128];
  pd4c_ctx_t ctx[STC_AXIS_COUNT];
  int axis_connected[STC_AXIS_COUNT];
  int axis_enabled[STC_AXIS_COUNT];
  mtr_mode_t axis_mode[STC_AXIS_COUNT];
  uint64_t axis_last_seen_ms[STC_AXIS_COUNT];
  uint64_t axis_last_healthcheck_ms[STC_AXIS_COUNT];
  motion_snapshot_t snap;
};

/*
 * can_ok criteria:
 * - At least one axis is expected (configured with a non-zero node ID), and
 * - Every expected axis is currently connected/usable.
 *
 * This intentionally treats partial bring-up as degraded (can_ok=0), while
 * preserving per-axis availability in axis_available[] so callers can diagnose
 * which axes are missing.
 */
static int compute_can_ok(const struct motion_backend* b) {
  if (!b) return 0;

  int expected_axes = 0;
  int connected_expected_axes = 0;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (b->cfg.axis_node_id[axis] == 0) continue;
    expected_axes++;
    if (b->axis_connected[axis]) connected_expected_axes++;
  }

  if (expected_axes == 0) return 0;
  return connected_expected_axes == expected_axes;
}

static void refresh_can_status(struct motion_backend* b) {
  if (!b) return;
  b->snap.can_ok = compute_can_ok(b);
  b->snap.capabilities.supports_native_a2_speed = 0;
  b->snap.capabilities.supports_native_absolute_position = 0;
  b->snap.backend_mode = 2;
  b->snap.transport_status = 1;
}

static int node_in_discovery_list(const motion_snapshot_t* snap, uint8_t node_id) {
  if (!snap || node_id == 0) return 0;
  const int discovered = (int)snap->can_discovered_devices;
  const int limit = (int)(sizeof(snap->can_discovered_nodes) / sizeof(snap->can_discovered_nodes[0]));
  const int check = (discovered < limit) ? discovered : limit;
  for (int i = 0; i < check; i++) {
    if (snap->can_discovered_nodes[i] == node_id) return 1;
  }
  return 0;
}

static int bounded_scan_max(int configured_scan_max) {
  int scan_max = configured_scan_max;
  if (scan_max < 1) scan_max = 1;
  if (scan_max > 127) scan_max = 127;
  return scan_max;
}

static int effective_scan_max(const motion_backend_cfg_t* cfg) {
  if (!cfg) return bounded_scan_max(1);

  int axis_max_node = 0;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((int)cfg->axis_node_id[axis] > axis_max_node) axis_max_node = (int)cfg->axis_node_id[axis];
  }

  const int scan_max = (cfg->can_scan_max_node > axis_max_node) ? cfg->can_scan_max_node : axis_max_node;
  return bounded_scan_max(scan_max);
}

static uint64_t mono_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static int ensure_enabled(struct motion_backend* b, size_t axis, mtr_mode_t mode) {
  if (b->axis_enabled[axis] && b->axis_mode[axis] == mode) return PD4C_OK;
  int rc = pd4c_ds402_enable_operation(&b->ctx[axis], (int8_t)mode, b->cfg.ds402_timeout_ms);
  if (rc == PD4C_OK) {
    b->axis_last_seen_ms[axis] = mono_ms();
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

static int bounded_sdo_timeout_ms(uint32_t timeout_ms) {
  if (timeout_ms == 0 || timeout_ms > INT32_MAX) return 500;
  return (int)timeout_ms;
}

static int read_node_identity(pd4c_ctx_t* ctx, uint32_t timeout_ms, uint32_t identity[4], uint8_t* failed_subidx) {
  if (!ctx || !identity || !failed_subidx) return -1;

  const int sdo_timeout_ms = bounded_sdo_timeout_ms(timeout_ms);
  pd4c_set_sdo_timeout(ctx, sdo_timeout_ms);

  for (uint8_t i = 0; i < 4; i++) {
    const uint8_t subidx = (uint8_t)(i + 1u);
    int rc = pd4c_sdo_read_u32(ctx, 0x1018u, subidx, &identity[i]);
    if (rc != PD4C_OK) {
      *failed_subidx = subidx;
      return rc;
    }
  }

  *failed_subidx = 0;
  return PD4C_OK;
}

static int read_node_id_crosscheck(pd4c_ctx_t* ctx, uint32_t timeout_ms, uint8_t* node_id) {
  if (!ctx || !node_id) return -1;

  const int sdo_timeout_ms = bounded_sdo_timeout_ms(timeout_ms);
  pd4c_set_sdo_timeout(ctx, sdo_timeout_ms);
  return pd4c_sdo_read_u8(ctx, 0x2009u, 0x00u, node_id);
}

static int discover_responsive_nodes(struct motion_backend* b) {
  if (!b) return 0;

  int responsive_count = 0;
  const int scan_max = effective_scan_max(&b->cfg);

  memset(b->node_discovery, 0, sizeof(b->node_discovery));
  memset(b->snap.can_discovered_nodes, 0, sizeof(b->snap.can_discovered_nodes));
  for (int node = 1; node <= scan_max; node++) {
    pd4c_ctx_t tmp;
    memset(&tmp, 0, sizeof(tmp));

    STC_LOG_INFO("probe open node=%d", node);
    if (pd4c_open(&tmp, b->cfg.can_if, (uint8_t)node) != PD4C_OK) continue;

    (void)pd4c_nmt_start(&tmp);
    const int responsive = verify_node_responsive(&tmp, b->cfg.can_timeout_ms);
    b->node_discovery[node].responsive = responsive ? 1 : 0;
    if (responsive) {
      uint32_t identity[4] = {0};
      uint8_t failed_subidx = 0;
      int identity_rc = read_node_identity(&tmp, b->cfg.can_timeout_ms, identity, &failed_subidx);
      if (identity_rc == PD4C_OK) {
        b->node_discovery[node].identity_ok = 1;
        b->node_discovery[node].vendor_id = identity[0];
        b->node_discovery[node].product_code = identity[1];
        b->node_discovery[node].revision = identity[2];
        b->node_discovery[node].serial = identity[3];
        STC_LOG_INFO(
          "can_discovery node=%d responsive=yes vendor_id=0x%08x product_code=0x%08x revision=0x%08x serial=0x%08x",
          node,
          b->node_discovery[node].vendor_id,
          b->node_discovery[node].product_code,
          b->node_discovery[node].revision,
          b->node_discovery[node].serial);
      } else {
        b->node_discovery[node].identity_ok = 0;
        b->node_discovery[node].identity_rc = identity_rc;
        STC_LOG_WARN(
          "can_discovery node=%d responsive=yes identity_read=failed failed_subidx=0x%02x rc=%d",
          node, failed_subidx, identity_rc);
      }

      uint8_t reported_node_id = 0;
      const int node_id_rc = read_node_id_crosscheck(&tmp, b->cfg.can_timeout_ms, &reported_node_id);
      if (node_id_rc == PD4C_OK) {
        if ((int)reported_node_id != node) {
          STC_LOG_WARN(
            "can_discovery node=%d node_id_object_0x2009=0x%02x mismatch=yes",
            node,
            reported_node_id);
        } else {
          STC_LOG_INFO(
            "can_discovery node=%d node_id_object_0x2009=0x%02x mismatch=no",
            node,
            reported_node_id);
        }
      } else {
        STC_LOG_WARN(
          "can_discovery node=%d node_id_object_0x2009=unavailable rc=%d",
          node,
          node_id_rc);
      }

      const int node_idx = responsive_count;
      if (node_idx < (int)(sizeof(b->snap.can_discovered_nodes) / sizeof(b->snap.can_discovered_nodes[0]))) {
        b->snap.can_discovered_nodes[node_idx] = (uint8_t)node;
      }
      responsive_count++;
    } else {
      STC_LOG_INFO("can_discovery node=%d responsive=no identity_read=skipped reason=liveness_failed", node);
    }
    pd4c_close(&tmp);
  }

  b->snap.can_discovered_devices = (uint8_t)responsive_count;
  return responsive_count;
}

int motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend) {
  if (!cfg || !out_backend) return -1;
  struct motion_backend* b = (struct motion_backend*)calloc(1, sizeof(*b));
  if (!b) return -1;
  b->cfg = *cfg;
  STC_LOG_INFO("CAN backend assumes interface '%s' is preconfigured externally (manual canup/systemd); skipping interface setup commands.",
    (cfg->can_if && cfg->can_if[0]) ? cfg->can_if : "(unset)");

  (void)discover_responsive_nodes(b);

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    uint8_t node_id = cfg->axis_node_id[axis];
    if (!node_id) continue;

    if (!node_in_discovery_list(&b->snap, node_id)) {
      STC_LOG_WARN("Configured axis %zu node %u was not discovered as responsive; skipping connection.", axis, node_id);
      continue;
    }

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
    b->axis_last_seen_ms[axis] = mono_ms();
    b->axis_last_healthcheck_ms[axis] = b->axis_last_seen_ms[axis];
    b->snap.last_rx_ms[axis] = b->axis_last_seen_ms[axis];
    b->snap.last_rx_ms[axis] = mono_ms();
    STC_LOG_INFO("connected axis=%zu node=%u", axis, node_id);
    if (b->node_discovery[node_id].identity_ok) {
      STC_LOG_INFO(
        "can_axis_connect axis=%zu configured_node=%u discovery_identity=ok vendor_id=0x%08x product_code=0x%08x revision=0x%08x serial=0x%08x",
        axis,
        node_id,
        b->node_discovery[node_id].vendor_id,
        b->node_discovery[node_id].product_code,
        b->node_discovery[node_id].revision,
        b->node_discovery[node_id].serial);
    } else {
      STC_LOG_INFO(
        "can_axis_connect axis=%zu configured_node=%u discovery_identity=unavailable reason=read_failed rc=%d",
        axis,
        node_id,
        b->node_discovery[node_id].identity_rc);
    }
  }
  STC_LOG_INFO("discovery summary: scanned %d, responsive %u, connected %u",
    effective_scan_max(&b->cfg), b->snap.can_discovered_devices, b->snap.can_connected_devices);
  refresh_can_status(b);
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
  const int responsive_count = discover_responsive_nodes(b);
  refresh_can_status(b);
  STC_LOG_INFO("discovery summary: scanned %d, responsive %d, connected %u",
    effective_scan_max(&b->cfg), responsive_count, b->snap.can_connected_devices);
  return responsive_count;
}

int motion_backend_apply_pp_move(motion_backend_t* backend, uint32_t axes_mask, const double targets_deg[STC_AXIS_COUNT]) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !targets_deg) return -1;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0 || !b->axis_connected[axis]) continue;
    if (ensure_enabled(b, axis, MTR_MODE_PP) != PD4C_OK) continue;
    int32_t target_counts = axis_deg_to_counts(&b->cfg.axis_cfg[axis], targets_deg[axis]);
    uint16_t pv = b->cfg.axis_cfg[axis].pp_profile_vel;
    if (pd4c_pp_move(&b->ctx[axis], target_counts, pv, 0, b->cfg.ds402_timeout_ms) == PD4C_OK) {
      b->axis_last_seen_ms[axis] = mono_ms();
    }
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
    if (pd4c_rpdo3_send_target_velocity(&b->ctx[axis], (int16_t)tv_raw) == PD4C_OK) {
      b->axis_last_seen_ms[axis] = mono_ms();
    }
  }
  return 0;
}

int motion_backend_quick_stop(motion_backend_t* backend, uint32_t axes_mask) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0 || !b->axis_connected[axis]) continue;
    if (pd4c_ds402_quick_stop(&b->ctx[axis], (int8_t)b->axis_mode[axis]) == PD4C_OK) {
      b->axis_last_seen_ms[axis] = mono_ms();
    }
    b->axis_enabled[axis] = 0;
  }
  return 0;
}

int motion_backend_clear_faults(motion_backend_t* backend, uint32_t axes_mask) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0 || !b->axis_connected[axis]) continue;

    const int8_t mode = (b->axis_mode[axis] != MTR_MODE_NONE)
      ? (int8_t)b->axis_mode[axis]
      : (int8_t)MTR_MODE_PP;

    const int rc = pd4c_ds402_fault_reset(&b->ctx[axis], mode, b->cfg.ds402_timeout_ms);
    if (rc == PD4C_OK) {
      b->axis_last_seen_ms[axis] = mono_ms();
      b->axis_enabled[axis] = 0;
    } else {
      STC_LOG_WARN("axis=%zu fault reset failed rc=%d", axis, rc);
    }
  }

  return 0;
}

int motion_backend_set_laser(motion_backend_t* backend, uint8_t laser_id, uint8_t on_off) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;
  STC_LOG_INFO("laser_stub id=%u on_off=%u (no physical laser hardware integrated)", laser_id, on_off);
  return 0;
}

int motion_backend_get_snapshot(motion_backend_t* backend, motion_snapshot_t* snapshot) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !snapshot) return -1;
  uint64_t now = mono_ms();
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!b->axis_connected[axis]) continue;

    int poll_rc = 0;
    do {
      poll_rc = pd4c_poll(&b->ctx[axis], 0);
      if (poll_rc >= PD4C_OK) {
        const pd4c_telemetry_t* t = pd4c_telem(&b->ctx[axis]);
        b->snap.pos_raw[axis] = t->pos_actual;
        b->snap.vel_raw[axis] = t->vel_actual;
        b->snap.status_word[axis] = t->statusword;
        b->snap.mode_display[axis] = t->mode_display;
        b->axis_last_seen_ms[axis] = now;
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
    } while (poll_rc >= PD4C_OK && poll_rc != PD4C_ETIMEDOUT);

    if (poll_rc == PD4C_ETIMEDOUT && b->cfg.pp_sdo_diag) {
      const uint64_t elapsed_since_check_ms = now - b->axis_last_healthcheck_ms[axis];
      if (elapsed_since_check_ms >= b->cfg.can_timeout_ms && verify_node_responsive(&b->ctx[axis], b->cfg.can_timeout_ms)) {
        b->axis_last_seen_ms[axis] = now;
        b->axis_last_healthcheck_ms[axis] = now;
      }
    }
    b->snap.last_rx_ms[axis] = b->axis_last_seen_ms[axis];
  }
  refresh_can_status(b);
  *snapshot = b->snap;
  return 0;
}
