/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Adapts the generic motion backend API used by stc_server to real CAN hardware.
 *
 * Design notes:
 *   Generic CAN transport backend that dispatches axis commands to the
 *   configured per-axis driver (PD4-C CANopen/DS402 or MYACTUATOR protocol).
 *   Performs node discovery and identity reads for diagnostics, then keeps
 *   expected-vs-responsive availability and CAN health in snapshot state.
 */

#define _POSIX_C_SOURCE 200809L

#include "motion_backend.h"
#include "myact_backend.h"

#include "pd4c.h"
#include "x8_25.h"
#include "myact_socketcan.h"
#include "stc_log.h"
#include "stc_diag.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

int myact_backend_quiesce_active_replies(myact_ctx_t* ctx, int drain_ms) {
  if (!ctx) return MYACT_EINVAL;
  const int cmds[] = { MYACT_CMD_STATUS1, MYACT_CMD_STATUS2, MYACT_CMD_STATUS3, MYACT_CMD_READ_ANGLE_MULTI };
  for (size_t i = 0; i < (sizeof(cmds) / sizeof(cmds[0])); i++) {
    (void)myact_active_reply_config(ctx, (uint8_t)cmds[i], false, 0);
  }

  const uint64_t start_ms = ctx->io.time_ms ? ctx->io.time_ms(ctx->io.user) : 0;
  const uint64_t budget_ms = (drain_ms > 0) ? (uint64_t)drain_ms : 0u;
  int drained = 0;
  while (budget_ms > 0) {
    const uint64_t now = ctx->io.time_ms ? ctx->io.time_ms(ctx->io.user) : start_ms;
    if ((now - start_ms) >= budget_ms) break;
    const int rc = myact_poll(ctx, 0);
    if (rc == MYACT_ETIMEDOUT) break;
    if (rc < 0) break;
    drained++;
  }
  return drained;
}

typedef enum {
  MTR_MODE_NONE = 0,
  /* PD4-C velocity mode command path. */
  MTR_MODE_PV = PD4C_MODE_VELOCITY,
  MTR_MODE_PP = PD4C_MODE_PROFILE_POSITION
} mtr_mode_t;

typedef axis_cfg_t stc_axis_cfg_t;
typedef motion_snapshot_t stc_axis_telem_t;

typedef struct {
  const char* name;

  int (*init)(void* driver_ctx, const stc_axis_cfg_t* cfg);
  int (*poll)(void* driver_ctx, stc_axis_telem_t* out);
  int (*stop)(void* driver_ctx);
  int (*shutdown)(void* driver_ctx);

  int (*move_abs)(void* driver_ctx, double target_deg, double speed_deg_s);
  int (*move_rel)(void* driver_ctx, double delta_deg, double speed_deg_s);
  int (*speed)(void* driver_ctx, double speed_deg_s);
} stc_axis_driver_ops_t;

typedef struct {
  stc_axis_driver_t driver;
  const stc_axis_driver_ops_t* ops;
  int connected;
  int enabled;
  int faulted;
  uint64_t last_rx_ms;
  uint64_t last_cmd_ms;
  uint64_t last_healthcheck_ms;
  union {
    struct {
      pd4c_ctx_t ctx;
      int mode;
    } pd4c;
    struct {
      myact_ctx_t ctx;
      myact_socketcan_t io;
      int brake_released;
      int runmode;
      double raw_zero_deg;
      double last_raw_angle_deg;
      double last_physical_deg;
      double last_speed_deg_s;
      uint16_t error_flags;
      double bus_voltage_v;
      int motor_temp_c;
      int active_status2_enabled;
    } x8;
  } u;
} axis_rt_t;

static int pd4c_axis_init(void* driver_ctx, const stc_axis_cfg_t* cfg) { (void)driver_ctx; (void)cfg; return PD4C_OK; }
static int pd4c_axis_poll(void* driver_ctx, stc_axis_telem_t* out) { (void)out; return pd4c_poll((pd4c_ctx_t*)driver_ctx, 0); }
static int pd4c_axis_stop(void* driver_ctx) { return pd4c_ds402_quick_stop((pd4c_ctx_t*)driver_ctx, (int8_t)MTR_MODE_PP); }
static int pd4c_axis_shutdown(void* driver_ctx) { pd4c_close((pd4c_ctx_t*)driver_ctx); return PD4C_OK; }
static int pd4c_axis_move_abs(void* driver_ctx, double target_deg, double speed_deg_s) { (void)driver_ctx; (void)target_deg; (void)speed_deg_s; return PD4C_EINVAL; }
static int pd4c_axis_move_rel(void* driver_ctx, double delta_deg, double speed_deg_s) { (void)driver_ctx; (void)delta_deg; (void)speed_deg_s; return PD4C_EINVAL; }
static int pd4c_axis_speed(void* driver_ctx, double speed_deg_s) {
  const int32_t tv_raw = (int32_t)llround(speed_deg_s);
  return pd4c_rpdo3_send_target_velocity((pd4c_ctx_t*)driver_ctx, (int16_t)tv_raw);
}

static int myact_axis_init(void* driver_ctx, const stc_axis_cfg_t* cfg) { (void)driver_ctx; (void)cfg; return MYACT_OK; }
static int myact_axis_poll(void* driver_ctx, stc_axis_telem_t* out) { (void)out; return myact_poll((myact_ctx_t*)driver_ctx, 0); }
static int myact_axis_stop(void* driver_ctx) { return myact_stop((myact_ctx_t*)driver_ctx); }
static int myact_axis_shutdown(void* driver_ctx) { return myact_shutdown_send((myact_ctx_t*)driver_ctx); }
static int myact_axis_move_abs(void* driver_ctx, double target_deg, double speed_deg_s) {
  myact_basic_feedback_t fb;
  return myact_position_abs((myact_ctx_t*)driver_ctx, (uint16_t)llround(speed_deg_s), myact_deg_to_raw_0p01(target_deg), &fb);
}
static int myact_axis_move_rel(void* driver_ctx, double delta_deg, double speed_deg_s) {
  myact_basic_feedback_t fb;
  return myact_position_inc((myact_ctx_t*)driver_ctx, (uint16_t)llround(speed_deg_s), myact_deg_to_raw_0p01(delta_deg), &fb);
}
static int myact_axis_speed(void* driver_ctx, double speed_deg_s) {
  myact_basic_feedback_t fb;
  return myact_speed((myact_ctx_t*)driver_ctx, myact_speed_dps_to_raw(speed_deg_s), &fb);
}

static const stc_axis_driver_ops_t pd4c_ops = {
    .name = "pd4c",
    .init = pd4c_axis_init,
    .poll = pd4c_axis_poll,
    .stop = pd4c_axis_stop,
    .shutdown = pd4c_axis_shutdown,
    .move_abs = pd4c_axis_move_abs,
    .move_rel = pd4c_axis_move_rel,
    .speed = pd4c_axis_speed,
};

static const stc_axis_driver_ops_t myact_ops = {
    .name = "myact_x8_25",
    .init = myact_axis_init,
    .poll = myact_axis_poll,
    .stop = myact_axis_stop,
    .shutdown = myact_axis_shutdown,
    .move_abs = myact_axis_move_abs,
    .move_rel = myact_axis_move_rel,
    .speed = myact_axis_speed,
};

static void* axis_driver_ctx(axis_rt_t* axis) {
  if (!axis) return NULL;
  if (axis->driver == STC_AXIS_DRIVER_PD4C) return &axis->u.pd4c.ctx;
  if (axis->driver == STC_AXIS_DRIVER_MYACT_X8_25) return &axis->u.x8.ctx;
  return NULL;
}

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
  axis_rt_t axis[STC_AXIS_COUNT];
  int32_t axis_last_cmd_pos_raw[STC_AXIS_COUNT];
  int32_t axis_last_cmd_vel_raw[STC_AXIS_COUNT];
  uint8_t axis_cmd_ack_pending[STC_AXIS_COUNT];
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
static void expected_connected_counts(const struct motion_backend* b, int* expected_axes, int* connected_expected_axes) {
  int expected = 0;
  int connected = 0;
  if (b) {
    for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
      if (!b->cfg.axis_enabled[axis] || b->cfg.axis_cfg[axis].driver == STC_AXIS_DRIVER_DISABLED) continue;
      expected++;
      if (b->axis[axis].connected) connected++;
    }
  }
  if (expected_axes) *expected_axes = expected;
  if (connected_expected_axes) *connected_expected_axes = connected;
}

static int compute_can_ok(const struct motion_backend* b) {
  if (!b) return 0;

  int expected_axes = 0;
  int connected_expected_axes = 0;
  expected_connected_counts(b, &expected_axes, &connected_expected_axes);

  if (expected_axes == 0) return 0;
  return connected_expected_axes == expected_axes;
}

static uint64_t mono_ms(void);

static void refresh_can_status(struct motion_backend* b) {
  if (!b) return;
  /* Aggregate per-axis connectivity into global CAN health for telemetry/state gating. */
  b->snap.can_ok = compute_can_ok(b);
  b->snap.capabilities.supports_native_a2_speed = 0;
  b->snap.capabilities.supports_native_absolute_position = 0;
  b->snap.backend_mode = 2;
  b->snap.transport_status = 1;
}

int myact_backend_discover_axis(motion_backend_t* backend, size_t axis, const char* can_if, uint8_t node_id) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || axis >= STC_AXIS_COUNT || !can_if || !can_if[0] || node_id == 0) return -1;

  STC_LOG_INFO("MYACT axis%zu node=%u opening SocketCAN on %s", axis, (unsigned)node_id, can_if);
  int rc = myact_socketcan_open(&b->axis[axis].u.x8.io, can_if, node_id);
  if (rc != MYACT_OK) {
    STC_LOG_ERROR("MYACT axis%zu discovery FAILED rc=%d reason=socketcan_open expected_rx=0x%03X", axis, rc, (unsigned)(0x240u + node_id));
    return rc;
  }

  STC_LOG_INFO("MYACT socketcan axis%zu node=%u tx_single=0x%03X rx_single=0x%03X tx_motion=0x%03X rx_motion=0x%03X",
               axis, (unsigned)node_id,
               (unsigned)b->axis[axis].u.x8.io.tx_single_id, (unsigned)b->axis[axis].u.x8.io.rx_single_id,
               (unsigned)b->axis[axis].u.x8.io.tx_motion_id, (unsigned)b->axis[axis].u.x8.io.rx_motion_id);
  if (myact_init(&b->axis[axis].u.x8.ctx, node_id, &b->axis[axis].u.x8.io.io) != MYACT_OK) {
    myact_socketcan_close(&b->axis[axis].u.x8.io);
    STC_LOG_ERROR("MYACT axis%zu discovery FAILED rc=%d reason=myact_init expected_rx=0x%03X", axis, MYACT_EIO, (unsigned)(0x240u + node_id));
    return MYACT_EIO;
  }
  const int drained = myact_backend_quiesce_active_replies(&b->axis[axis].u.x8.ctx, 150);
  if (drained > 0) {
    STC_LOG_INFO("MYACT axis%zu quiesce: disabled active replies, drained %d stale frames", axis, drained);
  } else {
    STC_LOG_INFO("MYACT axis%zu quiesce: disabled active replies", axis);
  }

  STC_LOG_INFO("MYACT axis%zu discovery: sending read_model", axis);
  char model[8] = {0};
  rc = myact_read_model(&b->axis[axis].u.x8.ctx, model);
  if (rc != MYACT_OK) {
    const int poll_rc = myact_poll(&b->axis[axis].u.x8.ctx, 0);
    if (poll_rc >= 0) {
      b->axis[axis].last_rx_ms = mono_ms();
      b->snap.last_rx_ms[axis] = b->axis[axis].last_rx_ms;
    }
    myact_status2_t st2;
    memset(&st2, 0, sizeof(st2));
    const int st2_rc = myact_read_status2(&b->axis[axis].u.x8.ctx, &st2);
    int32_t angle_raw_0p01 = 0;
    const int ang_rc = myact_read_angle_multi(&b->axis[axis].u.x8.ctx, &angle_raw_0p01);
    if (st2_rc == MYACT_OK || ang_rc == MYACT_OK) {
      STC_LOG_WARN("MYACT axis%zu model read timed out, but status2/angle replied; accepting configured myact_x8_25 node as connected", axis);
      strncpy(model, "unknown", sizeof(model) - 1);
    } else {
      STC_LOG_ERROR("MYACT axis%zu discovery FAILED rc=%d reason=myact_read_model expected_rx=0x%03X", axis, rc, (unsigned)(0x240u + node_id));
      myact_socketcan_close(&b->axis[axis].u.x8.io);
      return rc;
    }
  }

  STC_LOG_INFO("MYACT axis%zu discovery OK model=%s", axis, model);
  b->axis[axis].connected = 1;
  b->axis[axis].enabled = 1;
  b->axis[axis].last_rx_ms = mono_ms();
  b->axis[axis].last_healthcheck_ms = b->axis[axis].last_rx_ms;
  b->snap.last_rx_ms[axis] = b->axis[axis].last_rx_ms;
  b->snap.axis_available[axis] = 1;
  return MYACT_OK;
}

static int axis_selected(uint32_t axes_mask, size_t axis) {
  return (axes_mask & (1u << axis)) != 0u;
}

static int axis_ready_for_motion(const struct motion_backend* b, size_t axis) {
  if (!b) return 0;
  return b->axis[axis].connected && b->axis[axis].ops && axis_driver_ctx((axis_rt_t*)&b->axis[axis]);
}

static int stop_touched_axes(struct motion_backend* b, uint32_t touched_mask) {
  if (!b) return -1;
  int stop_rc = 0;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!axis_selected(touched_mask, axis)) continue;
    if (!axis_ready_for_motion(b, axis)) continue;
    if (b->axis[axis].ops->stop && b->axis[axis].ops->stop(axis_driver_ctx(&b->axis[axis])) != 0) {
      stop_rc = -1;
    }
    if (b->axis[axis].driver == STC_AXIS_DRIVER_PD4C) b->axis[axis].enabled = 0;
  }
  return stop_rc;
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

static double axis_gear_ratio_or_default(const struct motion_backend* b, size_t axis) {
  if (!b) return MYACT_X8_25_GEAR_RATIO;
  const double gr = b->cfg.axis_cfg[axis].gear_ratio;
  return (gr > 0.0) ? gr : MYACT_X8_25_GEAR_RATIO;
}

static double axis_output_to_motor_deg(const struct motion_backend* b, size_t axis, double output_deg) {
  return myact_output_deg_to_motor_deg(output_deg, axis_gear_ratio_or_default(b, axis));
}

static uint64_t mono_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static int ensure_enabled(struct motion_backend* b, size_t axis, mtr_mode_t mode) {
  if (b->axis[axis].enabled && (mtr_mode_t)b->axis[axis].u.pd4c.mode == mode) return PD4C_OK;
  int rc = pd4c_ds402_enable_operation(&b->axis[axis].u.pd4c.ctx, (int8_t)mode, b->cfg.ds402_timeout_ms);
  if (rc == PD4C_OK) {
    b->axis[axis].last_rx_ms = mono_ms();
    b->axis[axis].enabled = 1;
    b->axis[axis].u.pd4c.mode = mode;
  }
  return rc;
}

static int verify_node_responsive(pd4c_ctx_t* ctx, uint32_t timeout_ms) {
  /* Liveness probe: bounded SDO read of identity vendor entry (0x1018:01). */
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

  /* Identity tuple readout for diagnostics: vendor/product/revision/serial. */
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
  if (!b->cfg.can_discovery_enabled) {
    memset(b->node_discovery, 0, sizeof(b->node_discovery));
    memset(b->snap.can_discovered_nodes, 0, sizeof(b->snap.can_discovered_nodes));
    b->snap.can_discovered_devices = 0;
    return 0;
  }

  int responsive_count = 0;
  const int scan_max = effective_scan_max(&b->cfg);

  memset(b->node_discovery, 0, sizeof(b->node_discovery));
  memset(b->snap.can_discovered_nodes, 0, sizeof(b->snap.can_discovered_nodes));
  /* Discover responsive nodes before axis attach so startup can skip silent nodes cleanly. */
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

int can_motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend) {
  if (!cfg || !out_backend) return -1;
  struct motion_backend* b = (struct motion_backend*)calloc(1, sizeof(*b));
  if (!b) return -1;
  b->cfg = *cfg;
  STC_LOG_INFO("CAN backend assumes interface '%s' is preconfigured externally (manual canup/systemd); skipping interface setup commands.",
    (cfg->can_if && cfg->can_if[0]) ? cfg->can_if : "(unset)");

  STC_LOG_INFO("CAN backend discovery starts");
  (void)discover_responsive_nodes(b);

  /* Attach only configured/expected axes; keep missing nodes visible via availability flags. */
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    uint8_t node_id = cfg->axis_node_id[axis];
    const stc_axis_driver_t driver = cfg->axis_cfg[axis].driver;
    if (!cfg->axis_enabled[axis] || driver == STC_AXIS_DRIVER_DISABLED) continue;
    if (!node_id) continue;

    if (cfg->can_discovery_enabled &&
        driver == STC_AXIS_DRIVER_PD4C &&
        !node_in_discovery_list(&b->snap, node_id)) {
      STC_LOG_WARN("Configured axis %zu node %u was not discovered as responsive; skipping connection.", axis, node_id);
      continue;
    }

    b->axis[axis].driver = driver;
    b->axis[axis].ops = (driver == STC_AXIS_DRIVER_PD4C) ? &pd4c_ops
                      : (driver == STC_AXIS_DRIVER_MYACT_X8_25) ? &myact_ops
                      : NULL;
    switch (driver) {
      case STC_AXIS_DRIVER_PD4C: {
        int rc = pd4c_open(&b->axis[axis].u.pd4c.ctx, cfg->can_if, node_id);
        if (rc != PD4C_OK) break;
        (void)pd4c_nmt_start(&b->axis[axis].u.pd4c.ctx);
        if (!verify_node_responsive(&b->axis[axis].u.pd4c.ctx, cfg->can_timeout_ms)) {
          STC_LOG_WARN("CAN node %u (axis %zu) opened but failed liveness verification; closing context.", node_id, axis);
          pd4c_close(&b->axis[axis].u.pd4c.ctx);
          break;
        }
        b->axis[axis].connected = 1;
        b->snap.axis_available[axis] = 1;
        b->snap.can_connected_devices++;
        b->axis[axis].last_rx_ms = mono_ms();
        b->axis[axis].last_healthcheck_ms = b->axis[axis].last_rx_ms;
        b->snap.last_rx_ms[axis] = b->axis[axis].last_rx_ms;
        STC_LOG_INFO("axis%zu driver=pd4c node=%u connected", axis, node_id);
        STC_LOG_INFO("axis%zu pd4c init OK", axis);
      } break;
      case STC_AXIS_DRIVER_MYACT_X8_25: {
        int discovery_rc = myact_backend_discover_axis((motion_backend_t*)b, axis, cfg->can_if, node_id);
        if (discovery_rc == MYACT_OK) {
          b->snap.can_connected_devices++;
          int32_t angle_raw_0p01 = 0;
          (void)myact_read_angle_multi(&b->axis[axis].u.x8.ctx, &angle_raw_0p01);
          b->axis[axis].u.x8.last_raw_angle_deg = (double)angle_raw_0p01 / 100.0;
          if (cfg->axis_cfg[axis].zero_on_start) {
            b->axis[axis].u.x8.raw_zero_deg = b->axis[axis].u.x8.last_raw_angle_deg;
            STC_LOG_INFO("axis%zu zero_on_start raw_zero_deg=%.2f", axis, b->axis[axis].u.x8.raw_zero_deg);
          }
          if (cfg->axis_cfg[axis].active_status2_enable) {
            const uint16_t interval_10ms = (uint16_t)((cfg->axis_cfg[axis].active_status2_interval_ms > 0 ? cfg->axis_cfg[axis].active_status2_interval_ms : 50) / 10);
            const int ar_rc = myact_active_reply_config(&b->axis[axis].u.x8.ctx, MYACT_CMD_STATUS2, true, interval_10ms);
            if (ar_rc == MYACT_OK) {
              b->axis[axis].u.x8.active_status2_enabled = 1;
              STC_LOG_INFO("axis%zu active_reply 9C enabled interval_ms=%u", axis, (unsigned)(interval_10ms * 10u));
            } else {
              b->axis[axis].u.x8.active_status2_enabled = 0;
              STC_LOG_WARN("axis%zu active_reply 9C enable failed; polling fallback active", axis);
            }
          }
          STC_LOG_INFO("axis%zu driver=myact_x8_25 node=%u connected", axis, node_id);
          STC_LOG_INFO("axis%zu myact init OK", axis);
        }
      } break;
      case STC_AXIS_DRIVER_DISABLED:
      default:
        b->snap.axis_available[axis] = 0;
        break;
    }
    if (!b->axis[axis].connected) continue;
    if (cfg->can_discovery_enabled && b->node_discovery[node_id].identity_ok) {
      STC_LOG_INFO(
        "can_axis_connect axis=%zu configured_node=%u discovery_identity=ok vendor_id=0x%08x product_code=0x%08x revision=0x%08x serial=0x%08x",
        axis,
        node_id,
        b->node_discovery[node_id].vendor_id,
        b->node_discovery[node_id].product_code,
        b->node_discovery[node_id].revision,
        b->node_discovery[node_id].serial);
    } else if (cfg->can_discovery_enabled) {
      STC_LOG_INFO(
        "can_axis_connect axis=%zu configured_node=%u discovery_identity=unavailable reason=read_failed rc=%d",
        axis,
        node_id,
        b->node_discovery[node_id].identity_rc);
    }
  }
  if (cfg->can_discovery_enabled) {
    STC_LOG_INFO("discovery summary: scanned %d, responsive %u, connected %u",
      effective_scan_max(&b->cfg), b->snap.can_discovered_devices, b->snap.can_connected_devices);
  }
  refresh_can_status(b);
  {
    int expected_axes = 0;
    int connected_expected_axes = 0;
    expected_connected_counts(b, &expected_axes, &connected_expected_axes);
    STC_LOG_INFO("CAN startup inventory expected_motors=%d connected_motors=%d", expected_axes, connected_expected_axes);
  }
  *out_backend = b;
  return 0;
}

void can_motion_backend_stop(motion_backend_t* backend) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!b->axis[axis].connected) continue;
    if (b->axis[axis].ops && b->axis[axis].ops->shutdown) (void)b->axis[axis].ops->shutdown(axis_driver_ctx(&b->axis[axis]));
    if (b->axis[axis].driver == STC_AXIS_DRIVER_MYACT_X8_25) myact_socketcan_close(&b->axis[axis].u.x8.io);
  }
  free(b);
}

int can_motion_backend_discover(motion_backend_t* backend) {
  /* Re-scan bus on demand to refresh responsive-device diagnostics. */
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;
  if (!b->cfg.can_discovery_enabled) return 0;
  const int responsive_count = discover_responsive_nodes(b);
  refresh_can_status(b);
  STC_LOG_INFO("discovery summary: scanned %d, responsive %d, connected %u",
    effective_scan_max(&b->cfg), responsive_count, b->snap.can_connected_devices);
  return responsive_count;
}

int can_motion_backend_apply_pp_move(motion_backend_t* backend, uint32_t axes_mask, const double targets_deg[STC_AXIS_COUNT]) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !targets_deg) return -1;

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!axis_selected(axes_mask, axis)) continue;
    if (!axis_ready_for_motion(b, axis)) return -1;
    if (b->axis[axis].driver != STC_AXIS_DRIVER_PD4C && b->axis[axis].driver != STC_AXIS_DRIVER_MYACT_X8_25) return -1;
  }

  uint32_t touched_mask = 0u;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!axis_selected(axes_mask, axis)) continue;
    if (b->axis[axis].driver == STC_AXIS_DRIVER_PD4C) {
      if (ensure_enabled(b, axis, MTR_MODE_PP) != PD4C_OK) {
        (void)stop_touched_axes(b, touched_mask);
        return -1;
      }
      int32_t target_counts = axis_deg_to_counts(&b->cfg.axis_cfg[axis], targets_deg[axis]);
      uint16_t pv = b->cfg.axis_cfg[axis].pp_profile_vel;
      if (pd4c_pp_move(&b->axis[axis].u.pd4c.ctx, target_counts, pv, 0, b->cfg.ds402_timeout_ms) != PD4C_OK) {
        (void)stop_touched_axes(b, touched_mask);
        return -1;
      }
      b->axis_last_cmd_pos_raw[axis] = target_counts;
    } else {
      const double target_motor_deg = axis_output_to_motor_deg(b, axis, targets_deg[axis]);
      const double max_speed_output_dps = (b->cfg.axis_cfg[axis].max_speed_deg_s > 0.0) ? b->cfg.axis_cfg[axis].max_speed_deg_s : 30.0;
      const double max_speed_motor_dps = fabs(axis_output_to_motor_deg(b, axis, max_speed_output_dps));
      if (!b->axis[axis].ops || !b->axis[axis].ops->move_abs ||
          b->axis[axis].ops->move_abs(axis_driver_ctx(&b->axis[axis]), target_motor_deg, max_speed_motor_dps) != 0) {
        (void)stop_touched_axes(b, touched_mask);
        return -1;
      }
      b->axis_last_cmd_pos_raw[axis] = myact_deg_to_raw_0p01(target_motor_deg);
    }

    touched_mask |= (1u << axis);
    const uint64_t tms = mono_ms();
    b->axis[axis].last_rx_ms = tms;
    b->axis[axis].last_cmd_ms = tms;
    b->axis_cmd_ack_pending[axis] = 1u;
  }
  return 0;
}

int can_motion_backend_apply_speed(motion_backend_t* backend, uint32_t axes_mask, const double speeds_deg_s[STC_AXIS_COUNT]) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !speeds_deg_s) return -1;

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!axis_selected(axes_mask, axis)) continue;
    if (!axis_ready_for_motion(b, axis)) return -1;
    if (b->axis[axis].driver != STC_AXIS_DRIVER_PD4C && b->axis[axis].driver != STC_AXIS_DRIVER_MYACT_X8_25) return -1;
  }

  uint32_t touched_mask = 0u;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!axis_selected(axes_mask, axis)) continue;
    if (b->axis[axis].driver == STC_AXIS_DRIVER_PD4C) {
      /* Continuous speed stream relies on watchdog refresh from repeated commands. */
      if (ensure_enabled(b, axis, MTR_MODE_PV) != PD4C_OK) {
        (void)stop_touched_axes(b, touched_mask);
        return -1;
      }
    }
    int clipped = 0;
    int32_t tv_raw = axis_deg_s_to_target_vel_units(&b->cfg.axis_cfg[axis], speeds_deg_s[axis], &clipped);
    if (tv_raw > INT16_MAX) tv_raw = INT16_MAX;
    if (tv_raw < INT16_MIN) tv_raw = INT16_MIN;
    const double speed_cmd = (b->axis[axis].driver == STC_AXIS_DRIVER_PD4C) ? (double)tv_raw : axis_output_to_motor_deg(b, axis, speeds_deg_s[axis]);
    const int cmd_ok = (b->axis[axis].ops && b->axis[axis].ops->speed)
      ? (b->axis[axis].ops->speed(axis_driver_ctx(&b->axis[axis]), speed_cmd) == 0)
      : 0;
    if (!cmd_ok) {
      (void)stop_touched_axes(b, touched_mask);
      return -1;
    }
    touched_mask |= (1u << axis);
    const uint64_t tms = mono_ms();
    b->axis[axis].last_rx_ms = tms;
    b->axis[axis].last_cmd_ms = tms;
    b->axis_last_cmd_vel_raw[axis] = tv_raw;
    b->axis_cmd_ack_pending[axis] = 1u;
  }
  return 0;
}

int can_motion_backend_apply_motion(motion_backend_t* backend, uint32_t axes_mask, const stc_axis_motion_kind_t kind[STC_AXIS_COUNT], const double values_deg_or_dps[STC_AXIS_COUNT], const double speed_limit_deg_s[STC_AXIS_COUNT]) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !kind || !values_deg_or_dps || !speed_limit_deg_s) return -1;

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!axis_selected(axes_mask, axis)) continue;
    if (!axis_ready_for_motion(b, axis)) return -1;
  }

  uint32_t touched_mask = 0u;
  int sent_count = 0;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!axis_selected(axes_mask, axis)) continue;
    int rc = -1;

    if (b->axis[axis].driver == STC_AXIS_DRIVER_MYACT_X8_25) {
      myact_basic_feedback_t fb;
      memset(&fb, 0, sizeof(fb));
      const uint8_t node = b->cfg.axis_node_id[axis];
      switch (kind[axis]) {
        case STC_AXIS_MOTION_REL: {
          const double delta_motor_deg = axis_output_to_motor_deg(b, axis, values_deg_or_dps[axis]);
          const double max_speed_motor_dps = fabs(axis_output_to_motor_deg(b, axis, speed_limit_deg_s[axis]));
          const int32_t raw_delta = myact_deg_to_raw_0p01(delta_motor_deg);
          const uint16_t max_speed_dps = (uint16_t)llround(max_speed_motor_dps);
          STC_LOG_INFO("MYACT motion axis%zu node=%u mode=relative value=%.3f deg profile_speed=%.3f deg/s",
            axis, node, values_deg_or_dps[axis], speed_limit_deg_s[axis]);
          rc = myact_position_inc(&b->axis[axis].u.x8.ctx, max_speed_dps, raw_delta, &fb);
          break;
        }
        case STC_AXIS_MOTION_SPEED: {
          const double speed_motor_dps = axis_output_to_motor_deg(b, axis, values_deg_or_dps[axis]);
          const int32_t raw_speed = myact_speed_dps_to_raw(speed_motor_dps);
          STC_LOG_INFO("MYACT motion axis%zu node=%u mode=speed value=%.3f deg/s profile_speed=%.3f deg/s",
            axis, node, values_deg_or_dps[axis], speed_limit_deg_s[axis]);
          rc = myact_speed(&b->axis[axis].u.x8.ctx, raw_speed, &fb);
          break;
        }
        case STC_AXIS_MOTION_ABS:
        default: {
          const double abs_motor_deg = axis_output_to_motor_deg(b, axis, values_deg_or_dps[axis]);
          const double max_speed_motor_dps = fabs(axis_output_to_motor_deg(b, axis, speed_limit_deg_s[axis]));
          const int32_t raw_abs = myact_deg_to_raw_0p01(abs_motor_deg);
          const uint16_t max_speed_dps = (uint16_t)llround(max_speed_motor_dps);
          STC_LOG_INFO("MYACT motion axis%zu node=%u mode=absolute value=%.3f deg profile_speed=%.3f deg/s",
            axis, node, values_deg_or_dps[axis], speed_limit_deg_s[axis]);
          rc = myact_position_abs(&b->axis[axis].u.x8.ctx, max_speed_dps, raw_abs, &fb);
          break;
        }
      }
      if (rc != MYACT_OK) {
        STC_LOG_ERROR("MYACT command axis%zu FAILED rc=%d reason=%s", axis, rc, myact_strerror(rc));
        (void)stop_touched_axes(b, touched_mask);
        return -1;
      }
      const uint16_t err_flags = b->axis[axis].u.x8.ctx.cache.status1.valid ? b->axis[axis].u.x8.ctx.cache.status1.error_flags : 0u;
      STC_LOG_INFO("MYACT command axis%zu OK reply_pos=%d reply_vel=%d temp=%d err=0x%04x",
        axis, fb.angle_deg, fb.speed_dps, fb.motor_temp_c, (unsigned)err_flags);
      b->axis_last_cmd_pos_raw[axis] = myact_deg_to_raw_0p01((double)fb.angle_deg);
      b->axis_last_cmd_vel_raw[axis] = fb.speed_dps;
    } else if (b->axis[axis].driver == STC_AXIS_DRIVER_PD4C) {
      switch (kind[axis]) {
        case STC_AXIS_MOTION_SPEED: {
          double speed_cmd[STC_AXIS_COUNT] = {0};
          speed_cmd[axis] = values_deg_or_dps[axis];
          rc = can_motion_backend_apply_speed(backend, (1u << axis), speed_cmd);
          break;
        }
        case STC_AXIS_MOTION_REL:
        case STC_AXIS_MOTION_ABS:
        default: {
          double target_cmd[STC_AXIS_COUNT] = {0};
          target_cmd[axis] = (kind[axis] == STC_AXIS_MOTION_REL) ? ((double)b->snap.pos_raw[axis] + values_deg_or_dps[axis]) : values_deg_or_dps[axis];
          rc = can_motion_backend_apply_pp_move(backend, (1u << axis), target_cmd);
          break;
        }
      }
      if (rc != 0) return -1;
    } else {
      return -1;
    }

    touched_mask |= (1u << axis);
    sent_count++;
    const uint64_t tms = mono_ms();
    b->axis[axis].last_rx_ms = tms;
    b->axis[axis].last_cmd_ms = tms;
    b->axis_cmd_ack_pending[axis] = 1u;
  }
  return (sent_count > 0) ? 0 : -1;
}

int can_motion_backend_quick_stop(motion_backend_t* backend, uint32_t axes_mask) {
  /* Broadcast-safe quick-stop path across selected axes; recovery remains caller-managed. */
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0 || !b->axis[axis].connected) continue;
    if (b->axis[axis].driver == STC_AXIS_DRIVER_PD4C) {
      if (b->axis[axis].ops && b->axis[axis].ops->stop && b->axis[axis].ops->stop(axis_driver_ctx(&b->axis[axis])) == PD4C_OK) b->axis[axis].last_rx_ms = mono_ms();
      b->axis[axis].enabled = 0;
    } else if (b->axis[axis].driver == STC_AXIS_DRIVER_MYACT_X8_25) {
      if (b->axis[axis].ops && b->axis[axis].ops->stop) (void)b->axis[axis].ops->stop(axis_driver_ctx(&b->axis[axis]));
    }
  }
  return 0;
}

int can_motion_backend_clear_faults(motion_backend_t* backend, uint32_t axes_mask) {
  /* Fault reset helper across mixed drivers, preserving per-axis success accounting. */
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;

  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if ((axes_mask & (1u << axis)) == 0 || !b->axis[axis].connected) continue;
    if (b->axis[axis].driver == STC_AXIS_DRIVER_MYACT_X8_25) {
      (void)myact_fault_clear(&b->axis[axis].u.x8.ctx);
      b->axis[axis].last_rx_ms = mono_ms();
      continue;
    }
    if (b->axis[axis].driver != STC_AXIS_DRIVER_PD4C) continue;

    const int8_t mode = (b->axis[axis].u.pd4c.mode != MTR_MODE_NONE)
      ? (int8_t)b->axis[axis].u.pd4c.mode
      : (int8_t)MTR_MODE_PP;

    const int rc = pd4c_ds402_fault_reset(&b->axis[axis].u.pd4c.ctx, mode, b->cfg.ds402_timeout_ms);
    if (rc == PD4C_OK) {
      b->axis[axis].last_rx_ms = mono_ms();
      b->axis[axis].enabled = 0;
    } else {
      STC_LOG_WARN("axis=%zu fault reset failed rc=%d", axis, rc);
    }
  }

  return 0;
}

int can_motion_backend_set_laser(motion_backend_t* backend, uint8_t laser_id, uint8_t on_off) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b) return -1;
  /* Intentionally side-effect free: test and mock paths rely on deterministic success here. */
  STC_LOG_INFO("laser_stub id=%u on_off=%u (no physical laser hardware integrated)", laser_id, on_off);
  return 0;
}

int can_motion_backend_get_snapshot(motion_backend_t* backend, motion_snapshot_t* snapshot) {
  struct motion_backend* b = (struct motion_backend*)backend;
  if (!b || !snapshot) return -1;
  uint64_t now = mono_ms();
  /* Snapshot is populated from cached telemetry/state; no blocking bus operations here. */
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!b->axis[axis].connected) continue;

    int poll_rc = 0;
    if (b->axis[axis].driver == STC_AXIS_DRIVER_PD4C) do {
      poll_rc = pd4c_poll(&b->axis[axis].u.pd4c.ctx, 0);
      if (poll_rc >= PD4C_OK) {
        const pd4c_telemetry_t* t = pd4c_telem(&b->axis[axis].u.pd4c.ctx);
        b->snap.pos_raw[axis] = t->pos_actual;
        b->snap.vel_raw[axis] = t->vel_actual;
        b->snap.status_word[axis] = t->statusword;
        b->snap.mode_display[axis] = t->mode_display;
        b->axis[axis].last_rx_ms = now;
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
    else if (b->axis[axis].driver == STC_AXIS_DRIVER_MYACT_X8_25) {
      int ev = MYACT_EVENT_NONE;
      int polls = 0;
      do {
        ev = myact_poll(&b->axis[axis].u.x8.ctx, 0);
        if (ev >= 0) {
          b->axis[axis].last_rx_ms = now;
          polls++;
        }
      } while (ev >= 0 && polls < 64);

      const myact_cache_t* cache = &b->axis[axis].u.x8.ctx.cache;
      if (cache->angle_multi_valid) {
        b->snap.pos_raw[axis] = cache->angle_multi_0p01deg;
        b->axis[axis].u.x8.last_raw_angle_deg = (double)cache->angle_multi_0p01deg / 100.0;
      }
      if (cache->status2.valid) {
        b->snap.vel_raw[axis] = cache->status2.speed_dps;
      }
      if (!b->axis[axis].u.x8.active_status2_enabled || !cache->status2.valid) {
        myact_status2_t st2;
        if (myact_read_status2(&b->axis[axis].u.x8.ctx, &st2) == MYACT_OK) {
          b->snap.vel_raw[axis] = st2.speed_dps;
          b->axis[axis].last_rx_ms = now;
        }
      }
      if (!cache->angle_multi_valid) {
        int32_t ang_0p01 = 0;
        if (myact_read_angle_multi(&b->axis[axis].u.x8.ctx, &ang_0p01) == MYACT_OK) {
          b->snap.pos_raw[axis] = ang_0p01;
          b->axis[axis].u.x8.last_raw_angle_deg = (double)ang_0p01 / 100.0;
          b->axis[axis].last_rx_ms = now;
        }
      }
      if (cache->status1.valid) {
        b->axis[axis].u.x8.error_flags = cache->status1.error_flags;
        b->axis[axis].u.x8.bus_voltage_v = cache->status1.voltage_0p1v / 10.0;
        b->axis[axis].u.x8.motor_temp_c = cache->status1.motor_temp_c;
        b->axis[axis].u.x8.brake_released = cache->status1.brake_release_command_state ? 1 : 0;
      }
      b->snap.status_word[axis] = b->axis[axis].u.x8.error_flags;
      b->snap.diag_ext[axis] = ((uint32_t)(uint8_t)b->axis[axis].u.x8.motor_temp_c << 24)
                             | ((uint32_t)(b->axis[axis].u.x8.brake_released ? 1u : 0u) << 16)
                             | (uint32_t)(b->axis[axis].u.x8.bus_voltage_v * 10.0);

      poll_rc = (ev == MYACT_ETIMEDOUT) ? MYACT_OK : ev;
    }

    if (poll_rc == PD4C_ETIMEDOUT && b->cfg.pp_sdo_diag) {
      const uint64_t elapsed_since_check_ms = now - b->axis[axis].last_healthcheck_ms;
      if (elapsed_since_check_ms >= b->cfg.can_timeout_ms && verify_node_responsive(&b->axis[axis].u.pd4c.ctx, b->cfg.can_timeout_ms)) {
        b->axis[axis].last_rx_ms = now;
        b->axis[axis].last_healthcheck_ms = now;
      }
    }
    b->snap.last_rx_ms[axis] = b->axis[axis].last_rx_ms;
    b->snap.axis_connected[axis] = b->axis[axis].connected ? 1u : 0u;
    b->snap.axis_available[axis] = b->snap.axis_connected[axis];
    if (b->axis[axis].driver == STC_AXIS_DRIVER_MYACT_X8_25) {
      b->snap.axis_fault_active[axis] =
          (stc_diag_myact_error_severity((uint16_t)b->snap.status_word[axis]) == STC_DIAG_MYACT_SEV_CRITICAL) ? 1u : 0u;
    } else {
      b->snap.axis_fault_active[axis] = (b->snap.status_word[axis] & 0x0008u) ? 1u : 0u;
    }
    b->snap.axis_stale[axis] = (b->cfg.can_timeout_ms > 0 && b->axis[axis].last_rx_ms > 0 &&
                                (now - b->axis[axis].last_rx_ms) > (uint64_t)b->cfg.can_timeout_ms) ? 1u : 0u;
    b->snap.axis_moving[axis] = (b->snap.vel_raw[axis] > 5 || b->snap.vel_raw[axis] < -5) ? 1u : 0u;
    b->snap.fault_active[axis] = b->snap.axis_fault_active[axis];
    b->snap.command_age_ms[axis] = (b->axis[axis].last_cmd_ms > 0 && now >= b->axis[axis].last_cmd_ms) ? (uint32_t)(now - b->axis[axis].last_cmd_ms) : 0u;
    const int32_t abs_err = b->snap.pos_raw[axis] - b->axis_last_cmd_pos_raw[axis];
    const int pos_still_far = (abs_err > 20 || abs_err < -20);
    const int vel_still_zero = (b->snap.vel_raw[axis] < 5 && b->snap.vel_raw[axis] > -5);
    b->snap.cmd_ack_no_motion[axis] = (b->axis_cmd_ack_pending[axis] && pos_still_far && vel_still_zero && b->snap.command_age_ms[axis] > 200u) ? 1u : 0u;
  }
  refresh_can_status(b);
  *snapshot = b->snap;
  return 0;
}
