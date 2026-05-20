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
static int myact_axis_stop(void* driver_ctx) { return myact_stop_send((myact_ctx_t*)driver_ctx); }
static int myact_axis_shutdown(void* driver_ctx) { return myact_shutdown_send((myact_ctx_t*)driver_ctx); }
static int myact_axis_move_abs(void* driver_ctx, double target_deg, double speed_deg_s) {
  return myact_position_abs_deg_send_raw((myact_ctx_t*)driver_ctx, target_deg, (uint16_t)llround(speed_deg_s));
}
static int myact_axis_move_rel(void* driver_ctx, double delta_deg, double speed_deg_s) {
  return myact_position_inc_send_raw((myact_ctx_t*)driver_ctx, (uint16_t)llround(speed_deg_s), myact_deg_to_raw_0p01(delta_deg));
}
static int myact_axis_speed(void* driver_ctx, double speed_deg_s) { return myact_speed_dps_send_raw((myact_ctx_t*)driver_ctx, speed_deg_s); }

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
static int compute_can_ok(const struct motion_backend* b) {
  if (!b) return 0;

  int expected_axes = 0;
  int connected_expected_axes = 0;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!b->cfg.axis_enabled[axis] || b->cfg.axis_cfg[axis].driver == STC_AXIS_DRIVER_DISABLED) continue;
    expected_axes++;
    if (b->axis[axis].connected) connected_expected_axes++;
  }

  if (expected_axes == 0) return 0;
  return connected_expected_axes == expected_axes;
}

static void refresh_can_status(struct motion_backend* b) {
  if (!b) return;
  /* Aggregate per-axis connectivity into global CAN health for telemetry/state gating. */
  b->snap.can_ok = compute_can_ok(b);
  b->snap.capabilities.supports_native_a2_speed = 0;
  b->snap.capabilities.supports_native_absolute_position = 0;
  b->snap.backend_mode = 2;
  b->snap.transport_status = 1;
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
        int rc = myact_socketcan_open(&b->axis[axis].u.x8.io, cfg->can_if, node_id);
        if (rc != MYACT_OK) break;
        if (myact_init(&b->axis[axis].u.x8.ctx, node_id, &b->axis[axis].u.x8.io.io) != MYACT_OK) {
          myact_socketcan_close(&b->axis[axis].u.x8.io);
          break;
        }
        char model[8] = {0};
        uint32_t ver = 0, runtime = 0;
        myact_status1_t status1;
        myact_status3_t status3;
        int32_t angle_raw_0p01 = 0;
        const int model_rc = myact_read_model(&b->axis[axis].u.x8.ctx, model);
        const int ver_rc = myact_read_version_date(&b->axis[axis].u.x8.ctx, &ver);
        const int run_rc = myact_read_runtime(&b->axis[axis].u.x8.ctx, &runtime);
        const int st_rc = myact_read_status1(&b->axis[axis].u.x8.ctx, &status1);
        const int ang_rc = myact_read_angle_multi(&b->axis[axis].u.x8.ctx, &angle_raw_0p01);
        const int status3_rc = myact_read_status3(&b->axis[axis].u.x8.ctx, &status3);
        if (model_rc == MYACT_OK &&
            ver_rc == MYACT_OK &&
            run_rc == MYACT_OK &&
            st_rc == MYACT_OK &&
            ang_rc == MYACT_OK &&
            status3_rc == MYACT_OK) {
          b->axis[axis].connected = 1;
          b->axis[axis].last_rx_ms = mono_ms();
          b->axis[axis].last_healthcheck_ms = b->axis[axis].last_rx_ms;
          b->snap.last_rx_ms[axis] = b->axis[axis].last_rx_ms;
          b->snap.axis_available[axis] = 1;
          b->snap.can_connected_devices++;
          b->axis[axis].u.x8.last_raw_angle_deg = (double)angle_raw_0p01 / 100.0;
          if (cfg->axis_cfg[axis].zero_on_start) {
            b->axis[axis].u.x8.raw_zero_deg = b->axis[axis].u.x8.last_raw_angle_deg;
            STC_LOG_INFO("axis%zu zero_on_start raw_zero_deg=%.2f", axis, b->axis[axis].u.x8.raw_zero_deg);
          }
          if (cfg->axis_cfg[axis].active_status2_enable) {
            const uint16_t interval_10ms = (uint16_t)((cfg->axis_cfg[axis].active_status2_interval_ms > 0 ? cfg->axis_cfg[axis].active_status2_interval_ms : 50) / 10);
            (void)myact_active_reply_config(&b->axis[axis].u.x8.ctx, MYACT_CMD_STATUS2, true, interval_10ms);
            STC_LOG_INFO("axis%zu active_reply 9C enabled interval_ms=%u", axis, (unsigned)(interval_10ms * 10u));
          }
          STC_LOG_INFO("axis%zu driver=myact_x8_25 node=%u connected model=%s ver=%u runtime_ms=%u", axis, node_id, model, ver, runtime);
          STC_LOG_INFO("axis%zu myact init OK", axis);
        } else {
          myact_socketcan_close(&b->axis[axis].u.x8.io);
        }
        (void)status1;
        (void)status3;
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
    if (!axis_ready_for_motion(b, axis) || b->axis[axis].driver != STC_AXIS_DRIVER_PD4C) return -1;
  }

  uint32_t touched_mask = 0u;
  for (size_t axis = 0; axis < STC_AXIS_COUNT; axis++) {
    if (!axis_selected(axes_mask, axis)) continue;
    /* DS402 enable/operation mode must be active before profile-position target is sent. */
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
    touched_mask |= (1u << axis);
    const uint64_t tms = mono_ms();
    b->axis[axis].last_rx_ms = tms;
    b->axis[axis].last_cmd_ms = tms;
    b->axis_last_cmd_pos_raw[axis] = target_counts;
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
    const double speed_cmd = (b->axis[axis].driver == STC_AXIS_DRIVER_PD4C) ? (double)tv_raw : speeds_deg_s[axis];
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
    if ((axes_mask & (1u << axis)) == 0 || !b->axis[axis].connected || b->axis[axis].driver != STC_AXIS_DRIVER_PD4C) continue;

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
    if (b->axis[axis].driver == STC_AXIS_DRIVER_MYACT_X8_25) {
      b->snap.fault_active[axis] = (stc_diag_myact_error_severity((uint16_t)b->snap.status_word[axis]) == STC_DIAG_MYACT_SEV_CRITICAL) ? 1u : 0u;
    } else {
      b->snap.fault_active[axis] = (b->snap.status_word[axis] & 0x0008u) ? 1u : 0u;
    }
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
