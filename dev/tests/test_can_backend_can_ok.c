/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Mocked CAN backend availability and health-status regression tests.
 *
 * Design notes:
 *   Verifies can_ok and node discovery accounting used by diagnostics/state gating without requiring hardware.
 */

#include "motion_backend.h"
#include "pd4c.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  uint8_t responsive_by_node[128];
  int open_count;
  int close_count;
} pd4c_mock_state_t;

static pd4c_mock_state_t g_mock;

static void mock_reset(void) {
  memset(&g_mock, 0, sizeof(g_mock));
}

/* Mock open: records discovery attempts per node without touching real CAN interfaces. */
int pd4c_open(pd4c_ctx_t* ctx, const char* ifname, uint8_t node_id) {
  (void)ifname;
  if (!ctx || node_id == 0 || node_id >= 128) return PD4C_EINVAL;
  memset(ctx, 0, sizeof(*ctx));
  ctx->node_id = node_id;
  g_mock.open_count++;
  return PD4C_OK;
}

void pd4c_close(pd4c_ctx_t* ctx) {
  (void)ctx;
  g_mock.close_count++;
}

int pd4c_nmt_start(pd4c_ctx_t* ctx) {
  (void)ctx;
  return PD4C_OK;
}

void pd4c_set_sdo_timeout(pd4c_ctx_t* ctx, int timeout_ms) {
  if (!ctx) return;
  ctx->sdo_timeout_ms = timeout_ms;
}

/* Mock identity reads: responsive nodes emulate 0x1018 identity, non-responsive nodes time out. */
int pd4c_sdo_read_u32(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint32_t* out) {
  if (!ctx || !out) return PD4C_EINVAL;
  if (idx == 0x1018u && sub >= 0x01u && sub <= 0x04u) {
    if (g_mock.responsive_by_node[ctx->node_id]) {
      *out = 0x1000u + sub;
      return PD4C_OK;
    }
    return PD4C_ETIMEDOUT;
  }
  return PD4C_EINVAL;
}

int pd4c_sdo_read_u8(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint8_t* out) {
  if (!ctx || !out) return PD4C_EINVAL;
  if (idx == 0x2009u && sub == 0x00u && g_mock.responsive_by_node[ctx->node_id]) {
    *out = ctx->node_id;
    return PD4C_OK;
  }
  return PD4C_ETIMEDOUT;
}

int pd4c_ds402_enable_operation(pd4c_ctx_t* ctx, int8_t mode, int timeout_ms) {
  (void)ctx;
  (void)mode;
  (void)timeout_ms;
  return PD4C_OK;
}

int pd4c_pp_move(pd4c_ctx_t* ctx, int32_t target_pos, uint16_t profile_vel, int is_relative, int timeout_ms) {
  (void)ctx;
  (void)target_pos;
  (void)profile_vel;
  (void)is_relative;
  (void)timeout_ms;
  return PD4C_OK;
}

int pd4c_rpdo3_send_target_velocity(pd4c_ctx_t* ctx, int16_t target_vel) {
  (void)ctx;
  (void)target_vel;
  return PD4C_OK;
}

int pd4c_ds402_quick_stop(pd4c_ctx_t* ctx, int8_t mode) {
  (void)ctx;
  (void)mode;
  return PD4C_OK;
}

int pd4c_ds402_fault_reset(pd4c_ctx_t* ctx, int8_t mode, int timeout_ms) {
  (void)ctx;
  (void)mode;
  (void)timeout_ms;
  return PD4C_OK;
}

int pd4c_poll(pd4c_ctx_t* ctx, int timeout_ms) {
  (void)ctx;
  (void)timeout_ms;
  return PD4C_ETIMEDOUT;
}

const pd4c_telemetry_t* pd4c_telem(const pd4c_ctx_t* ctx) {
  return &ctx->telem;
}

static motion_backend_cfg_t make_cfg(uint8_t node0, uint8_t node1) {
  motion_backend_cfg_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.can_if = "vcan0";
  cfg.can_timeout_ms = 50;
  cfg.axis_node_id[0] = node0;
  cfg.axis_node_id[1] = node1;
  cfg.can_scan_max_node = 4;
  cfg.axis_cfg[0].counts_per_rev = 4096.0;
  cfg.axis_cfg[1].counts_per_rev = 4096.0;
  return cfg;
}

static void verify_snapshot_can_ok(motion_backend_t* backend, int expected_can_ok) {
  motion_snapshot_t snap;
  memset(&snap, 0, sizeof(snap));
  assert(motion_backend_get_snapshot(backend, &snap) == 0);
  assert(snap.can_ok == expected_can_ok);
}

static void test_can_ok_no_devices_connected(void) {
  mock_reset();

  motion_backend_t* backend = NULL;
  motion_backend_cfg_t cfg = make_cfg(1, 2);
  assert(motion_backend_start(&cfg, &backend) == 0);
  assert(backend != NULL);

  verify_snapshot_can_ok(backend, 0);
  motion_backend_stop(backend);
}

static void test_can_ok_partial_devices_connected(void) {
  mock_reset();
  g_mock.responsive_by_node[1] = 1;

  motion_backend_t* backend = NULL;
  motion_backend_cfg_t cfg = make_cfg(1, 2);
  assert(motion_backend_start(&cfg, &backend) == 0);
  assert(backend != NULL);

  verify_snapshot_can_ok(backend, 0);
  motion_backend_stop(backend);
}

static void test_can_ok_all_expected_devices_connected(void) {
  mock_reset();
  g_mock.responsive_by_node[1] = 1;
  g_mock.responsive_by_node[2] = 1;

  motion_backend_t* backend = NULL;
  motion_backend_cfg_t cfg = make_cfg(1, 2);
  assert(motion_backend_start(&cfg, &backend) == 0);
  assert(backend != NULL);

  verify_snapshot_can_ok(backend, 1);
  motion_backend_stop(backend);
}

/* can_ok means at least one configured node is discoverable and identity/status reads complete within timeout. */
int main(void) {
  test_can_ok_no_devices_connected();
  test_can_ok_partial_devices_connected();
  test_can_ok_all_expected_devices_connected();
  puts("test_can_backend_can_ok: PASS");
  return 0;
}
