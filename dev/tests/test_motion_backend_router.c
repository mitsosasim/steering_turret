/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Backend-router selection and dispatch regression tests for motion backend abstraction.
 *
 * Design notes:
 *   Verifies simulator/CAN backends expose consistent behavior behind shared API contracts.
 */

#include "motion_backend.h"
#include "stc_actions.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { int dummy; } test_backend_t;

static double g_current_pos[STC_AXIS_COUNT];
static double g_target_pos[STC_AXIS_COUNT];
static double g_speed[STC_AXIS_COUNT];

static void assert_true(int cond, const char* msg) {
  if (!cond) {
    fprintf(stderr, "FAIL: %s\n", msg);
    exit(1);
  }
}

int sim_motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out) {
  (void)cfg;
  static test_backend_t backend;
  memset(g_current_pos, 0, sizeof(g_current_pos));
  memset(g_target_pos, 0, sizeof(g_target_pos));
  memset(g_speed, 0, sizeof(g_speed));
  *out = (motion_backend_t*)&backend;
  return 0;
}
void sim_motion_backend_stop(motion_backend_t* b) { (void)b; }
int sim_motion_backend_discover(motion_backend_t* b) { (void)b; return 0; }

int sim_motion_backend_apply_pp_move(motion_backend_t* b, uint32_t mask, const double target[STC_AXIS_COUNT]) {
  (void)b;
  for (size_t i = 0; i < STC_AXIS_COUNT; ++i) {
    if (mask & (1u << i)) {
      g_target_pos[i] = target[i];
      g_current_pos[i] = target[i];
    }
  }
  return 0;
}

int sim_motion_backend_apply_speed(motion_backend_t* b, uint32_t mask, const double speed[STC_AXIS_COUNT]) {
  (void)b;
  for (size_t i = 0; i < STC_AXIS_COUNT; ++i) {
    if (mask & (1u << i)) g_speed[i] = speed[i];
  }
  return 0;
}

int sim_motion_backend_quick_stop(motion_backend_t* b, uint32_t m) { (void)b; (void)m; return 0; }
int sim_motion_backend_clear_faults(motion_backend_t* b, uint32_t m) { (void)b; (void)m; return 0; }
int sim_motion_backend_set_laser(motion_backend_t* b, uint8_t i, uint8_t o) { (void)b; (void)i; (void)o; return 0; }
int sim_motion_backend_get_snapshot(motion_backend_t* b, motion_snapshot_t* s) { (void)b; memset(s, 0, sizeof(*s)); for (size_t i = 0; i < STC_AXIS_COUNT; ++i) s->pos_raw[i] = (int32_t)g_current_pos[i]; return 0; }

/* Unused backends for link completeness. */
int pd4c_motion_backend_start(const motion_backend_cfg_t* c, motion_backend_t** o) { (void)c; (void)o; return -1; }
void pd4c_motion_backend_stop(motion_backend_t* b) { (void)b; }
int pd4c_motion_backend_discover(motion_backend_t* b) { (void)b; return -1; }
int pd4c_motion_backend_apply_pp_move(motion_backend_t* b, uint32_t m, const double t[STC_AXIS_COUNT]) { (void)b; (void)m; (void)t; return -1; }
int pd4c_motion_backend_apply_speed(motion_backend_t* b, uint32_t m, const double s[STC_AXIS_COUNT]) { (void)b; (void)m; (void)s; return -1; }
int pd4c_motion_backend_quick_stop(motion_backend_t* b, uint32_t m) { (void)b; (void)m; return -1; }
int pd4c_motion_backend_clear_faults(motion_backend_t* b, uint32_t m) { (void)b; (void)m; return -1; }
int pd4c_motion_backend_set_laser(motion_backend_t* b, uint8_t i, uint8_t o) { (void)b; (void)i; (void)o; return -1; }
int pd4c_motion_backend_get_snapshot(motion_backend_t* b, motion_snapshot_t* s) { (void)b; (void)s; return -1; }
int myact_motion_backend_start(const motion_backend_cfg_t* c, motion_backend_t** o) { (void)c; (void)o; return -1; }
void myact_motion_backend_stop(motion_backend_t* b) { (void)b; }
int myact_motion_backend_discover(motion_backend_t* b) { (void)b; return -1; }
int myact_motion_backend_apply_pp_move(motion_backend_t* b, uint32_t m, const double t[STC_AXIS_COUNT]) { (void)b; (void)m; (void)t; return -1; }
int myact_motion_backend_apply_motion(motion_backend_t* b, uint32_t m, const stc_axis_motion_kind_t k[STC_AXIS_COUNT], const double v[STC_AXIS_COUNT], const double s[STC_AXIS_COUNT]) { (void)b; (void)m; (void)k; (void)v; (void)s; return -1; }
int myact_motion_backend_apply_speed(motion_backend_t* b, uint32_t m, const double s[STC_AXIS_COUNT]) { (void)b; (void)m; (void)s; return -1; }
int myact_motion_backend_quick_stop(motion_backend_t* b, uint32_t m) { (void)b; (void)m; return -1; }
int myact_motion_backend_clear_faults(motion_backend_t* b, uint32_t m) { (void)b; (void)m; return -1; }
int myact_motion_backend_set_laser(motion_backend_t* b, uint8_t i, uint8_t o) { (void)b; (void)i; (void)o; return -1; }
int myact_motion_backend_get_snapshot(motion_backend_t* b, motion_snapshot_t* s) { (void)b; (void)s; return -1; }
int can_motion_backend_start(const motion_backend_cfg_t* c, motion_backend_t** o) { (void)c; (void)o; return -1; }
void can_motion_backend_stop(motion_backend_t* b) { (void)b; }
int can_motion_backend_discover(motion_backend_t* b) { (void)b; return -1; }
int can_motion_backend_apply_pp_move(motion_backend_t* b, uint32_t m, const double t[STC_AXIS_COUNT]) { (void)b; (void)m; (void)t; return -1; }
int can_motion_backend_apply_speed(motion_backend_t* b, uint32_t m, const double s[STC_AXIS_COUNT]) { (void)b; (void)m; (void)s; return -1; }
int can_motion_backend_quick_stop(motion_backend_t* b, uint32_t m) { (void)b; (void)m; return -1; }
int can_motion_backend_clear_faults(motion_backend_t* b, uint32_t m) { (void)b; (void)m; return -1; }
int can_motion_backend_set_laser(motion_backend_t* b, uint8_t i, uint8_t o) { (void)b; (void)i; (void)o; return -1; }
int can_motion_backend_get_snapshot(motion_backend_t* b, motion_snapshot_t* s) { (void)b; (void)s; return -1; }

static void test_abs_then_rel(void) {
  motion_backend_cfg_t cfg = {0};
  snprintf(cfg.backend, sizeof(cfg.backend), "simulator");
  motion_backend_t* backend = NULL;
  assert_true(motion_backend_start(&cfg, &backend) == 0, "start sim");

  stc_axis_motion_kind_t kind[STC_AXIS_COUNT] = {STC_AXIS_MOTION_ABS};
  double values[STC_AXIS_COUNT] = {10.0};
  double speeds[STC_AXIS_COUNT] = {0};
  assert_true(motion_backend_apply_motion(backend, 1u << 0, kind, values, speeds) == 0, "abs apply");
  assert_true(g_target_pos[0] == 10.0, "abs target");

  kind[0] = STC_AXIS_MOTION_REL;
  values[0] = 2.0;
  assert_true(motion_backend_apply_motion(backend, 1u << 0, kind, values, speeds) == 0, "rel apply");
  assert_true(g_target_pos[0] == 12.0, "rel composed onto abs endpoint");
}

static void test_multiple_rel_accumulates(void) {
  stc_axis_motion_kind_t kind[STC_AXIS_COUNT] = {STC_AXIS_MOTION_REL};
  double values[STC_AXIS_COUNT] = {1.0};
  double speeds[STC_AXIS_COUNT] = {0};
  assert_true(motion_backend_apply_motion(NULL, 1u << 0, kind, values, speeds) == 0, "rel #1");
  assert_true(g_target_pos[0] == 13.0, "rel #1 result");

  values[0] = -1.0;
  assert_true(motion_backend_apply_motion(NULL, 1u << 0, kind, values, speeds) == 0, "rel #2");
  assert_true(g_target_pos[0] == 12.0, "rel #2 cumulative result");
}

static void test_speed_interleaved_no_target_reset(void) {
  stc_axis_motion_kind_t kind[STC_AXIS_COUNT] = {STC_AXIS_MOTION_SPEED};
  double values[STC_AXIS_COUNT] = {3.0};
  double speeds[STC_AXIS_COUNT] = {0};

  assert_true(motion_backend_apply_motion(NULL, 1u << 0, kind, values, speeds) == 0, "speed apply");
  assert_true(g_speed[0] == 3.0, "speed set");
  assert_true(g_target_pos[0] == 12.0, "speed does not reset position target");

  kind[0] = STC_AXIS_MOTION_ABS;
  values[0] = 20.0;
  assert_true(motion_backend_apply_motion(NULL, 1u << 0, kind, values, speeds) == 0, "abs after speed");
  assert_true(g_target_pos[0] == 20.0, "abs target preserved after speed mode");
}


static void test_count_configured_axes_counts_enabled_only(void) {
  motion_backend_cfg_t cfg = {0};
  int pd4c_count = -1;
  int myact_count = -1;

  cfg.axis_enabled[0] = 1;
  cfg.axis_cfg[0].driver = STC_AXIS_DRIVER_MYACT_X8_25;
  cfg.axis_enabled[1] = 0;
  cfg.axis_cfg[1].driver = STC_AXIS_DRIVER_PD4C;

  hardware_backend_count_configured_axes(&cfg, &pd4c_count, &myact_count);
  assert_true(pd4c_count == 0, "pd4c init count stays zero for one-X8 config");
  assert_true(myact_count == 1, "myact init count is one for one-X8 config");
}

int main(void) {
  test_abs_then_rel();
  test_multiple_rel_accumulates();
  test_speed_interleaved_no_target_reset();
  test_count_configured_axes_counts_enabled_only();
  puts("OK");
  return 0;
}
