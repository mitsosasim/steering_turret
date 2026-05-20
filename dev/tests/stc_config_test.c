/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Config parser strictness and backward-compatibility regression tests for STC INI files.
 *
 * Design notes:
 *   Protects startup determinism by rejecting malformed configs while preserving accepted legacy spellings and defaults.
 */

#include "stc_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#ifdef _WIN32
#include <windows.h>
#endif

static void die(const char* msg) {
  fprintf(stderr, "FAIL: %s\n", msg);
  exit(1);
}

static void expect_true(int cond, const char* msg) {
  if (!cond) die(msg);
}

static void write_tmp_config(const char* contents, char out_path[], size_t out_path_size) {
  static unsigned counter = 0;
#ifdef _WIN32
  char tmp_dir[MAX_PATH] = {0};
  DWORD n = GetTempPathA((DWORD)sizeof(tmp_dir), tmp_dir);
  if (n == 0 || n >= sizeof(tmp_dir)) die("GetTempPathA failed");
  snprintf(out_path, out_path_size, "%sstc_cfg_test_%lu_%u.ini", tmp_dir, (unsigned long)GetCurrentProcessId(), counter++);
#else
  snprintf(out_path, out_path_size, "/tmp/stc_cfg_test_%ld_%u.ini", (long)getpid(), counter++);
#endif

  FILE* f = fopen(out_path, "w");
  if (!f) die("fopen failed");

  if (fputs(contents, f) == EOF) {
    fclose(f);
    die("fputs failed");
  }

  if (fclose(f) != 0) die("fclose failed");
}

/* Verifies strict key validation so typoed network keys fail fast instead of silently altering runtime networking. */
static void test_reject_typoed_network_key(void) {
  const char* cfg_text =
      "[Network]\n"
      "tcp_prt = 7000\n"
      "tlm_dest_ip = 127.0.0.1\n"
      "tlmh_port = 6010\n"
      "tlml_port = 6020\n"
      "tlmh_interval_ms = 10\n"
      "tlml_interval_ms = 100\n"
      "\n"
      "[Axis0]\n"
      "node_id = 1\n"
      "counts_per_rev = 4096\n"
      "gear_ratio = 1\n"
      "min_deg = -180\n"
      "max_deg = 180\n"
      "max_speed_deg_s = 90\n"
      "storage_deg = 0\n"
      "pp_profile_vel = 500\n"
      "diag_cat = 0x06\n"
      "diag_subcat = 0x3F\n"
      "\n"
      "[Turret]\n"
      "az_axis = 0\n"
      "el_axis = 0\n"
      "\n"
      "[diagnostics]\n"
      "axis[0].category = 0x06\n"
      "axis[0].subcategory = 0x3F\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(!load_config(path, &cfg), "expected typoed [Network] key to fail");
  unlink(path);
}

/* Verifies strict section-name parsing to catch operator configuration mistakes before turret runtime starts. */
static void test_reject_typoed_section_name(void) {
  const char* cfg_text =
      "[Netwrok]\n"
      "tcp_port = 7000\n"
      "\n"
      "[Axis0]\n"
      "node_id = 1\n"
      "counts_per_rev = 4096\n"
      "gear_ratio = 1\n"
      "min_deg = -180\n"
      "max_deg = 180\n"
      "max_speed_deg_s = 90\n"
      "storage_deg = 0\n"
      "pp_profile_vel = 500\n"
      "diag_cat = 0x06\n"
      "diag_subcat = 0x3F\n"
      "\n"
      "[Turret]\n"
      "az_axis = 0\n"
      "el_axis = 0\n"
      "\n"
      "[diagnostics]\n"
      "axis[0].category = 0x06\n"
      "axis[0].subcategory = 0x3F\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(!load_config(path, &cfg), "expected typoed section name to fail");
  unlink(path);
}


/* Verifies backward compatibility aliases to prevent regressions when loading legacy deployment INI files. */
static void test_accept_deprecated_multicast_network_keys(void) {
  const char* cfg_text =
      "[Network]\n"
      "tcp_port = 7000\n"
      "mcast_ip = 127.0.0.1\n"
      "mcast_tlmh_port = 6010\n"
      "mcast_tlml_port = 6020\n"
      "tlmh_interval_ms = 10\n"
      "tlml_interval_ms = 100\n"
      "\n"
      "[Axis0]\n"
      "node_id = 1\n"
      "counts_per_rev = 4096\n"
      "gear_ratio = 1\n"
      "min_deg = -180\n"
      "max_deg = 180\n"
      "max_speed_deg_s = 90\n"
      "storage_deg = 0\n"
      "pp_profile_vel = 500\n"
      "diag_cat = 0x06\n"
      "diag_subcat = 0x3F\n"
      "\n"
      "[Turret]\n"
      "az_axis = 0\n"
      "el_axis = 0\n"
      "\n"
      "[diagnostics]\n"
      "axis[0].category = 0x06\n"
      "axis[0].subcategory = 0x3F\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(load_config(path, &cfg), "expected deprecated multicast keys to parse");
  expect_true(strcmp(cfg.tlm_dest_ip, "127.0.0.1") == 0, "expected deprecated mcast_ip to map to tlm_dest_ip");
  expect_true(cfg.tlmh_port == 6010, "expected deprecated mcast_tlmh_port to map to tlmh_port");
  expect_true(cfg.tlml_port == 6020, "expected deprecated mcast_tlml_port to map to tlml_port");
  unlink(path);
}

/* Verifies a canonical valid config still loads, protecting startup compatibility and expected defaults. */
static void test_accept_valid_config(void) {
  const char* cfg_text =
      "[Network]\n"
      "tcp_port = 7000\n"
      "tlm_dest_ip = 127.0.0.1\n"
      "tlmh_port = 6010\n"
      "tlml_port = 6020\n"
      "tlmh_interval_ms = 10\n"
      "tlml_interval_ms = 100\n"
      "strict_stale_policy = 1\n"
      "\n"
      "[Axis0]\n"
      "node_id = 1\n"
      "counts_per_rev = 4096\n"
      "gear_ratio = 1\n"
      "min_deg = -180\n"
      "max_deg = 180\n"
      "max_speed_deg_s = 90\n"
      "storage_deg = 0\n"
      "pp_profile_vel = 500\n"
      "diag_cat = 0x06\n"
      "diag_subcat = 0x3F\n"
      "\n"
      "[Turret]\n"
      "az_axis = 0\n"
      "el_axis = 0\n"
      "\n"
      "[diagnostics]\n"
      "axis[0].category = 0x06\n"
      "axis[0].subcategory = 0x3F\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(load_config(path, &cfg), "expected valid config to pass");
  expect_true(cfg.strict_stale_policy == 1, "expected strict_stale_policy to parse");
  unlink(path);
}


/* Verifies sparse optional sections parse correctly, preventing false startup failures on partial installations. */
static void test_accept_sparse_sut_and_mirror_sections(void) {
  const char* cfg_text =
      "[Network]\n"
      "tcp_port = 7000\n"
      "tlm_dest_ip = 127.0.0.1\n"
      "tlmh_port = 6010\n"
      "tlml_port = 6020\n"
      "tlmh_interval_ms = 10\n"
      "tlml_interval_ms = 100\n"
      "\n"
      "[Axis0]\n"
      "node_id = 1\n"
      "counts_per_rev = 4096\n"
      "gear_ratio = 1\n"
      "min_deg = -180\n"
      "max_deg = 180\n"
      "max_speed_deg_s = 90\n"
      "storage_deg = 0\n"
      "pp_profile_vel = 500\n"
      "diag_cat = 0x06\n"
      "diag_subcat = 0x3F\n"
      "\n"
      "[Turret]\n"
      "az_axis = 0\n"
      "el_axis = 0\n"
      "\n"
      "[SUT1]\n"
      "az_axis = 0\n"
      "el_axis = 0\n"
      "\n"
      "[Mirror1]\n"
      "axis = 0\n"
      "\n"
      "[diagnostics]\n"
      "axis[0].category = 0x06\n"
      "axis[0].subcategory = 0x3F\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(load_config(path, &cfg), "expected sparse [SUT1]/[Mirror1] config to pass");
  expect_true(cfg.sut_count == 1, "expected sut_count to follow defined sections");
  expect_true(cfg.mirror_count == 1, "expected mirror_count to follow defined sections");
  expect_true(cfg.sut_az_axis[1] == -1 && cfg.sut_el_axis[1] == -1, "expected missing [SUT2] to remain unmapped");
  expect_true(cfg.mirror_axis[1] == -1, "expected missing [Mirror2] to remain unmapped");
  unlink(path);
}

/* Verifies mixed CAN driver families parse as a valid startup configuration. */

/* Verifies [Runtime].backend aliases map to expected runtime-backend enum values and reject axis-driver strings. */
static void test_runtime_backend_values(void) {
  struct runtime_case {
    const char* backend_value;
    int expect_ok;
    stc_runtime_backend_t expected_backend;
  } cases[] = {
      {"can", 1, STC_RUNTIME_BACKEND_HARDWARE_CAN},
      {"hardware", 1, STC_RUNTIME_BACKEND_HARDWARE_CAN},
      {"simulator", 1, STC_RUNTIME_BACKEND_SIMULATOR},
      {"myactuator", 0, STC_RUNTIME_BACKEND_SIMULATOR},
      {"pd4c", 0, STC_RUNTIME_BACKEND_SIMULATOR},
  };

  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
    char cfg_text[1024];
    snprintf(cfg_text, sizeof(cfg_text),
             "[Runtime]\n"
             "backend = %s\n"
             "\n"
             "[Network]\n"
             "tcp_port = 7000\n"
             "tlm_dest_ip = 127.0.0.1\n"
             "tlmh_port = 6010\n"
             "tlml_port = 6020\n"
             "tlmh_interval_ms = 10\n"
             "tlml_interval_ms = 100\n"
             "\n"
             "[Axis0]\n"
             "node_id = 1\n"
             "counts_per_rev = 4096\n"
             "gear_ratio = 1\n"
             "min_deg = -180\n"
             "max_deg = 180\n"
             "max_speed_deg_s = 90\n"
             "storage_deg = 0\n"
             "pp_profile_vel = 500\n"
             "diag_cat = 0x06\n"
             "diag_subcat = 0x3F\n"
             "\n"
             "[Turret]\n"
             "az_axis = 0\n"
             "el_axis = 0\n"
             "\n"
             "[diagnostics]\n"
             "axis[0].category = 0x06\n"
             "axis[0].subcategory = 0x3F\n",
             cases[i].backend_value);

    char path[64];
    stc_config_t cfg;
    char msg[128];
    write_tmp_config(cfg_text, path, sizeof(path));
    if (cases[i].expect_ok) {
      snprintf(msg, sizeof(msg), "expected [Runtime] backend=%s to parse", cases[i].backend_value);
      expect_true(load_config(path, &cfg), msg);
      snprintf(msg, sizeof(msg), "expected [Runtime] backend=%s to map to expected runtime backend", cases[i].backend_value);
      expect_true(cfg.runtime_backend == cases[i].expected_backend, msg);
    } else {
      snprintf(msg, sizeof(msg), "expected [Runtime] backend=%s to fail parse", cases[i].backend_value);
      expect_true(!load_config(path, &cfg), msg);
    }
    unlink(path);
  }
}

/* Verifies non-simulator builds default runtime backend to hardware_can when backend is not explicitly set. */
static void test_hardware_build_default_backend(void) {
  stc_config_t cfg;
  stc_config_set_defaults(&cfg);
#ifndef USE_SIMULATOR
  expect_true(cfg.runtime_backend == STC_RUNTIME_BACKEND_HARDWARE_CAN,
              "expected hardware build default runtime backend to hardware_can");
#else
  expect_true(cfg.runtime_backend == STC_RUNTIME_BACKEND_SIMULATOR,
              "expected simulator build default runtime backend to simulator");
#endif
}

static void test_accept_mixed_can_axis_drivers(void) {
  const char* cfg_text =
      "[Network]\n"
      "tcp_port = 7000\n"
      "tlm_dest_ip = 127.0.0.1\n"
      "tlmh_port = 6010\n"
      "tlml_port = 6020\n"
      "tlmh_interval_ms = 10\n"
      "tlml_interval_ms = 100\n"
      "\n"
      "[Axis0]\n"
      "enabled = 1\n"
      "driver = pd4c\n"
      "node_id = 1\n"
      "counts_per_rev = 4096\n"
      "gear_ratio = 1\n"
      "min_deg = -180\n"
      "max_deg = 180\n"
      "max_speed_deg_s = 90\n"
      "storage_deg = 0\n"
      "pp_profile_vel = 500\n"
      "diag_cat = 0x06\n"
      "diag_subcat = 0x3F\n"
      "\n"
      "[Axis1]\n"
      "enabled = 1\n"
      "driver = myact_x8_25\n"
      "node_id = 2\n"
      "counts_per_rev = 4096\n"
      "gear_ratio = 1\n"
      "min_deg = -180\n"
      "max_deg = 180\n"
      "max_speed_deg_s = 90\n"
      "storage_deg = 0\n"
      "pp_profile_vel = 500\n"
      "diag_cat = 0x06\n"
      "diag_subcat = 0x3F\n"
      "\n"
      "[Axis2]\n"
      "enabled = 0\n"
      "driver = disabled\n"
      "node_id = 0\n"
      "counts_per_rev = 4096\n"
      "gear_ratio = 1\n"
      "min_deg = -180\n"
      "max_deg = 180\n"
      "max_speed_deg_s = 90\n"
      "storage_deg = 0\n"
      "pp_profile_vel = 500\n"
      "diag_cat = 0x06\n"
      "diag_subcat = 0x3F\n"
      "\n"
      "[Turret]\n"
      "az_axis = 0\n"
      "el_axis = 1\n"
      "\n"
      "[diagnostics]\n"
      "axis[0].category = 0x06\n"
      "axis[0].subcategory = 0x3F\n"
      "axis[1].category = 0x06\n"
      "axis[1].subcategory = 0x3F\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(load_config(path, &cfg), "expected mixed PD4C/MyActuator axis drivers to parse");
  expect_true(cfg.axis_driver[0] == STC_AXIS_DRIVER_PD4C, "expected Axis0 driver pd4c");
  expect_true(cfg.axis_driver[1] == STC_AXIS_DRIVER_MYACT_X8_25, "expected Axis1 driver myact_x8_25");
  expect_true(cfg.axis_driver[2] == STC_AXIS_DRIVER_DISABLED, "expected Axis2 driver disabled");
  unlink(path);
}


static void test_missing_axis_sections_default_to_disabled(void) {
  const char* cfg_text =
      "[Runtime]\n"
      "backend=can\n"
      "\n"
      "[CAN]\n"
      "ifname=can0\n"
      "\n"
      "[Axis0]\n"
      "enabled=true\n"
      "driver=myact_x8_25\n"
      "node_id=1\n"
      "diag_cat = 0x06\n"
      "diag_subcat = 0x3F\n"
      "\n"
      "[Mirror1]\n"
      "axis=0\n"
      "\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(load_config(path, &cfg), "expected sparse axis sections config to parse");
  expect_true(cfg.axis_enabled[0] == 1, "expected Axis0 enabled=true");
  expect_true(cfg.axis_driver[0] == STC_AXIS_DRIVER_MYACT_X8_25, "expected Axis0 driver myact_x8_25");
  for (size_t i = 1; i < STC_AXIS_COUNT; ++i) {
    expect_true(cfg.axis_enabled[i] == 0, "expected missing AxisN enabled=false by default");
    expect_true(cfg.axis_driver[i] == STC_AXIS_DRIVER_DISABLED, "expected missing AxisN driver=disabled by default");
  }
  unlink(path);
}

static void test_enabled_axis_requires_explicit_driver(void) {
  const char* cfg_text =
      "[Runtime]\n"
      "backend=can\n"
      "\n"
      "[CAN]\n"
      "ifname=can0\n"
      "\n"
      "[Axis0]\n"
      "enabled=true\n"
      "node_id=1\n"
      "diag_cat = 0x06\n"
      "diag_subcat = 0x3F\n"
      "\n"
      "[Mirror1]\n"
      "axis=0\n"
      "\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(!load_config(path, &cfg), "expected enabled axis without driver to fail parsing");
  unlink(path);
}

int main(void) {
  test_reject_typoed_network_key();
  test_reject_typoed_section_name();
  test_accept_valid_config();
  test_accept_deprecated_multicast_network_keys();
  test_accept_sparse_sut_and_mirror_sections();
  test_accept_mixed_can_axis_drivers();
  test_missing_axis_sections_default_to_disabled();
  test_enabled_axis_requires_explicit_driver();
  test_runtime_backend_values();
  test_hardware_build_default_backend();
  fprintf(stderr, "PASS: stc_config_test\n");
  return 0;
}
