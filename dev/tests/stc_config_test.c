#include "stc_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void die(const char* msg) {
  fprintf(stderr, "FAIL: %s\n", msg);
  exit(1);
}

static void expect_true(int cond, const char* msg) {
  if (!cond) die(msg);
}

static void write_tmp_config(const char* contents, char out_path[], size_t out_path_size) {
  static unsigned counter = 0;
  snprintf(out_path, out_path_size, "/tmp/stc_cfg_test_%ld_%u.ini", (long)getpid(), counter++);

  FILE* f = fopen(out_path, "w");
  if (!f) die("fopen failed");

  if (fputs(contents, f) == EOF) {
    fclose(f);
    die("fputs failed");
  }

  if (fclose(f) != 0) die("fclose failed");
}

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
      "axis[0].subcategory = 0x3F\n"
      "motor_status_map = 0,-1,-1,-1,-1,-1,-1,-1\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(!load_config(path, &cfg), "expected typoed [Network] key to fail");
  unlink(path);
}

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
      "axis[0].subcategory = 0x3F\n"
      "motor_status_map = 0,-1,-1,-1,-1,-1,-1,-1\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(!load_config(path, &cfg), "expected typoed section name to fail");
  unlink(path);
}


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
      "axis[0].subcategory = 0x3F\n"
      "motor_status_map = 0,-1,-1,-1,-1,-1,-1,-1\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(load_config(path, &cfg), "expected deprecated multicast keys to parse");
  expect_true(strcmp(cfg.tlm_dest_ip, "127.0.0.1") == 0, "expected deprecated mcast_ip to map to tlm_dest_ip");
  expect_true(cfg.tlmh_port == 6010, "expected deprecated mcast_tlmh_port to map to tlmh_port");
  expect_true(cfg.tlml_port == 6020, "expected deprecated mcast_tlml_port to map to tlml_port");
  unlink(path);
}

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
      "axis[0].subcategory = 0x3F\n"
      "motor_status_map = 0,-1,-1,-1,-1,-1,-1,-1\n";

  char path[64];
  stc_config_t cfg;
  write_tmp_config(cfg_text, path, sizeof(path));
  expect_true(load_config(path, &cfg), "expected valid config to pass");
  expect_true(cfg.strict_stale_policy == 1, "expected strict_stale_policy to parse");
  unlink(path);
}


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
      "axis[0].subcategory = 0x3F\n"
      "motor_status_map = 0,-1,-1,-1,-1,-1,-1,-1\n";

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
int main(void) {
  test_reject_typoed_network_key();
  test_reject_typoed_section_name();
  test_accept_valid_config();
  test_accept_deprecated_multicast_network_keys();
  test_accept_sparse_sut_and_mirror_sections();
  fprintf(stderr, "PASS: stc_config_test\n");
  return 0;
}
