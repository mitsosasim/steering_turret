#include "stc_icd.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static void die(const char* msg) {
  fprintf(stderr, "FAIL: %s\n", msg);
  exit(1);
}

static void assert_eq_u16(uint16_t a, uint16_t b, const char* msg) {
  if (a != b) { fprintf(stderr, "FAIL: %s (got=%u exp=%u)\n", msg, a, b); exit(1); }
}
static void assert_eq_u32(uint32_t a, uint32_t b, const char* msg) {
  if (a != b) { fprintf(stderr, "FAIL: %s (got=%u exp=%u)\n", msg, a, b); exit(1); }
}
static void assert_eq_u64(uint64_t a, uint64_t b, const char* msg) {
  if (a != b) { fprintf(stderr, "FAIL: %s (got=%llu exp=%llu)\n", msg,
                        (unsigned long long)a, (unsigned long long)b); exit(1); }
}
static void assert_true(int cond, const char* msg) {
  if (!cond) die(msg);
}
static void assert_mem(const uint8_t* a, const uint8_t* b, size_t n, const char* msg) {
  if (memcmp(a,b,n)!=0) die(msg);
}

static void test_cmd_turret_pack_parse(void) {
  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();
  cfg.strict_version = 1;

  stc_icd_cmd_t cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.version = 0x01;
  cmd.controller_id = 11;
  cmd.cmd_id = STC_CMD_TURRET;
  cmd.seq = 42;
  cmd.timestamp_ms = 123456789ull;
  cmd.payload_decoded = 1;
  cmd.u.turret.motion_type = STC_MOTION_SPEED;
  cmd.u.turret.az = 1.0f;
  cmd.u.turret.el = -2.5f;

  uint8_t buf[256];
  size_t out_len=0;
  stc_icd_rc_t rc = stc_icd_cmd_pack(&cfg, &cmd, buf, sizeof(buf), &out_len);
  assert_true(rc==STC_ICD_OK, "cmd_pack turret");

  /* Expect header + payload size */
  assert_eq_u16((uint16_t)out_len, (uint16_t)(20+9), "turret cmd length");

  /* Fixed expected bytes for the header parts */
  const uint8_t exp_magic[4] = {'T','C','M','D'};
  assert_mem(buf, exp_magic, 4, "turret cmd magic");
  assert_true(buf[4]==0x01, "turret cmd version");
  assert_true(buf[5]==11, "turret cmd controller_id");
  /* cmd_id LE at [6..7] */
  assert_true(buf[6]==0x06 && buf[7]==0x00, "turret cmd_id le");
  /* seq LE at [8..9] = 42 */
  assert_true(buf[8]==42 && buf[9]==0x00, "turret seq le");
  /* payload_size at [18..19] = 9 */
  assert_true(buf[18]==9 && buf[19]==0x00, "turret payload_size le");
  /* payload[0] motion_type */
  assert_true(buf[20]==(uint8_t)STC_MOTION_SPEED, "turret motion type byte");

  /* parse back */
  stc_icd_cmd_t parsed;
  rc = stc_icd_cmd_parse(&cfg, buf, out_len, &parsed);
  assert_true(rc==STC_ICD_OK, "cmd_parse turret");
  assert_true(parsed.payload_decoded==1, "turret payload decoded");
  assert_eq_u16(parsed.cmd_id, STC_CMD_TURRET, "turret cmd_id parsed");
  assert_eq_u16(parsed.seq, 42, "turret seq parsed");
  assert_eq_u64(parsed.timestamp_ms, 123456789ull, "turret ts parsed");
  assert_true(parsed.u.turret.motion_type==STC_MOTION_SPEED, "turret motion parsed");
}

static void test_rsp_pack_parse(void) {
  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();
  cfg.strict_version = 1;

  stc_icd_rsp_t rsp;
  memset(&rsp, 0, sizeof(rsp));
  rsp.version = 0x01;
  rsp.cmd_id = STC_CMD_REQ_SW_VERSION;
  rsp.seq = 77;
  rsp.result = STC_RSP_ACK;
  rsp.err_code = STC_ERR_NO_ERROR;

  const char* s = "stc-v0.1";
  rsp.payload_size = (uint16_t)strlen(s);
  memcpy(rsp.payload, s, rsp.payload_size);

  uint8_t buf[256];
  size_t out_len=0;
  stc_icd_rc_t rc = stc_icd_rsp_pack(&cfg, &rsp, buf, sizeof(buf), &out_len);
  assert_true(rc==STC_ICD_OK, "rsp_pack");

  /* parse back */
  stc_icd_rsp_t parsed;
  rc = stc_icd_rsp_parse(&cfg, buf, out_len, &parsed);
  assert_true(rc==STC_ICD_OK, "rsp_parse");
  assert_eq_u16(parsed.cmd_id, STC_CMD_REQ_SW_VERSION, "rsp cmd_id");
  assert_eq_u16(parsed.seq, 77, "rsp seq");
  assert_true(parsed.result==STC_RSP_ACK, "rsp ack");
  assert_eq_u16(parsed.payload_size, (uint16_t)strlen(s), "rsp payload size");
  assert_true(memcmp(parsed.payload, s, strlen(s))==0, "rsp payload content");
}

static void test_tlmh_pack_parse(void) {
  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();

  stc_tlmh_t tlm;
  memset(&tlm, 0, sizeof(tlm));
  tlm.version = 0x01;
  tlm.seq = 1000;
  tlm.timestamp_ms = 55555;

  for (int i=0;i<(int)STC_ICD_AXES_COUNT;i++) {
    tlm.position[i] = (float)i;
    tlm.rates[i] = (float)(100+i);
  }

  uint8_t buf[512];
  size_t out_len=0;
  stc_icd_rc_t rc = stc_icd_tlmh_pack(&cfg, &tlm, buf, sizeof(buf), &out_len);
  assert_true(rc==STC_ICD_OK, "tlmh_pack");

  stc_tlmh_t parsed;
  rc = stc_icd_tlmh_parse(&cfg, buf, out_len, &parsed);
  assert_true(rc==STC_ICD_OK, "tlmh_parse");
  assert_eq_u16(parsed.seq, 1000, "tlmh seq");

  /* spot check */
  assert_true(parsed.position[0]==0.0f, "tlmh pos0");
  assert_true(parsed.position[17]==17.0f, "tlmh pos17");
  assert_true(parsed.rates[0]==100.0f, "tlmh rate0");
  assert_true(parsed.rates[17]==117.0f, "tlmh rate17");
}

static void test_tlml_pack_parse(void) {
  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();
  cfg.strict_version = 1;

  stc_tlml_t tlm;
  memset(&tlm, 0, sizeof(tlm));
  tlm.version = 0x01;
  tlm.controller_id = 7;
  tlm.seq = 222;
  tlm.timestamp_ms = 8888;
  tlm.state = 3;
  tlm.uptime_sec = 1234;
  tlm.temperature_c = 25.5f;
  tlm.humidity_pct = 45.0f;
  tlm.pressure_hpa = 1013.25f;
  tlm.lat = 35.123;
  tlm.lon = 24.456;
  tlm.alt_m = 120.0f;
  tlm.heading_deg = 270.0f;
  for (int i = 0; i < STC_ICD_AXES_COUNT; ++i) {
    tlm.motor_status[i] = (uint8_t)i;
  }

  for (int i = 0; i < STC_ICD_LASER_COUNT; ++i) {
    tlm.laser_status[i] = (uint8_t)(0xA0u + i);
  }
  tlm.level_x = 0.1f;
  tlm.level_y = -0.2f;
  tlm.error_count = 3;
  tlm.errors[0] = 0xAABBCCDDu;
  tlm.errors[1] = 0x11223344u;
  tlm.errors[2] = 0x55667788u;

  uint8_t buf[512];
  size_t out_len=0;
  stc_icd_rc_t rc = stc_icd_tlml_pack(&cfg, &tlm, buf, sizeof(buf), &out_len);
  assert_true(rc==STC_ICD_OK, "tlml_pack");
  assert_true(out_len == STC_ICD_TLML_BASE_SIZE + 4u * tlm.error_count,
              "tlml packed base size + errors");
  assert_true(STC_ICD_TLML_BASE_SIZE == 100u, "tlml canonical base size");

  stc_tlml_t parsed;
  rc = stc_icd_tlml_parse(&cfg, buf, out_len, &parsed);
  assert_true(rc==STC_ICD_OK, "tlml_parse");

  assert_true(parsed.version == 0x01, "tlml version");
  assert_true(parsed.controller_id == 7, "tlml controller_id");
  assert_eq_u16(parsed.seq, 222, "tlml seq");
  assert_eq_u64(parsed.timestamp_ms, 8888, "tlml timestamp");
  assert_true(parsed.state == 3, "tlml state");
  assert_eq_u32(parsed.uptime_sec, 1234, "tlml uptime");
  assert_true(parsed.temperature_c == 25.5f, "tlml temperature");
  assert_true(parsed.humidity_pct == 45.0f, "tlml humidity");
  assert_true(parsed.pressure_hpa == 1013.25f, "tlml pressure");
  assert_true(parsed.lat == 35.123, "tlml lat");
  assert_true(parsed.lon == 24.456, "tlml lon");
  assert_true(parsed.alt_m == 120.0, "tlml alt");
  assert_true(parsed.heading_deg == 270.0, "tlml heading");
  assert_mem(parsed.motor_status,
             tlm.motor_status,
             STC_ICD_AXES_COUNT,
             "tlml motor_status[18]");

  assert_mem(parsed.laser_status,
             tlm.laser_status,
             STC_ICD_LASER_COUNT,
             "tlml laser_status[8]");
  assert_true(parsed.level_x == 0.1f, "tlml level_x");
  assert_true(parsed.level_y == -0.2f, "tlml level_y");
  assert_eq_u16(parsed.error_count, 3, "tlml error_count");
  assert_eq_u32(parsed.errors[0], 0xAABBCCDDu, "tlml err0");
  assert_eq_u32(parsed.errors[2], 0x55667788u, "tlml err2");
}

static void test_cmd_mirror_id(void) {
  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();

  stc_icd_cmd_t cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.version = 1;
  cmd.controller_id = 11;
  cmd.cmd_id = STC_CMD_MIRROR; /* 0x0007 */
  cmd.seq = 1;
  cmd.timestamp_ms = 1;
  cmd.payload_decoded = 1;
  cmd.u.mirror.mirror_id = 0xFF;
  cmd.u.mirror.motion_type = STC_MOTION_ABSOLUTE;
  cmd.u.mirror.elevation = 12.34f;

  uint8_t buf[128];
  size_t out_len=0;
  stc_icd_rc_t rc = stc_icd_cmd_pack(&cfg, &cmd, buf, sizeof(buf), &out_len);
  assert_true(rc==STC_ICD_OK, "mirror cmd pack");

  /* check cmd_id bytes */
  assert_true(buf[6]==0x07 && buf[7]==0x00, "mirror cmd_id le bytes");
}

static void test_tlml_declared_error_count_mismatch(void) {
  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();

  stc_tlml_t tlm;
  memset(&tlm, 0, sizeof(tlm));
  tlm.version = 0x01;
  tlm.error_count = 2;
  tlm.errors[0] = 0x11111111u;
  tlm.errors[1] = 0x22222222u;

  uint8_t buf[512];
  size_t out_len = 0;
  stc_icd_rc_t rc = stc_icd_tlml_pack(&cfg, &tlm, buf, sizeof(buf), &out_len);
  assert_true(rc == STC_ICD_OK, "tlml_pack mismatch frame setup");

  /* Corrupt only the declared error count, keep trailing bytes unchanged. */
  buf[out_len - ((size_t)tlm.error_count * 4u) - 1u] = (uint8_t)(tlm.error_count + 1u);

  stc_tlml_t parsed;
  rc = stc_icd_tlml_parse(&cfg, buf, out_len, &parsed);
  assert_true(rc == STC_ICD_EPROTO, "tlml parse declared count mismatch");
}

int main(void) {
  test_cmd_turret_pack_parse();
  test_rsp_pack_parse();
  test_tlmh_pack_parse();
  test_tlml_pack_parse();
  test_cmd_mirror_id();
  test_tlml_declared_error_count_mismatch();

  printf("OK\n");
  return 0;
}
