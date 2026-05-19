/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Implementation/declarations for the STC development codebase.
 *
 * Design notes:
 *   Documentation/comment ownership header only; no runtime behavior changes.
 */

#include "stc_icd.h"

#include <assert.h>
#include <string.h>

/* ----------------------------- Endian helpers ----------------------------- */

static uint16_t rd_u16_le(const uint8_t* p) {
  return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}
static uint32_t rd_u32_le(const uint8_t* p) {
  return (uint32_t)(p[0] |
                   ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) |
                   ((uint32_t)p[3] << 24));
}
static uint64_t rd_u64_le(const uint8_t* p) {
  uint64_t v = 0;
  v |= (uint64_t)p[0];
  v |= ((uint64_t)p[1] << 8);
  v |= ((uint64_t)p[2] << 16);
  v |= ((uint64_t)p[3] << 24);
  v |= ((uint64_t)p[4] << 32);
  v |= ((uint64_t)p[5] << 40);
  v |= ((uint64_t)p[6] << 48);
  v |= ((uint64_t)p[7] << 56);
  return v;
}
static float rd_f32_le(const uint8_t* p) {
  uint32_t u = rd_u32_le(p);
  float f;
  memcpy(&f, &u, sizeof(f));
  return f;
}
static double rd_f64_le(const uint8_t* p) {
  uint64_t u = rd_u64_le(p);
  double d;
  memcpy(&d, &u, sizeof(d));
  return d;
}

static void wr_u16_le(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}
static void wr_u32_le(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}
static void wr_u64_le(uint8_t* p, uint64_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
  p[4] = (uint8_t)((v >> 32) & 0xFF);
  p[5] = (uint8_t)((v >> 40) & 0xFF);
  p[6] = (uint8_t)((v >> 48) & 0xFF);
  p[7] = (uint8_t)((v >> 56) & 0xFF);
}
static void wr_f32_le(uint8_t* p, float f) {
  uint32_t u;
  memcpy(&u, &f, sizeof(u));
  wr_u32_le(p, u);
}
static void wr_f64_le(uint8_t* p, double d) {
  uint64_t u;
  memcpy(&u, &d, sizeof(u));
  wr_u64_le(p, u);
}

static int magic_eq4(const uint8_t a[4], const uint8_t b[4]) {
  return (a[0]==b[0] && a[1]==b[1] && a[2]==b[2] && a[3]==b[3]);
}

stc_icd_cfg_t stc_icd_parse_cfg_default(void) {
  stc_icd_cfg_t cfg;
  cfg.cmd_magic[0]  = 'T'; cfg.cmd_magic[1]  = 'C'; cfg.cmd_magic[2]  = 'M'; cfg.cmd_magic[3]  = 'D';
  cfg.rsp_magic[0]  = 'T'; cfg.rsp_magic[1]  = 'R'; cfg.rsp_magic[2]  = 'S'; cfg.rsp_magic[3]  = 'P';
  cfg.tlmh_magic[0] = 'T'; cfg.tlmh_magic[1] = 'L'; cfg.tlmh_magic[2] = 'M'; cfg.tlmh_magic[3] = 'H';
  cfg.tlml_magic[0] = 'T'; cfg.tlml_magic[1] = 'L'; cfg.tlml_magic[2] = 'M'; cfg.tlml_magic[3] = 'L';
  cfg.protocol_version_default = 0x01;
  cfg.strict_magic = 0;
  cfg.strict_version = 0;
  return cfg;
}

/* ----------------------------- CMD parse/pack ----------------------------- */

static stc_icd_rc_t decode_cmd_payload(stc_icd_cmd_t* out) {
  out->payload_decoded = 0;

  const uint16_t id = out->cmd_id;
  const uint16_t n  = out->payload_size;
  const uint8_t* p  = out->payload_raw;

  switch (id) {
    case STC_CMD_CU_STATE:
      if (n != 1) return STC_ICD_EPROTO;
      out->u.cu_state.state = p[0];
      out->payload_decoded = 1;
      return STC_ICD_OK;

    case STC_CMD_SELFLEV:
    case STC_CMD_IBIT:
    case STC_CMD_POSMODE:
      if (n != 1) return STC_ICD_EPROTO;
      if (id == STC_CMD_SELFLEV) out->u.selflev.v = p[0];
      else if (id == STC_CMD_IBIT) out->u.ibit.v = p[0];
      else out->u.posmode.v = p[0];
      out->payload_decoded = 1;
      return STC_ICD_OK;

    case STC_CMD_STORAGE:
    case STC_CMD_RESET:
    case STC_CMD_DELLOG:
    case STC_CMD_FACTORY:
    case STC_CMD_SELFTEST:
    case STC_CMD_SHUTDOWN:
    case STC_CMD_REQ_SW_VERSION:
    case STC_CMD_REQ_HW_INFO:
    case STC_CMD_REQ_WORK_TIME:
      if (n != 0) return STC_ICD_EPROTO;
      out->payload_decoded = 1;
      return STC_ICD_OK;

    case STC_CMD_LASER:
      if (n != 2) return STC_ICD_EPROTO;
      out->u.laser.id = p[0];
      out->u.laser.on_off = p[1];
      out->payload_decoded = 1;
      return STC_ICD_OK;

    case STC_CMD_TURRET:
      if (n != 1 + 4 + 4) return STC_ICD_EPROTO;
      out->u.turret.motion_type = p[0];
      out->u.turret.az = rd_f32_le(&p[1]);
      out->u.turret.el = rd_f32_le(&p[5]);
      out->payload_decoded = 1;
      return STC_ICD_OK;

    case STC_CMD_SUT:
      if (n != 2 + 4 + 4) return STC_ICD_EPROTO;
      out->u.sut.id = p[0];
      out->u.sut.motion_type = p[1];
      out->u.sut.az = rd_f32_le(&p[2]);
      out->u.sut.el = rd_f32_le(&p[6]);
      out->payload_decoded = 1;
      return STC_ICD_OK;

    case STC_CMD_MIRROR:
      if (n != 2 + 4) return STC_ICD_EPROTO;
      out->u.mirror.mirror_id = p[0];
      out->u.mirror.motion_type = p[1];
      out->u.mirror.elevation = rd_f32_le(&p[2]);
      out->payload_decoded = 1;
      return STC_ICD_OK;

    case STC_CMD_MANUALPOS:
      if (n != 32) return STC_ICD_EPROTO;
      out->u.manualpos.lat = rd_f64_le(&p[0]);
      out->u.manualpos.lon = rd_f64_le(&p[8]);
      out->u.manualpos.alt = rd_f64_le(&p[16]);
      out->u.manualpos.heading = rd_f64_le(&p[24]);
      out->u.manualpos.encoded_as_double = 1;
      out->payload_decoded = 1;
      return STC_ICD_OK;

    case STC_CMD_CONFIG:
    default:
      /* unknown or config => keep raw only */
      out->payload_decoded = 0;
      return STC_ICD_OK;
  }
}

stc_icd_rc_t stc_icd_cmd_parse(const stc_icd_cfg_t* cfg,
                               const uint8_t* buf, size_t len,
                               stc_icd_cmd_t* out) {
  if (!cfg || !buf || !out) return STC_ICD_EINVAL;

  /* header size = 20 */
  if (len < 20) return STC_ICD_EPROTO;

  memset(out, 0, sizeof(*out));

  memcpy(out->magic, &buf[0], 4);
  out->version = buf[4];
  out->controller_id = buf[5];
  out->cmd_id = rd_u16_le(&buf[6]);
  out->seq = rd_u16_le(&buf[8]);
  out->timestamp_ms = rd_u64_le(&buf[10]);
  out->payload_size = rd_u16_le(&buf[18]);

  if (cfg->strict_magic && !magic_eq4(out->magic, cfg->cmd_magic)) {
    return STC_ICD_EPROTO;
  }
  if (cfg->strict_version && out->version != cfg->protocol_version_default) {
    return STC_ICD_EPROTO;
  }

  const size_t total = 20u + (size_t)out->payload_size;
  if (total != len) return STC_ICD_EPROTO;

  if (out->payload_size > STC_ICD_MAX_PAYLOAD) return STC_ICD_EBUFS;

  if (out->payload_size > 0) {
    memcpy(out->payload_raw, &buf[20], out->payload_size);
  }

  /* decode known payloads if possible */
  return decode_cmd_payload(out);
}

static stc_icd_rc_t encode_cmd_payload(const stc_icd_cmd_t* cmd,
                                      uint8_t* payload_out,
                                      uint16_t* payload_len_out) {
  if (!cmd || !payload_out || !payload_len_out) return STC_ICD_EINVAL;

  const uint16_t id = cmd->cmd_id;

  /* if caller already filled payload_raw + payload_size, honor that */
  if (!cmd->payload_decoded && cmd->payload_size > 0) {
    if (cmd->payload_size > STC_ICD_MAX_PAYLOAD) return STC_ICD_EBUFS;
    memcpy(payload_out, cmd->payload_raw, cmd->payload_size);
    *payload_len_out = cmd->payload_size;
    return STC_ICD_OK;
  }

  switch (id) {
    case STC_CMD_CU_STATE:
      payload_out[0] = cmd->u.cu_state.state;
      *payload_len_out = 1;
      return STC_ICD_OK;

    case STC_CMD_SELFLEV:
      payload_out[0] = cmd->u.selflev.v;
      *payload_len_out = 1;
      return STC_ICD_OK;

    case STC_CMD_IBIT:
      payload_out[0] = cmd->u.ibit.v;
      *payload_len_out = 1;
      return STC_ICD_OK;

    case STC_CMD_POSMODE:
      payload_out[0] = cmd->u.posmode.v;
      *payload_len_out = 1;
      return STC_ICD_OK;

    case STC_CMD_STORAGE:
    case STC_CMD_RESET:
    case STC_CMD_DELLOG:
    case STC_CMD_FACTORY:
    case STC_CMD_SELFTEST:
    case STC_CMD_SHUTDOWN:
    case STC_CMD_REQ_SW_VERSION:
    case STC_CMD_REQ_HW_INFO:
    case STC_CMD_REQ_WORK_TIME:
      *payload_len_out = 0;
      return STC_ICD_OK;

    case STC_CMD_LASER:
      payload_out[0] = cmd->u.laser.id;
      payload_out[1] = cmd->u.laser.on_off;
      *payload_len_out = 2;
      return STC_ICD_OK;

    case STC_CMD_TURRET:
      payload_out[0] = cmd->u.turret.motion_type;
      wr_f32_le(&payload_out[1], cmd->u.turret.az);
      wr_f32_le(&payload_out[5], cmd->u.turret.el);
      *payload_len_out = (uint16_t)(1 + 4 + 4);
      return STC_ICD_OK;

    case STC_CMD_SUT:
      payload_out[0] = cmd->u.sut.id;
      payload_out[1] = cmd->u.sut.motion_type;
      wr_f32_le(&payload_out[2], cmd->u.sut.az);
      wr_f32_le(&payload_out[6], cmd->u.sut.el);
      *payload_len_out = (uint16_t)(2 + 4 + 4);
      return STC_ICD_OK;

    case STC_CMD_MIRROR:
      payload_out[0] = cmd->u.mirror.mirror_id;
      payload_out[1] = cmd->u.mirror.motion_type;
      wr_f32_le(&payload_out[2], cmd->u.mirror.elevation);
      *payload_len_out = (uint16_t)(2 + 4);
      return STC_ICD_OK;

    case STC_CMD_MANUALPOS:
      /* default to double (32B) */
      wr_f64_le(&payload_out[0],  cmd->u.manualpos.lat);
      wr_f64_le(&payload_out[8],  cmd->u.manualpos.lon);
      wr_f64_le(&payload_out[16], cmd->u.manualpos.alt);
      wr_f64_le(&payload_out[24], cmd->u.manualpos.heading);
      *payload_len_out = 32;
      return STC_ICD_OK;

    case STC_CMD_CONFIG:
    default:
      /* send raw payload if provided; else empty */
      if (cmd->payload_size > 0) {
        if (cmd->payload_size > STC_ICD_MAX_PAYLOAD) return STC_ICD_EBUFS;
        memcpy(payload_out, cmd->payload_raw, cmd->payload_size);
        *payload_len_out = cmd->payload_size;
      } else {
        *payload_len_out = 0;
      }
      return STC_ICD_OK;
  }
}

stc_icd_rc_t stc_icd_cmd_pack(const stc_icd_cfg_t* cfg,
                              const stc_icd_cmd_t* cmd,
                              uint8_t* out_buf, size_t out_cap,
                              size_t* out_len) {
  if (!cfg || !cmd || !out_buf || !out_len) return STC_ICD_EINVAL;

  uint8_t payload[STC_ICD_MAX_PAYLOAD];
  uint16_t payload_len = 0;

  stc_icd_rc_t rc = encode_cmd_payload(cmd, payload, &payload_len);
  if (rc != STC_ICD_OK) return rc;

  const size_t total = 20u + (size_t)payload_len;
  if (total > out_cap) return STC_ICD_EBUFS;

  /* header */
  memcpy(&out_buf[0], cfg->cmd_magic, 4);
  out_buf[4] = cmd->version ? cmd->version : cfg->protocol_version_default;
  out_buf[5] = cmd->controller_id;
  wr_u16_le(&out_buf[6], cmd->cmd_id);
  wr_u16_le(&out_buf[8], cmd->seq);
  wr_u64_le(&out_buf[10], cmd->timestamp_ms);
  wr_u16_le(&out_buf[18], payload_len);

  /* payload */
  if (payload_len > 0) {
    memcpy(&out_buf[20], payload, payload_len);
  }

  *out_len = total;
  return STC_ICD_OK;
}

/* ----------------------------- RSP parse/pack ----------------------------- */

stc_icd_rc_t stc_icd_rsp_parse(const stc_icd_cfg_t* cfg,
                               const uint8_t* buf, size_t len,
                               stc_icd_rsp_t* out) {
  if (!cfg || !buf || !out) return STC_ICD_EINVAL;
  if (len < 14) return STC_ICD_EPROTO;

  memset(out, 0, sizeof(*out));

  memcpy(out->magic, &buf[0], 4);
  out->version = buf[4];
  out->cmd_id = rd_u16_le(&buf[5]);
  out->seq = rd_u16_le(&buf[7]);
  out->result = buf[9];
  out->err_code = rd_u16_le(&buf[10]);
  out->payload_size = rd_u16_le(&buf[12]);

  if (cfg->strict_magic && !magic_eq4(out->magic, cfg->rsp_magic)) {
    return STC_ICD_EPROTO;
  }
  if (cfg->strict_version && out->version != cfg->protocol_version_default) {
    return STC_ICD_EPROTO;
  }

  const size_t total = 14u + (size_t)out->payload_size;
  if (total != len) return STC_ICD_EPROTO;
  if (out->payload_size > STC_ICD_MAX_PAYLOAD) return STC_ICD_EBUFS;

  if (out->payload_size > 0) {
    memcpy(out->payload, &buf[14], out->payload_size);
  }
  return STC_ICD_OK;
}

stc_icd_rc_t stc_icd_rsp_pack(const stc_icd_cfg_t* cfg,
                              const stc_icd_rsp_t* rsp,
                              uint8_t* out_buf, size_t out_cap,
                              size_t* out_len) {
  if (!cfg || !rsp || !out_buf || !out_len) return STC_ICD_EINVAL;

  const size_t total = 14u + (size_t)rsp->payload_size;
  if (total > out_cap) return STC_ICD_EBUFS;
  if (rsp->payload_size > STC_ICD_MAX_PAYLOAD) return STC_ICD_EBUFS;

  memcpy(&out_buf[0], cfg->rsp_magic, 4);
  out_buf[4] = rsp->version ? rsp->version : cfg->protocol_version_default;
  wr_u16_le(&out_buf[5], rsp->cmd_id);
  wr_u16_le(&out_buf[7], rsp->seq);
  out_buf[9] = rsp->result;
  wr_u16_le(&out_buf[10], rsp->err_code);
  wr_u16_le(&out_buf[12], rsp->payload_size);

  if (rsp->payload_size > 0) {
    memcpy(&out_buf[14], rsp->payload, rsp->payload_size);
  }

  *out_len = total;
  return STC_ICD_OK;
}

/* ----------------------------- TLMH parse/pack ----------------------------- */

stc_icd_rc_t stc_icd_tlmh_parse(const stc_icd_cfg_t* cfg,
                                const uint8_t* buf, size_t len,
                                stc_tlmh_t* out) {
  if (!cfg || !buf || !out) return STC_ICD_EINVAL;

  const size_t header = 4u + 1u + 2u + 8u;
  const size_t arrays = (STC_ICD_AXES_COUNT * 4u) * 3u;
  const size_t expect = header + arrays;
  if (len != expect) return STC_ICD_EPROTO;

  memset(out, 0, sizeof(*out));
  memcpy(out->magic, &buf[0], 4);

  if (cfg->strict_magic && !magic_eq4(out->magic, cfg->tlmh_magic)) return STC_ICD_EPROTO;

  out->version = buf[4];
  out->seq = rd_u16_le(&buf[5]);
  out->timestamp_ms = rd_u64_le(&buf[7]);

  size_t off = 15;
  for (size_t i = 0; i < STC_ICD_AXES_COUNT; i++) {
    out->position[i] = rd_f32_le(&buf[off]); off += 4;
  }
  for (size_t i = 0; i < STC_ICD_AXES_COUNT; i++) {
    out->rates[i] = rd_f32_le(&buf[off]); off += 4;
  }
  for (size_t i = 0; i < STC_ICD_AXES_COUNT; i++) {
    out->acceleration[i] = rd_f32_le(&buf[off]); off += 4;
  }

  if (cfg->strict_version && out->version != cfg->protocol_version_default) return STC_ICD_EPROTO;
  return STC_ICD_OK;
}

stc_icd_rc_t stc_icd_tlmh_pack(const stc_icd_cfg_t* cfg,
                               const stc_tlmh_t* tlm,
                               uint8_t* out_buf, size_t out_cap,
                               size_t* out_len) {
  if (!cfg || !tlm || !out_buf || !out_len) return STC_ICD_EINVAL;

  const size_t header = 4u + 1u + 2u + 8u;
  const size_t arrays = (STC_ICD_AXES_COUNT * 4u) * 3u;
  const size_t total = header + arrays;
  if (total > out_cap) return STC_ICD_EBUFS;

  memcpy(&out_buf[0], cfg->tlmh_magic, 4);
  out_buf[4] = tlm->version ? tlm->version : cfg->protocol_version_default;
  wr_u16_le(&out_buf[5], tlm->seq);
  wr_u64_le(&out_buf[7], tlm->timestamp_ms);

  size_t off = 15;
  for (size_t i = 0; i < STC_ICD_AXES_COUNT; i++) {
    wr_f32_le(&out_buf[off], tlm->position[i]); off += 4;
  }
  for (size_t i = 0; i < STC_ICD_AXES_COUNT; i++) {
    wr_f32_le(&out_buf[off], tlm->rates[i]); off += 4;
  }
  for (size_t i = 0; i < STC_ICD_AXES_COUNT; i++) {
    wr_f32_le(&out_buf[off], tlm->acceleration[i]); off += 4;
  }

  *out_len = total;
  return STC_ICD_OK;
}

/* ----------------------------- TLML parse/pack ----------------------------- */

stc_icd_rc_t stc_icd_tlml_parse(const stc_icd_cfg_t* cfg,
                                const uint8_t* buf, size_t len,
                                stc_tlml_t* out) {
  if (!cfg || !buf || !out) return STC_ICD_EINVAL;

  assert(STC_ICD_TLML_BASE_SIZE == 100u);
  if (len < STC_ICD_TLML_BASE_SIZE) return STC_ICD_EPROTO;

  memset(out, 0, sizeof(*out));
  memcpy(out->magic, &buf[0], 4);
  if (cfg->strict_magic && !magic_eq4(out->magic, cfg->tlml_magic)) return STC_ICD_EPROTO;

  out->version = buf[4];
  out->controller_id = buf[5];
  out->seq = rd_u16_le(&buf[6]);
  out->timestamp_ms = rd_u64_le(&buf[8]);

  size_t off = 16;
  out->state = buf[off]; off += 1;
  out->uptime_sec = rd_u32_le(&buf[off]); off += 4;

  out->temperature_c = rd_f32_le(&buf[off]); off += 4;
  out->humidity_pct  = rd_f32_le(&buf[off]); off += 4;
  out->pressure_hpa  = rd_f32_le(&buf[off]); off += 4;

  out->lat = rd_f64_le(&buf[off]); off += 8;
  out->lon = rd_f64_le(&buf[off]); off += 8;
  out->alt_m = rd_f64_le(&buf[off]); off += 8;
  out->heading_deg = rd_f64_le(&buf[off]); off += 8;

  memcpy(out->motor_status, &buf[off], STC_ICD_AXES_COUNT); off += STC_ICD_AXES_COUNT;
  memcpy(out->laser_status, &buf[off], STC_ICD_LASER_COUNT); off += STC_ICD_LASER_COUNT;

  out->level_x = rd_f32_le(&buf[off]); off += 4;
  out->level_y = rd_f32_le(&buf[off]); off += 4;

  uint8_t declared = buf[off]; off += 1;

  if (len != (STC_ICD_TLML_BASE_SIZE + ((size_t)declared * 4u))) return STC_ICD_EPROTO;

  uint16_t use_n = declared;

  if (use_n > STC_ICD_MAX_ERRORS) use_n = STC_ICD_MAX_ERRORS;
  out->error_count = (uint8_t)use_n;

  for (uint16_t i = 0; i < use_n; i++) {
    out->errors[i] = rd_u32_le(&buf[off]); off += 4;
  }

  if (cfg->strict_version && out->version != cfg->protocol_version_default) return STC_ICD_EPROTO;
  return STC_ICD_OK;
}

stc_icd_rc_t stc_icd_tlml_pack(const stc_icd_cfg_t* cfg,
                               const stc_tlml_t* tlm,
                               uint8_t* out_buf, size_t out_cap,
                               size_t* out_len) {
  if (!cfg || !tlm || !out_buf || !out_len) return STC_ICD_EINVAL;

  uint16_t n = tlm->error_count;
  if (n > STC_ICD_MAX_ERRORS) n = STC_ICD_MAX_ERRORS;

  assert(STC_ICD_TLML_BASE_SIZE == 100u);
  const size_t total = STC_ICD_TLML_BASE_SIZE + ((size_t)n * 4u);
  if (total > out_cap) return STC_ICD_EBUFS;

  size_t off = 0;
  memcpy(&out_buf[off], cfg->tlml_magic, 4); off += 4;
  out_buf[off] = tlm->version ? tlm->version : cfg->protocol_version_default; off += 1;
  out_buf[off] = tlm->controller_id; off += 1;
  wr_u16_le(&out_buf[off], tlm->seq); off += 2;
  wr_u64_le(&out_buf[off], tlm->timestamp_ms); off += 8;

  out_buf[off] = tlm->state; off += 1;
  wr_u32_le(&out_buf[off], tlm->uptime_sec); off += 4;

  wr_f32_le(&out_buf[off], tlm->temperature_c); off += 4;
  wr_f32_le(&out_buf[off], tlm->humidity_pct); off += 4;
  wr_f32_le(&out_buf[off], tlm->pressure_hpa); off += 4;

  wr_f64_le(&out_buf[off], tlm->lat); off += 8;
  wr_f64_le(&out_buf[off], tlm->lon); off += 8;
  wr_f64_le(&out_buf[off], tlm->alt_m); off += 8;
  wr_f64_le(&out_buf[off], tlm->heading_deg); off += 8;

  memcpy(&out_buf[off], tlm->motor_status, STC_ICD_AXES_COUNT); off += STC_ICD_AXES_COUNT;
  memcpy(&out_buf[off], tlm->laser_status, STC_ICD_LASER_COUNT); off += STC_ICD_LASER_COUNT;

  wr_f32_le(&out_buf[off], tlm->level_x); off += 4;
  wr_f32_le(&out_buf[off], tlm->level_y); off += 4;

  out_buf[off] = (uint8_t)n; off += 1;
  assert(off == STC_ICD_TLML_BASE_SIZE);

  for (uint16_t i = 0; i < n; i++) {
    wr_u32_le(&out_buf[off], tlm->errors[i]); off += 4;
  }

  if (off != total) return STC_ICD_EPROTO;
  *out_len = total;
  return STC_ICD_OK;
}
