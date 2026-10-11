/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Transport-agnostic binary ICD parser/packer for STC command and telemetry frames.
 *
 * Design notes:
 *   This module is the only code that should know exact binary wire layout for
 *   CMD/RSP (TCP request/response control) and TLMH/TLML (UDP telemetry).
 *   Encoding/decoding is manual little-endian byte handling; struct casting is
 *   intentionally avoided to keep ABI/packing behavior explicit and portable.
 *   Implementation ownership: stc_icd.c in the ICD codec module.
 *   Real-time sensitivity: bounded codec operations suitable for runtime paths.
 *   Interface type: transport/protocol-facing, hardware-agnostic binary framing.
 *   Threading/ownership: caller owns buffers and synchronization.
 */

#ifndef STC_ICD_H
#define STC_ICD_H

/*
 * STC ICD codec (transport-agnostic)
 * - Parses/builds TCP CMD and RSP frames
 * - Parses/builds UDP telemetry frames (TLMH/TLML)
 * - No struct-casting; all little-endian manual packing/unpacking
 *
 * Project decisions:
 * - MIRROR cmd_id = 0x0007
 * - MANUALPOS uses strict 4xdouble (32B) encoding
 * - Wire magics follow ICD: "TCMD" and "TRSP"
 */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Limits ----------------------------- */

#define STC_ICD_MAX_PAYLOAD       2048u
#define STC_ICD_AXES_COUNT        18u
#define STC_ICD_LASER_COUNT       8u
#define STC_ICD_TLML_BASE_SIZE    100u
#define STC_ICD_MAX_ERRORS        64u

/* ----------------------------- Status codes ----------------------------- */

typedef enum stc_icd_rc_e {
  STC_ICD_OK        = 0,
  STC_ICD_EINVAL    = -1,
  STC_ICD_EBUFS     = -2,
  STC_ICD_EPROTO    = -3,
  STC_ICD_ERANGE    = -4
} stc_icd_rc_t;

/* ----------------------------- Config (magic, strictness) ----------------------------- */

typedef struct stc_icd_cfg_s {
  uint8_t cmd_magic[4];   /* default: {'T','C','M','D'} */
  uint8_t rsp_magic[4];   /* default: {'T','R','S','P'} */
  uint8_t tlmh_magic[4];  /* default: {'T','L','M','H'} */
  uint8_t tlml_magic[4];  /* default: {'T','L','M','L'} */

  uint8_t protocol_version_default; /* default: 0x01 */

  /* strictness knobs */
  int strict_magic;   /* if 1: reject mismatched magic, else accept */
  int strict_version; /* if 1: reject version != protocol_version_default */
} stc_icd_cfg_t;

stc_icd_cfg_t stc_icd_parse_cfg_default(void);

/* ----------------------------- Command IDs ----------------------------- */

typedef enum stc_cmd_id_e {
  STC_CMD_CU_STATE        = 0x0001,
  STC_CMD_SELFLEV         = 0x0002,
  STC_CMD_IBIT            = 0x0003,
  STC_CMD_STORAGE         = 0x0004,
  STC_CMD_LASER           = 0x0005,
  STC_CMD_TURRET          = 0x0006,
  STC_CMD_MIRROR          = 0x0007, /* PROJECT DECISION */
  STC_CMD_SUT             = 0x0008,
  /* 0x0009 unused */
  STC_CMD_RESET           = 0x000A,
  STC_CMD_DELLOG          = 0x000B,
  STC_CMD_FACTORY         = 0x000C,
  STC_CMD_SELFTEST        = 0x000D,
  STC_CMD_SHUTDOWN        = 0x000E,
  STC_CMD_CONFIG          = 0x000F,
  STC_CMD_MANUALPOS       = 0x0010,
  STC_CMD_POSMODE         = 0x0011,
  STC_CMD_REQ_SW_VERSION  = 0x0012,
  STC_CMD_REQ_HW_INFO     = 0x0013,
  STC_CMD_REQ_WORK_TIME   = 0x0014
} stc_cmd_id_t;

/* motion_type is used in TURRET/SUT/MIRROR payloads */
typedef enum stc_motion_type_e {
  STC_MOTION_RELATIVE = 0,
  STC_MOTION_ABSOLUTE = 1,
  STC_MOTION_SPEED    = 2
} stc_motion_type_t;

/* ----------------------------- TCP CMD frame ----------------------------- */
/*
 * Canonical encoding we implement:
 *   magic[4]        = "TCMD"
 *   version         u8
 *   controller_id   u8
 *   cmd_id          u16 LE
 *   seq             u16 LE
 *   timestamp_ms    u64 LE
 *   payload_size    u16 LE
 *   payload[payload_size]
 *
 * Header size = 20 bytes.
 */

typedef struct stc_cmd_payload_laser_s {
  uint8_t id;
  uint8_t on_off;
} stc_cmd_payload_laser_t;

typedef struct stc_cmd_payload_turret_s {
  uint8_t motion_type;
  float az;
  float el;
} stc_cmd_payload_turret_t;

typedef struct stc_cmd_payload_sut_s {
  uint8_t id;         /* 0xFF = all */
  uint8_t motion_type;
  float az;
  float el;
} stc_cmd_payload_sut_t;

typedef struct stc_cmd_payload_mirror_s {
  uint8_t mirror_id;  /* 0xFF = all */
  uint8_t motion_type;
  float elevation;
} stc_cmd_payload_mirror_t;

typedef struct stc_cmd_payload_cu_state_s {
  uint8_t state;
} stc_cmd_payload_cu_state_t;

typedef struct stc_cmd_payload_u8_s {
  uint8_t v;
} stc_cmd_payload_u8_t;

typedef struct stc_cmd_payload_manualpos_s {
  /* strict wire encoding: 4xdouble (lat, lon, alt, heading), 32 bytes total */
  double lat;
  double lon;
  double alt;
  double heading;
  int encoded_as_double; /* always 1 for valid MANUALPOS payloads */
} stc_cmd_payload_manualpos_t;

typedef struct stc_icd_cmd_s {
  uint8_t magic[4];
  uint8_t version;
  uint8_t controller_id;
  uint16_t cmd_id;
  uint16_t seq;
  uint64_t timestamp_ms;
  uint16_t payload_size;

  /* parsed payload (for known cmd_id) */
  int payload_decoded;

  union {
    stc_cmd_payload_cu_state_t cu_state;
    stc_cmd_payload_u8_t selflev;   /* start/stop */
    stc_cmd_payload_u8_t ibit;      /* start/stop */
    stc_cmd_payload_laser_t laser;
    stc_cmd_payload_turret_t turret;
    stc_cmd_payload_sut_t sut;
    stc_cmd_payload_mirror_t mirror;
    stc_cmd_payload_manualpos_t manualpos;
    stc_cmd_payload_u8_t posmode;
  } u;

  /* raw payload copy (always filled if payload_size > 0) */
  uint8_t payload_raw[STC_ICD_MAX_PAYLOAD];
} stc_icd_cmd_t;

/** Parse one TCP CMD wire frame into host representation plus decoded payload view. */
stc_icd_rc_t stc_icd_cmd_parse(const stc_icd_cfg_t* cfg,
                               const uint8_t* buf, size_t len,
                               stc_icd_cmd_t* out);

/** Pack one TCP CMD frame using canonical little-endian wire encoding. */
stc_icd_rc_t stc_icd_cmd_pack(const stc_icd_cfg_t* cfg,
                              const stc_icd_cmd_t* cmd,
                              uint8_t* out_buf, size_t out_cap,
                              size_t* out_len);

/* ----------------------------- TCP RSP frame ----------------------------- */
/*
 * Canonical encoding we implement:
 *   magic[4]      = "TRSP"
 *   version       u8
 *   cmd_id        u16 LE
 *   seq           u16 LE
 *   result        u8   (0=ACK, 1=NACK)
 *   err_code      u16 LE
 *   payload_size  u16 LE
 *   payload[payload_size]
 *
 * Header size = 14 bytes.
 */

typedef enum stc_rsp_result_e {
  STC_RSP_ACK  = 0x00,
  STC_RSP_NACK = 0x01
} stc_rsp_result_t;

typedef enum stc_nack_err_e {
  STC_ERR_NO_ERROR                    = 0x0000,
  STC_ERR_CLIENT_UNKNOWN              = 0x0001,
  STC_ERR_PROTOCOL_MISMATCH           = 0x0002,
  STC_ERR_SYNTAX_ERROR                = 0x0003,
  STC_ERR_VALUE_OUT_OF_RANGE          = 0x0004,
  STC_ERR_NOT_ALLOWED_STATE           = 0x0005,
  STC_ERR_INSUFFICIENT_RES            = 0x0006,
  STC_ERR_TIMEOUT                     = 0x0007,
  STC_ERR_COMMAND_NOT_IMPLEMENTED     = 0x0008,
  STC_ERR_NOT_CONFIGURED              = 0x0009,
  STC_ERR_AXIS_NOT_CONNECTED          = 0x000A,
  STC_ERR_PAYLOAD_TOO_LARGE           = 0x000B,
  STC_ERR_GENERIC                     = 0x00FF
} stc_nack_err_t;


typedef struct stc_icd_rsp_s {
  uint8_t magic[4];
  uint8_t version;
  uint16_t cmd_id;
  uint16_t seq;
  uint8_t result;
  uint16_t err_code;     // <-- was uint8_t
  uint16_t payload_size;
  uint8_t payload[STC_ICD_MAX_PAYLOAD];
} stc_icd_rsp_t;


/** Parse one TCP RSP wire frame (ACK/NACK path) from STC server to controller. */
stc_icd_rc_t stc_icd_rsp_parse(const stc_icd_cfg_t* cfg,
                               const uint8_t* buf, size_t len,
                               stc_icd_rsp_t* out);

/** Pack one TCP RSP frame preserving strict ICD field order and endianness. */
stc_icd_rc_t stc_icd_rsp_pack(const stc_icd_cfg_t* cfg,
                              const stc_icd_rsp_t* rsp,
                              uint8_t* out_buf, size_t out_cap,
                              size_t* out_len);

/* ----------------------------- UDP Telemetry (TLMH/TLML) ----------------------------- */

/*
 * NOTE: Telemetry exact field list may evolve; we implement a stable,
 * deterministic encoding that matches the common structure described in the ICD:
 *
 * TLMH (fixed):
 *   magic[4] = "TLMH"
 *   version u8
 *   seq u16
 *   timestamp_ms u64
 *   position[18] float
 *   rates[18]    float
 *   acceleration[18] float
 * Total size = 231 bytes (15-byte header + 3 * 72-byte vectors)
 */

typedef struct stc_tlmh_s {
  uint8_t magic[4];
  uint8_t version;
  uint16_t seq;
  uint64_t timestamp_ms;
  float position[STC_ICD_AXES_COUNT];
  float rates[STC_ICD_AXES_COUNT];
  float acceleration[STC_ICD_AXES_COUNT];
} stc_tlmh_t;

/** Parse high-rate TLMH telemetry frame (fixed-size kinematic stream). */
stc_icd_rc_t stc_icd_tlmh_parse(const stc_icd_cfg_t* cfg,
                                const uint8_t* buf, size_t len,
                                stc_tlmh_t* out);

/** Pack high-rate TLMH telemetry frame in canonical binary order. */
stc_icd_rc_t stc_icd_tlmh_pack(const stc_icd_cfg_t* cfg,
                               const stc_tlmh_t* tlm,
                               uint8_t* out_buf, size_t out_cap,
                               size_t* out_len);

/*
 * TLML (variable diagnostics list):
 *   magic[4] = "TLML"
 *   version u8
 *   controller_id u8
 *   seq u16
 *   timestamp_ms u64
 *   state u8
 *   uptime_sec u32
 *   temperature_c float
 *   humidity_pct float
 *   pressure_hpa float
 *   lat double
 *   lon double
 *   alt_m double
 *   heading_deg double
 *   motor_status[18] u8
 *     Canonical axis order:
 *       0  ST az
 *       1  ST el
 *       2  SUT1 az
 *       3  SUT1 el
 *       ...
 *       16 SUT8 az
 *       17 SUT8 el
 *   laser_status[8] u8
 *     SUT laser status:
 *       index 0 = SUT1 laser
 *       ...
 *       index 7 = SUT8 laser
 *       value 0 = OFF / unavailable
 *       value 1 = ON
 *   level_x float
 *   level_y float
 *   error_count u8
 *   errors[error_count] u32
 *
 * Base size without errors = 100 bytes.
 */

typedef struct stc_tlml_s {
  uint8_t magic[4];
  uint8_t version;
  uint8_t controller_id;
  uint16_t seq;
  uint64_t timestamp_ms;

  uint8_t state;
  uint32_t uptime_sec;

  float temperature_c;
  float humidity_pct;
  float pressure_hpa;

  double lat;
  double lon;
  double alt_m;
  double heading_deg;

  uint8_t motor_status[STC_ICD_AXES_COUNT];
  uint8_t laser_status[STC_ICD_LASER_COUNT];

  float level_x;
  float level_y;

  uint8_t error_count;
  uint32_t errors[STC_ICD_MAX_ERRORS];
} stc_tlml_t;

/** Parse low-rate TLML telemetry including variable-length diagnostic list. */
stc_icd_rc_t stc_icd_tlml_parse(const stc_icd_cfg_t* cfg,
                                const uint8_t* buf, size_t len,
                                stc_tlml_t* out);

/** Pack low-rate TLML telemetry and trailing diagnostics array. */
stc_icd_rc_t stc_icd_tlml_pack(const stc_icd_cfg_t* cfg,
                               const stc_tlml_t* tlm,
                               uint8_t* out_buf, size_t out_cap,
                               size_t* out_len);

#ifdef __cplusplus
}
#endif

#endif /* STC_ICD_H */
