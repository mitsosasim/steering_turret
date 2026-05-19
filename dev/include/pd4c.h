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

#ifndef PD4C_H
#define PD4C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * PD4-C CANopen low-level library (SocketCAN).
 *
 * Design:
 * - Library is single-threaded (no internal threads).
 * - The application owns threading and may call pd4c_poll() from a comms thread.
 * - If multiple threads call send APIs, protect ctx with an external mutex.
 *
 * Runtime strategy (your requirement):
 * - Use PDOs for real-time commands/feedback.
 * - Use SDO only for non-real-time setup/diagnostics and objects not in PDOs.
 *
 * Fixed PDO IDs used (per your dump):
 *   RPDO1: 0x200+node -> 6040(u16) + 6060(i8)
 *   RPDO2: 0x300+node -> 607A(i32) + 6081(u16)
 *   RPDO3: 0x400+node -> 6042(i16)
 *   TPDO1: 0x180+node -> 6041(u16) + 6061(i8)
 *   TPDO2: 0x280+node -> 6064(i32)
 *   TPDO3: 0x380+node -> 6044(??; parse by DLC: 2->i16, 4->i32)
 *   HB:    0x700+node -> 1 byte state
 *   EMCY:  0x080+node -> 8 bytes (CiA301)
 */

typedef enum {
    PD4C_OK          = 0,
    PD4C_EINVAL      = -1,
    PD4C_EIO         = -2,
    PD4C_ETIMEDOUT   = -3,
    PD4C_ESDO_ABORT  = -4,
    PD4C_ESTATE      = -5
} pd4c_rc_t;

const char* pd4c_strerror(int rc);

typedef enum {
    PD4C_LOG_ERROR = 0,
    PD4C_LOG_WARN  = 1,
    PD4C_LOG_INFO  = 2,
    PD4C_LOG_DEBUG = 3
} pd4c_log_level_t;

typedef void (*pd4c_log_fn)(pd4c_log_level_t lvl, const char* msg, void* user);

typedef enum {
    PD4C_NMT_BOOTUP       = 0x00,
    PD4C_NMT_STOPPED      = 0x04,
    PD4C_NMT_OPERATIONAL  = 0x05,
    PD4C_NMT_PREOP        = 0x7F
} pd4c_nmt_state_t;

typedef enum {
    PD4C_MODE_PROFILE_POSITION = 1,
    PD4C_MODE_PROFILE_VELOCITY = 3,
    PD4C_MODE_VELOCITY         = 2
} pd4c_op_mode_t;

typedef struct {
    uint16_t statusword;     // TPDO1
    int8_t   mode_display;   // TPDO1
    int32_t  pos_actual;     // TPDO2
    int32_t  vel_actual;     // TPDO3 (normalized to i32)
    bool     have_tpdo1;
    bool     have_tpdo2;
    bool     have_tpdo3;

    pd4c_nmt_state_t hb_state;
    bool     have_hb;

    // EMCY (CiA301)
    uint16_t emcy_error_code;
    uint8_t  emcy_error_reg;
    uint8_t  emcy_mfg[5];
    bool     have_emcy;

    // monotonic timestamps (ms)
    uint64_t ts_tpdo1_ms;
    uint64_t ts_tpdo2_ms;
    uint64_t ts_tpdo3_ms;
    uint64_t ts_hb_ms;
    uint64_t ts_emcy_ms;
} pd4c_telemetry_t;

typedef struct pd4c_ctx {
    int      sock;
    char     ifname[32];
    uint8_t  node_id;

    uint32_t sdo_tx_id; // 0x600+node
    uint32_t sdo_rx_id; // 0x580+node

    uint32_t rpdo1_id;  // 0x200+node
    uint32_t rpdo2_id;  // 0x300+node
    uint32_t rpdo3_id;  // 0x400+node

    uint32_t tpdo1_id;  // 0x180+node
    uint32_t tpdo2_id;  // 0x280+node
    uint32_t tpdo3_id;  // 0x380+node

    uint32_t hb_id;     // 0x700+node
    uint32_t emcy_id;   // 0x080+node

    int      sdo_timeout_ms;

    pd4c_log_fn log_cb;
    void*       log_user;

    pd4c_telemetry_t telem;
} pd4c_ctx_t;

/* --------- Core open/close --------- */
int  pd4c_open(pd4c_ctx_t* ctx, const char* ifname, uint8_t node_id);
void pd4c_close(pd4c_ctx_t* ctx);

void pd4c_set_logger(pd4c_ctx_t* ctx, pd4c_log_fn fn, void* user);
void pd4c_set_sdo_timeout(pd4c_ctx_t* ctx, int timeout_ms);
const pd4c_telemetry_t* pd4c_telem(const pd4c_ctx_t* ctx);

/* --------- Polling / parsing --------- */
/* Event bitmask returned by pd4c_poll() */
enum {
    PD4C_EVT_NONE      = 0,
    PD4C_EVT_TPDO1     = 1u << 0,
    PD4C_EVT_TPDO2     = 1u << 1,
    PD4C_EVT_TPDO3     = 1u << 2,
    PD4C_EVT_HEARTBEAT = 1u << 3,
    PD4C_EVT_EMCY      = 1u << 4
};

/* Poll socket for incoming CAN frames and update telemetry.
 * timeout_ms: -1 block forever, 0 non-blocking, >0 ms
 * returns: >=0 event bitmask, <0 error
 */
int pd4c_poll(pd4c_ctx_t* ctx, int timeout_ms);

/* --------- NMT helpers --------- */
int pd4c_nmt_start(pd4c_ctx_t* ctx);
int pd4c_nmt_stop(pd4c_ctx_t* ctx);
int pd4c_nmt_preop(pd4c_ctx_t* ctx);
int pd4c_nmt_reset_node(pd4c_ctx_t* ctx);

/* --------- SDO expedited (non real-time) --------- */
int pd4c_sdo_read_u8 (pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint8_t*  out);
int pd4c_sdo_read_u16(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint16_t* out);
int pd4c_sdo_read_u32(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint32_t* out);
int pd4c_sdo_read_i32(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, int32_t*  out);

int pd4c_sdo_write_u8 (pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint8_t  v);
int pd4c_sdo_write_u16(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint16_t v);
int pd4c_sdo_write_u32(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint32_t v);
int pd4c_sdo_write_i32(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, int32_t  v);

/* --------- Runtime PDO send APIs --------- */
/* RPDO1: 6040 + 6060 */
int pd4c_rpdo1_send_control_mode(pd4c_ctx_t* ctx, uint16_t controlword, int8_t mode);

/* RPDO2: 607A + 6081 */
int pd4c_rpdo2_send_pp(pd4c_ctx_t* ctx, int32_t target_pos, uint16_t profile_vel);

/* RPDO3: 6042 */
int pd4c_rpdo3_send_target_velocity(pd4c_ctx_t* ctx, int16_t target_vel);

/* --------- DS402 convenience (uses PDO + telemetry) --------- */
const char* pd4c_ds402_state_str(uint16_t statusword);
int pd4c_ds402_quick_stop(pd4c_ctx_t* ctx, int8_t mode);

/* Profile Position move with correct DS402 "new set-point" handshake.
 *
 * This function:
 *  - assumes caller owns the ctx and polling thread (single-threaded lib).
 *  - uses RPDO2 to send 607A+6081, and RPDO1 to toggle controlword bit4.
 *  - enforces bounded timeouts and progresses by calling pd4c_poll().
 *
 * Params:
 *  target_pos: position in counts (device units in 0x607A)
 *  profile_vel: profile velocity in units of 0x6081 (whatever your mapping expects)
 *  is_relative: 0 = absolute positioning, 1 = relative (CiA402 bit6 is set when relative)
 *  timeout_ms: total per-handshake wait bound (>= 50 recommended; typical 200..600)
 */
int pd4c_pp_move(pd4c_ctx_t* ctx,
                 int32_t target_pos,
                 uint16_t profile_vel,
                 int is_relative,
                 int timeout_ms);

/* Sends DS402 enable sequence via RPDO1 (mode is sent too).
 * Requires TPDO1 updates (statusword) to observe completion.
 */
int pd4c_ds402_enable_operation(pd4c_ctx_t* ctx, int8_t mode, int timeout_ms);

/* Fault reset pulse (DS402 bit7) via RPDO1.
 * If you want a specific pattern (e.g., 0x6F/0x7F), do it at application level.
 */
int pd4c_ds402_fault_reset(pd4c_ctx_t* ctx, int8_t mode, int timeout_ms);

/* Quick stop command (clear bit2) via RPDO1. */
int pd4c_ds402_quick_stop(pd4c_ctx_t* ctx, int8_t mode);

/* --------- Unit helpers (optional) --------- */
/* For your reducer: 100 motor rev = 1 turret rev.
 * These helpers do not assume a specific "counts per rev" (that depends on device config).
 * They are placeholders for the layer above to define scaling.
 */
double pd4c_deg_clamp(double deg, double min_deg, double max_deg);

#ifdef __cplusplus
}
#endif

#endif /* PD4C_H */
