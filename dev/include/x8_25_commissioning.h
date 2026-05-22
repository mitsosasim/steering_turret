/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Commissioning-only API for persistent MYACTUATOR X8-25 parameter writes/reads.
 *
 * Design notes:
 *   Owned by x8_25 commissioning module and separated from runtime-safe API.
 *   Not real-time sensitive: endpoints are setup/maintenance, not motion loop paths.
 *   Protocol/hardware-facing vendor command surface for CAN-connected actuators.
 *   Caller owns sequencing, locking, and when persistent writes are allowed.
 */

#ifndef X8_25_COMMISSIONING_H
#define X8_25_COMMISSIONING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "x8_25.h"

/*
 * Commissioning-only API (persistent/vendor configuration writes).
 *
 * This header is intentionally separate from x8_25.h so production runtime
 * code paths do not get accidental access to persistent drive configuration
 * entry points. To export these functions from x8_25.c, build with:
 *   -DSTC_ENABLE_COMMISSIONING
 */


typedef enum {
    MYACT_PID_CURRENT_KP  = 0x01,
    MYACT_PID_CURRENT_KI  = 0x02,
    MYACT_PID_SPEED_KP    = 0x04,
    MYACT_PID_SPEED_KI    = 0x05,
    MYACT_PID_POSITION_KP = 0x07,
    MYACT_PID_POSITION_KI = 0x08,
    MYACT_PID_POSITION_KD = 0x09
} myact_pid_index_t;

typedef enum {
    MYACT_ACCEL_POSITION_ACC = 0x00,
    MYACT_ACCEL_POSITION_DEC = 0x01,
    MYACT_ACCEL_SPEED_ACC    = 0x02,
    MYACT_ACCEL_SPEED_DEC    = 0x03
} myact_accel_index_t;

typedef enum {
    MYACT_BAUD_CODE_0 = 0, /* RS485 115200, CAN 500k */
    MYACT_BAUD_CODE_1 = 1, /* RS485 500k,   CAN 1M   */
    MYACT_BAUD_CODE_2 = 2, /* RS485 1M      */
    MYACT_BAUD_CODE_3 = 3, /* RS485 1.5M    */
    MYACT_BAUD_CODE_4 = 4  /* RS485 2.5M    */
} myact_baud_code_t;

typedef enum {
    MYACT_FUNC_CLEAR_MULTI_TURN            = 0x01,
    MYACT_FUNC_CANID_FILTER_ENABLE         = 0x02,
    MYACT_FUNC_ERROR_TX_ENABLE             = 0x03,
    MYACT_FUNC_SAVE_MULTI_TURN_ON_POWEROFF = 0x04,
    MYACT_FUNC_SET_CANID                   = 0x05,
    MYACT_FUNC_SET_POSITIVE_LIMIT          = 0x06,
    MYACT_FUNC_SET_NEGATIVE_LIMIT          = 0x07
} myact_function_index_t;

int myact_pid_read(myact_ctx_t* ctx, myact_pid_index_t index, float* out_value);
int myact_pid_write_ram(myact_ctx_t* ctx, myact_pid_index_t index, float value);
/* Persistent write: stores PID to non-volatile memory; run only in controlled commissioning windows. */
int myact_pid_write_rom(myact_ctx_t* ctx, myact_pid_index_t index, float value);
int myact_accel_read(myact_ctx_t* ctx, myact_accel_index_t index, int32_t* out_accel_dps2);
/* Persistent write: updates startup acceleration defaults and survives reboot/power-cycle. */
int myact_accel_write_rom(myact_ctx_t* ctx, myact_accel_index_t index, uint32_t accel_dps2);
int myact_read_encoder_raw(myact_ctx_t* ctx, int32_t* out_raw);
int myact_read_encoder_offset(myact_ctx_t* ctx, int32_t* out_offset);
int myact_write_zero_offset(myact_ctx_t* ctx, int32_t encoder_offset);
int myact_write_zero_from_current(myact_ctx_t* ctx, int32_t* out_encoder_offset);
int myact_set_comm_protect_ms(myact_ctx_t* ctx, uint32_t timeout_ms);
int myact_set_baud_code(myact_ctx_t* ctx, myact_baud_code_t code);
int myact_function_control(myact_ctx_t* ctx, myact_function_index_t index, int32_t value);
int myact_clear_multi_turn(myact_ctx_t* ctx);
int myact_set_canid_filter(myact_ctx_t* ctx, bool enable);
int myact_set_error_status_broadcast(myact_ctx_t* ctx, bool enable);
int myact_set_multi_turn_save_on_poweroff(myact_ctx_t* ctx, bool enable);
int myact_set_canid_via_function(myact_ctx_t* ctx, uint8_t new_node_id);
int myact_set_positive_limit_raw(myact_ctx_t* ctx, int32_t raw_value);
int myact_set_negative_limit_raw(myact_ctx_t* ctx, int32_t raw_value);
int myact_set_positive_limit_deg(myact_ctx_t* ctx, double deg_value);
int myact_set_negative_limit_deg(myact_ctx_t* ctx, double deg_value);
int myact_canid_read(myact_ctx_t* ctx, uint8_t* out_node_id);
int myact_canid_write(myact_ctx_t* ctx, uint8_t new_node_id);

#ifdef __cplusplus
}
#endif

#endif /* X8_25_COMMISSIONING_H */
