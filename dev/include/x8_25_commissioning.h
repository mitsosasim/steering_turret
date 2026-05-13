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

int myact_pid_read(myact_ctx_t* ctx, myact_pid_index_t index, float* out_value);
int myact_pid_write_ram(myact_ctx_t* ctx, myact_pid_index_t index, float value);
int myact_pid_write_rom(myact_ctx_t* ctx, myact_pid_index_t index, float value);
int myact_accel_read(myact_ctx_t* ctx, myact_accel_index_t index, int32_t* out_accel_dps2);
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
int myact_set_error_tx_enable(myact_ctx_t* ctx, bool enable);
int myact_set_save_multi_turn_on_poweroff(myact_ctx_t* ctx, bool enable);
int myact_set_canid_via_function(myact_ctx_t* ctx, uint8_t new_node_id);
int myact_set_positive_limit_raw(myact_ctx_t* ctx, int32_t raw_value);
int myact_set_negative_limit_raw(myact_ctx_t* ctx, int32_t raw_value);
int myact_canid_read(myact_ctx_t* ctx, uint8_t* out_node_id);
int myact_canid_write(myact_ctx_t* ctx, uint8_t new_node_id);

#ifdef __cplusplus
}
#endif

#endif /* X8_25_COMMISSIONING_H */
