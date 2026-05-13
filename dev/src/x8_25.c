#include "x8_25.h"

#include "x8_25_commissioning.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define MYACT_CMD_ID_BASE           0x140u
#define MYACT_REPLY_ID_BASE         0x240u
#define MYACT_CANID_SERVICE_ID      0x300u
#define MYACT_MOTION_CMD_ID_BASE    0x400u
#define MYACT_MOTION_REPLY_ID_BASE  0x500u
enum {
    MYACT_STRICT_BAD_CAN_ID = -101,
    MYACT_STRICT_WRONG_OPCODE = -102,
    MYACT_STRICT_MALFORMED_PAYLOAD = -103,
    MYACT_STRICT_OUT_OF_RANGE_COMMAND = -104
};

typedef struct {
    bool has_status1;
    bool has_basic_fb;
    bool has_active_reply;
    bool has_motion_mode;
} myact_caps_t;

typedef struct {
    uint64_t ts_ms;
    uint8_t node_id;
    uint8_t last_cmd;
    myact_status1_t status1;
    myact_basic_feedback_t basic_feedback;
} myact_state_t;

static void myact_rebuild_ids(myact_ctx_t* ctx) {
    ctx->cmd_id         = MYACT_CMD_ID_BASE + ctx->node_id;
    ctx->reply_id       = MYACT_REPLY_ID_BASE + ctx->node_id;
    ctx->motion_cmd_id  = MYACT_MOTION_CMD_ID_BASE + ctx->node_id;
    ctx->motion_reply_id= MYACT_MOTION_REPLY_ID_BASE + ctx->node_id;
}

static uint64_t myact_now_ms(const myact_ctx_t* ctx) {
    return ctx->io.time_ms(ctx->io.user);
}

static uint16_t rd_u16_le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static int16_t rd_i16_le(const uint8_t* p) {
    return (int16_t)rd_u16_le(p);
}
static uint32_t rd_u32_le(const uint8_t* p) {
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}
static int32_t rd_i32_le(const uint8_t* p) {
    return (int32_t)rd_u32_le(p);
}
static void wr_u16_le(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}
static void wr_u32_le(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

double myact_clamp(double x, double x_min, double x_max) {
    if (x < x_min) return x_min;
    if (x > x_max) return x_max;
    return x;
}

static int myact_send_frame(myact_ctx_t* ctx, uint32_t id, const uint8_t data[8]) {
    myact_can_frame_t fr;
    if (!ctx || !data) return MYACT_EINVAL;
    memset(&fr, 0, sizeof(fr));
    fr.id = id;
    fr.dlc = 8;
    memcpy(fr.data, data, 8);
    return ctx->io.send(&fr, ctx->io.user);
}

static int myact_recv_frame(myact_ctx_t* ctx, uint32_t timeout_ms, myact_can_frame_t* out) {
    if (!ctx || !out) return MYACT_EINVAL;
    return ctx->io.recv(out, timeout_ms, ctx->io.user);
}

static int proto_expect(const myact_can_frame_t* fr, uint8_t opcode, uint8_t min_dlc) {
    if (!fr) return MYACT_EINVAL;
    if (fr->dlc < min_dlc) return MYACT_STRICT_MALFORMED_PAYLOAD;
    if (fr->data[0] != opcode) return MYACT_STRICT_WRONG_OPCODE;
    return MYACT_OK;
}

const char* myact_strerror(int rc) {
    switch (rc) {
        case MYACT_OK:        return "OK";
        case MYACT_EINVAL:    return "EINVAL";
        case MYACT_EIO:       return "EIO";
        case MYACT_ETIMEDOUT: return "ETIMEDOUT";
        case MYACT_EPROTO:    return "EPROTO";
        case MYACT_ERANGE:    return "ERANGE";
        case MYACT_ESTATE:    return "ESTATE";
        case MYACT_ENOTSUP:   return "ENOTSUP";
        case MYACT_EBUS:      return "EBUS";
        case MYACT_ENOMOTION: return "ENOMOTION";
        case MYACT_STRICT_BAD_CAN_ID:           return "STRICT_BAD_CAN_ID";
        case MYACT_STRICT_WRONG_OPCODE:         return "STRICT_WRONG_OPCODE";
        case MYACT_STRICT_MALFORMED_PAYLOAD:    return "STRICT_MALFORMED_PAYLOAD";
        case MYACT_STRICT_OUT_OF_RANGE_COMMAND: return "STRICT_OUT_OF_RANGE_COMMAND";
        default:              return "UNKNOWN";
    }
}

int myact_init(myact_ctx_t* ctx, uint8_t node_id, const myact_io_t* io) {
    if (!ctx || !io || !io->send || !io->recv || !io->time_ms) return MYACT_EINVAL;
    if (node_id < MYACT_NODE_ID_MIN || node_id > MYACT_NODE_ID_MAX) return MYACT_EINVAL;

    memset(ctx, 0, sizeof(*ctx));
    ctx->io = *io;
    ctx->timeout_ms = MYACT_DEFAULT_TIMEOUT_MS;
    ctx->node_id = node_id;
    myact_rebuild_ids(ctx);
    return MYACT_OK;
}

void myact_reset_cache(myact_ctx_t* ctx) {
    if (!ctx) return;
    memset(&ctx->cache, 0, sizeof(ctx->cache));
}

void myact_set_timeout_ms(myact_ctx_t* ctx, uint32_t timeout_ms) {
    if (!ctx) return;
    ctx->timeout_ms = (timeout_ms == 0u) ? MYACT_DEFAULT_TIMEOUT_MS : timeout_ms;
}

void myact_set_logger(myact_ctx_t* ctx, myact_log_fn log_fn, void* user) {
    if (!ctx) return;
    ctx->log_fn = log_fn;
    ctx->log_user = user;
}

void myact_set_node_id(myact_ctx_t* ctx, uint8_t node_id) {
    if (!ctx) return;
    if (node_id < MYACT_NODE_ID_MIN || node_id > MYACT_NODE_ID_MAX) return;
    ctx->node_id = node_id;
    myact_rebuild_ids(ctx);
}

const myact_cache_t* myact_cache(const myact_ctx_t* ctx) {
    return ctx ? &ctx->cache : NULL;
}

static int parse_status1(const myact_can_frame_t* fr, uint64_t ts_ms, myact_status1_t* out) {
    int rc = proto_expect(fr, MYACT_CMD_STATUS1, 8);
    if (!out) return MYACT_EINVAL;
    if (rc < 0) return rc;
    out->valid = true;
    out->ts_ms = ts_ms;
    out->motor_temp_c = (int8_t)fr->data[1];
    out->brake_release_command_state = (fr->data[3] != 0u);
    out->voltage_0p1v = rd_u16_le(&fr->data[4]);
    out->error_flags = rd_u16_le(&fr->data[6]);
    return MYACT_OK;
}

static int parse_status2(const myact_can_frame_t* fr, uint64_t ts_ms, myact_status2_t* out) {
    int rc = proto_expect(fr, MYACT_CMD_STATUS2, 8);
    if (!out) return MYACT_EINVAL;
    if (rc < 0) return rc;
    out->valid = true;
    out->ts_ms = ts_ms;
    out->motor_temp_c = (int8_t)fr->data[1];
    out->iq_0p01a = rd_i16_le(&fr->data[2]);
    out->speed_dps = rd_i16_le(&fr->data[4]);
    out->angle_deg = rd_i16_le(&fr->data[6]);
    return MYACT_OK;
}

static int parse_status3(const myact_can_frame_t* fr, uint64_t ts_ms, myact_status3_t* out) {
    int rc = proto_expect(fr, MYACT_CMD_STATUS3, 8);
    if (!out) return MYACT_EINVAL;
    if (rc < 0) return rc;
    out->valid = true;
    out->ts_ms = ts_ms;
    out->motor_temp_c = (int8_t)fr->data[1];
    out->phase_a_0p01a = rd_i16_le(&fr->data[2]);
    out->phase_b_0p01a = rd_i16_le(&fr->data[4]);
    out->phase_c_0p01a = rd_i16_le(&fr->data[6]);
    return MYACT_OK;
}

static int parse_basic_feedback(const myact_can_frame_t* fr, uint64_t ts_ms, myact_basic_feedback_t* out) {
    uint8_t cmd;
    if (!fr || !out || fr->dlc < 8) return MYACT_EPROTO;
    cmd = fr->data[0];
    if (!(cmd == MYACT_CMD_TORQUE || cmd == MYACT_CMD_SPEED ||
          cmd == MYACT_CMD_POSITION_ABS || cmd == MYACT_CMD_POSITION_INC)) {
        return MYACT_EPROTO;
    }
    out->valid = true;
    out->ts_ms = ts_ms;
    out->source_cmd = cmd;
    out->motor_temp_c = (int8_t)fr->data[1];
    out->iq_0p01a = rd_i16_le(&fr->data[2]);
    out->speed_dps = rd_i16_le(&fr->data[4]);
    out->angle_deg = rd_i16_le(&fr->data[6]);
    return MYACT_OK;
}

static int parse_singleturn_feedback(const myact_can_frame_t* fr, uint64_t ts_ms, myact_singleturn_feedback_t* out) {
    int rc = proto_expect(fr, MYACT_CMD_POSITION_SINGLE, 8);
    if (!out) return MYACT_EINVAL;
    if (rc < 0) return rc;
    out->valid = true;
    out->ts_ms = ts_ms;
    out->motor_temp_c = (int8_t)fr->data[1];
    out->iq_0p01a = rd_i16_le(&fr->data[2]);
    out->speed_dps = rd_i16_le(&fr->data[4]);
    out->encoder = rd_u16_le(&fr->data[6]);
    return MYACT_OK;
}

static int parse_singleturn_encoder(const myact_can_frame_t* fr, uint64_t ts_ms, myact_singleturn_encoder_t* out) {
    int rc = proto_expect(fr, MYACT_CMD_READ_ENCODER_SINGLE, 8);
    if (!out) return MYACT_EINVAL;
    if (rc < 0) return rc;
    out->valid = true;
    out->ts_ms = ts_ms;
    out->encoder_pos = rd_u16_le(&fr->data[2]);
    out->encoder_raw = rd_u16_le(&fr->data[4]);
    out->encoder_offset = rd_u16_le(&fr->data[6]);
    return MYACT_OK;
}

static uint16_t float_to_uint(float x, float x_min, float x_max, unsigned bits) {
    const float span = x_max - x_min;
    const float offset = x - x_min;
    const float max_int = (float)((1u << bits) - 1u);
    long v;
    if (span <= 0.0f) return 0u;
    v = lroundf((offset * max_int) / span);
    if (v < 0) v = 0;
    if ((unsigned long)v > (unsigned long)((1u << bits) - 1u)) v = (long)((1u << bits) - 1u);
    return (uint16_t)v;
}

static float uint_to_float(uint32_t x, float x_min, float x_max, unsigned bits) {
    const float span = x_max - x_min;
    const float max_int = (float)((1u << bits) - 1u);
    return ((float)x) * span / max_int + x_min;
}

int myact_motion_pack(float p_rad, float v_rad_s, float kp, float kd, float t_ff_nm, uint8_t out_data[8]) {
    uint16_t p_int;
    uint16_t v_int;
    uint16_t kp_int;
    uint16_t kd_int;
    uint16_t t_int;

    if (!out_data) return MYACT_EINVAL;
    if (p_rad < MYACT_MOTION_P_MIN_RAD || p_rad > MYACT_MOTION_P_MAX_RAD) return MYACT_ERANGE;
    if (v_rad_s < MYACT_MOTION_V_MIN_RAD_S || v_rad_s > MYACT_MOTION_V_MAX_RAD_S) return MYACT_ERANGE;
    if (kp < MYACT_MOTION_KP_MIN || kp > MYACT_MOTION_KP_MAX) return MYACT_ERANGE;
    if (kd < MYACT_MOTION_KD_MIN || kd > MYACT_MOTION_KD_MAX) return MYACT_ERANGE;
    if (t_ff_nm < MYACT_MOTION_T_MIN_NM || t_ff_nm > MYACT_MOTION_T_MAX_NM) return MYACT_ERANGE;

    p_int  = float_to_uint(p_rad,   MYACT_MOTION_P_MIN_RAD,   MYACT_MOTION_P_MAX_RAD,   16);
    v_int  = float_to_uint(v_rad_s, MYACT_MOTION_V_MIN_RAD_S, MYACT_MOTION_V_MAX_RAD_S, 12);
    kp_int = float_to_uint(kp,      MYACT_MOTION_KP_MIN,      MYACT_MOTION_KP_MAX,      12);
    kd_int = float_to_uint(kd,      MYACT_MOTION_KD_MIN,      MYACT_MOTION_KD_MAX,      12);
    t_int  = float_to_uint(t_ff_nm, MYACT_MOTION_T_MIN_NM,    MYACT_MOTION_T_MAX_NM,    12);

    out_data[0] = (uint8_t)(p_int >> 8);
    out_data[1] = (uint8_t)(p_int & 0xFFu);
    out_data[2] = (uint8_t)(v_int >> 4);
    out_data[3] = (uint8_t)(((v_int & 0x0Fu) << 4) | ((kp_int >> 8) & 0x0Fu));
    out_data[4] = (uint8_t)(kp_int & 0xFFu);
    out_data[5] = (uint8_t)(kd_int >> 4);
    out_data[6] = (uint8_t)(((kd_int & 0x0Fu) << 4) | ((t_int >> 8) & 0x0Fu));
    out_data[7] = (uint8_t)(t_int & 0xFFu);
    return MYACT_OK;
}

int myact_motion_unpack_reply(const myact_can_frame_t* fr, myact_motion_feedback_t* out) {
    uint16_t p_int;
    uint16_t v_int;
    uint16_t t_int;

    if (!fr || !out || fr->dlc < 6) return MYACT_EINVAL;

    p_int = (uint16_t)(((uint16_t)fr->data[1] << 8) | fr->data[2]);
    v_int = (uint16_t)(((uint16_t)fr->data[3] << 4) | ((fr->data[4] >> 4) & 0x0Fu));
    t_int = (uint16_t)((((uint16_t)fr->data[4] & 0x0Fu) << 8) | fr->data[5]);

    out->valid = true;
    out->p_rad = uint_to_float(p_int, MYACT_MOTION_P_MIN_RAD, MYACT_MOTION_P_MAX_RAD, 16);
    out->v_rad_s = uint_to_float(v_int, MYACT_MOTION_V_MIN_RAD_S, MYACT_MOTION_V_MAX_RAD_S, 12);
    out->t_nm = uint_to_float(t_int, MYACT_MOTION_T_MIN_NM, MYACT_MOTION_T_MAX_NM, 12);
    return MYACT_OK;
}

static int myact_parse_frame(myact_ctx_t* ctx, const myact_can_frame_t* fr) {
    uint64_t ts_ms;
    int ev = MYACT_EVENT_NONE;

    if (!ctx || !fr) return MYACT_EINVAL;
    ts_ms = myact_now_ms(ctx);

    if (fr->id == ctx->reply_id && fr->dlc >= 1) {
        switch (fr->data[0]) {
            case MYACT_CMD_READ_ENCODER_MULTI:
                if (fr->dlc >= 8) {
                    ctx->cache.encoder_multi = rd_i32_le(&fr->data[4]);
                    ctx->cache.encoder_multi_valid = true;
                    ctx->cache.encoder_multi_ts_ms = ts_ms;
                    ev |= MYACT_EVENT_ENCODER_MULTI;
                }
                break;
            case MYACT_CMD_READ_ENCODER_RAW:
                if (fr->dlc >= 8) {
                    ctx->cache.encoder_raw = rd_i32_le(&fr->data[4]);
                    ctx->cache.encoder_raw_valid = true;
                    ctx->cache.encoder_raw_ts_ms = ts_ms;
                    ev |= MYACT_EVENT_ENCODER_RAW;
                }
                break;
            case MYACT_CMD_READ_ENCODER_OFFSET:
            case MYACT_CMD_WRITE_ZERO_FROM_NOW:
                if (fr->dlc >= 8) {
                    ctx->cache.encoder_offset = rd_i32_le(&fr->data[4]);
                    ctx->cache.encoder_offset_valid = true;
                    ctx->cache.encoder_offset_ts_ms = ts_ms;
                    ev |= MYACT_EVENT_ENCODER_OFFSET;
                }
                break;
            case MYACT_CMD_READ_ENCODER_SINGLE:
                if (parse_singleturn_encoder(fr, ts_ms, &ctx->cache.singleturn_encoder) == MYACT_OK) {
                    ev |= MYACT_EVENT_ENCODER_SINGLE;
                }
                break;
            case MYACT_CMD_READ_ANGLE_MULTI:
                if (fr->dlc >= 8) {
                    ctx->cache.angle_multi_0p01deg = rd_i32_le(&fr->data[4]);
                    ctx->cache.angle_multi_valid = true;
                    ctx->cache.angle_multi_ts_ms = ts_ms;
                    ev |= MYACT_EVENT_ANGLE_MULTI;
                }
                break;
            case MYACT_CMD_READ_ANGLE_SINGLE:
                if (fr->dlc >= 6) {
                    ctx->cache.angle_single_0p01deg = rd_u16_le(&fr->data[6]);
                    ctx->cache.angle_single_valid = true;
                    ctx->cache.angle_single_ts_ms = ts_ms;
                    ev |= MYACT_EVENT_ANGLE_SINGLE;
                }
                break;
            case MYACT_CMD_GET_RUNMODE:
                if (fr->dlc >= 8) {
                    ctx->cache.operating_mode = (myact_runmode_t)fr->data[7];
                    ctx->cache.operating_mode_valid = true;
                    ctx->cache.operating_mode_ts_ms = ts_ms;
                    ev |= MYACT_EVENT_OPERATING_MODE;
                }
                break;
            case MYACT_CMD_STATUS1:
                if (parse_status1(fr, ts_ms, &ctx->cache.status1) == MYACT_OK) {
                    ev |= MYACT_EVENT_STATUS1;
                }
                break;
            case MYACT_CMD_STATUS2:
                if (parse_status2(fr, ts_ms, &ctx->cache.status2) == MYACT_OK) {
                    ev |= MYACT_EVENT_STATUS2;
                }
                break;
            case MYACT_CMD_STATUS3:
                if (parse_status3(fr, ts_ms, &ctx->cache.status3) == MYACT_OK) {
                    ev |= MYACT_EVENT_STATUS3;
                }
                break;
            case MYACT_CMD_TORQUE:
            case MYACT_CMD_SPEED:
            case MYACT_CMD_POSITION_ABS:
            case MYACT_CMD_POSITION_INC:
                if (parse_basic_feedback(fr, ts_ms, &ctx->cache.basic_feedback) == MYACT_OK) {
                    ev |= MYACT_EVENT_BASIC_FB;
                }
                break;
            case MYACT_CMD_POSITION_SINGLE:
                if (parse_singleturn_feedback(fr, ts_ms, &ctx->cache.singleturn_feedback) == MYACT_OK) {
                    ev |= MYACT_EVENT_SINGLETURN_FB;
                }
                break;
            case MYACT_CMD_RUNTIME:
                if (fr->dlc >= 8) {
                    ctx->cache.runtime_ms = rd_u32_le(&fr->data[4]);
                    ctx->cache.runtime_valid = true;
                    ctx->cache.runtime_ts_ms = ts_ms;
                    ev |= MYACT_EVENT_RUNTIME;
                }
                break;
            case MYACT_CMD_VERSION_DATE:
                if (fr->dlc >= 8) {
                    ctx->cache.version_date_yyyymmdd = rd_u32_le(&fr->data[4]);
                    ctx->cache.version_valid = true;
                    ctx->cache.version_ts_ms = ts_ms;
                    ev |= MYACT_EVENT_VERSION;
                }
                break;
            case MYACT_CMD_MODEL:
                if (fr->dlc >= 8) {
                    memcpy(ctx->cache.model_ascii7, &fr->data[1], 7);
                    ctx->cache.model_ascii7[7] = '\0';
                    ctx->cache.model_valid = true;
                    ctx->cache.model_ts_ms = ts_ms;
                    ev |= MYACT_EVENT_MODEL;
                }
                break;
            default:
                ctx->cache.last_raw_reply = *fr;
                ev |= MYACT_EVENT_RAW_REPLY;
                break;
        }
        return ev;
    }

    if (fr->id == ctx->motion_reply_id) {
        if (myact_motion_unpack_reply(fr, &ctx->cache.motion_feedback) == MYACT_OK) {
            ctx->cache.motion_feedback.ts_ms = ts_ms;
            ev |= MYACT_EVENT_MOTION_FB;
        }
        return ev;
    }

    if (fr->id == MYACT_CANID_SERVICE_ID && fr->dlc >= 8 && fr->data[0] == MYACT_CMD_CANID) {
        if (fr->data[2] == 1u) {
            uint16_t reply_id = rd_u16_le(&fr->data[6]);
            if (reply_id >= MYACT_REPLY_ID_BASE + MYACT_NODE_ID_MIN &&
                reply_id <= MYACT_REPLY_ID_BASE + MYACT_NODE_ID_MAX) {
                ctx->cache.current_can_id = (uint8_t)(reply_id - MYACT_REPLY_ID_BASE);
                ctx->cache.canid_valid = true;
                ctx->cache.canid_ts_ms = ts_ms;
                ev |= MYACT_EVENT_CANID;
            }
        }
        return ev;
    }

    return MYACT_EVENT_NONE;
}

int myact_poll(myact_ctx_t* ctx, uint32_t timeout_ms) {
    myact_can_frame_t fr;
    int rc;

    if (!ctx) return MYACT_EINVAL;
    rc = myact_recv_frame(ctx, timeout_ms, &fr);
    if (rc < 0) return rc;
    return myact_parse_frame(ctx, &fr);
}

static int myact_wait_for_reply(myact_ctx_t* ctx,
                                uint32_t reply_id,
                                int expected_cmd_or_neg1,
                                uint32_t timeout_ms,
                                myact_can_frame_t* out) {
    uint64_t start;
    uint64_t now;
    myact_can_frame_t fr;
    int rc;

    if (!ctx) return MYACT_EINVAL;
    start = myact_now_ms(ctx);

    for (;;) {
        uint32_t remain;
        now = myact_now_ms(ctx);
        if (now - start >= timeout_ms) return MYACT_ETIMEDOUT;
        remain = (uint32_t)(timeout_ms - (now - start));

        rc = myact_recv_frame(ctx, remain, &fr);
        if (rc < 0) return rc;

        (void)myact_parse_frame(ctx, &fr);

        if (fr.id != reply_id) {
            if (fr.id == ctx->reply_id || fr.id == ctx->motion_reply_id) {
                (void)myact_parse_frame(ctx, &fr);
            }
            continue;
        }
        if (fr.dlc != 8) return MYACT_EPROTO;
        if (expected_cmd_or_neg1 >= 0 && fr.data[0] != (uint8_t)expected_cmd_or_neg1) {
            (void)myact_parse_frame(ctx, &fr);
            if (ctx->log_fn) {
                char msg[120];
                (void)snprintf(msg, sizeof(msg),
                               "wait_for_reply opcode mismatch exp=0x%02X got=0x%02X id=0x%03X",
                               (unsigned)expected_cmd_or_neg1, (unsigned)fr.data[0], (unsigned)fr.id);
                ctx->log_fn(MYACT_LOG_DEBUG, msg, ctx->log_user);
            }
            continue;
        }

        if (out) *out = fr;
        return MYACT_OK;
    }
}

static int myact_txrx(myact_ctx_t* ctx,
                      uint32_t tx_id,
                      const uint8_t data[8],
                      uint32_t reply_id,
                      int expected_cmd_or_neg1,
                      myact_can_frame_t* out) {
    int rc;
    rc = myact_send_frame(ctx, tx_id, data);
    if (rc < 0) return rc;
    return myact_wait_for_reply(ctx, reply_id, expected_cmd_or_neg1, ctx->timeout_ms, out);
}

int myact_send_raw_single(myact_ctx_t* ctx, const uint8_t data[8]) {
    return myact_send_frame(ctx, ctx->cmd_id, data);
}

int myact_send_raw_motion(myact_ctx_t* ctx, const uint8_t data[8]) {
    return myact_send_frame(ctx, ctx->motion_cmd_id, data);
}

/* PID / accel */

/* Commissioning/private API: intentionally not declared in the runtime header. */
int myact_pid_read(myact_ctx_t* ctx, myact_pid_index_t index, float* out_value) {
    uint8_t tx[8] = { MYACT_CMD_READ_PID, (uint8_t)index, 0, 0, 0, 0, 0, 0 };
    myact_can_frame_t rx;
    uint32_t raw;
    int rc;

    if (!ctx || !out_value) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_READ_PID, &rx);
    if (rc < 0) return rc;
    raw = rd_u32_le(&rx.data[4]);
    memcpy(out_value, &raw, sizeof(raw));
    return MYACT_OK;
}

static int pid_write_common(myact_ctx_t* ctx, uint8_t cmd, myact_pid_index_t index, float value) {
    uint8_t tx[8] = { 0 };
    uint32_t raw;
    if (!ctx) return MYACT_EINVAL;
    memcpy(&raw, &value, sizeof(raw));
    tx[0] = cmd;
    tx[1] = (uint8_t)index;
    wr_u32_le(&tx[4], raw);
    return myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, cmd, NULL);
}

int myact_pid_write_ram(myact_ctx_t* ctx, myact_pid_index_t index, float value) {
    return pid_write_common(ctx, MYACT_CMD_WRITE_PID_RAM, index, value);
}

int myact_pid_write_rom(myact_ctx_t* ctx, myact_pid_index_t index, float value) {
    return pid_write_common(ctx, MYACT_CMD_WRITE_PID_ROM, index, value);
}

int myact_accel_read(myact_ctx_t* ctx, myact_accel_index_t index, int32_t* out_accel_dps2) {
    uint8_t tx[8] = { MYACT_CMD_READ_ACCEL, (uint8_t)index, 0, 0, 0, 0, 0, 0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out_accel_dps2) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_READ_ACCEL, &rx);
    if (rc < 0) return rc;
    *out_accel_dps2 = rd_i32_le(&rx.data[4]);
    return MYACT_OK;
}

int myact_accel_write_rom(myact_ctx_t* ctx, myact_accel_index_t index, uint32_t accel_dps2) {
    uint8_t tx[8] = { MYACT_CMD_WRITE_ACCEL_ROM, (uint8_t)index, 0, 0, 0, 0, 0, 0 };
    if (!ctx) return MYACT_EINVAL;
    wr_u32_le(&tx[4], accel_dps2);
    return myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_WRITE_ACCEL_ROM, NULL);
}

/* reads */


int myact_read_encoder_multi(myact_ctx_t* ctx, int32_t* out_encoder) {
    uint8_t tx[8] = { MYACT_CMD_READ_ENCODER_MULTI, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out_encoder) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_READ_ENCODER_MULTI, &rx);
    if (rc < 0) return rc;
    *out_encoder = rd_i32_le(&rx.data[4]);
    return MYACT_OK;
}

int myact_read_encoder_raw(myact_ctx_t* ctx, int32_t* out_encoder_raw) {
    uint8_t tx[8] = { MYACT_CMD_READ_ENCODER_RAW, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out_encoder_raw) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_READ_ENCODER_RAW, &rx);
    if (rc < 0) return rc;
    *out_encoder_raw = rd_i32_le(&rx.data[4]);
    return MYACT_OK;
}

int myact_read_encoder_offset(myact_ctx_t* ctx, int32_t* out_encoder_offset) {
    uint8_t tx[8] = { MYACT_CMD_READ_ENCODER_OFFSET, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out_encoder_offset) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_READ_ENCODER_OFFSET, &rx);
    if (rc < 0) return rc;
    *out_encoder_offset = rd_i32_le(&rx.data[4]);
    return MYACT_OK;
}

int myact_write_zero_offset(myact_ctx_t* ctx, int32_t encoder_offset) {
    uint8_t tx[8] = { MYACT_CMD_WRITE_ZERO_OFFSET, 0,0,0,0,0,0,0 };
    if (!ctx) return MYACT_EINVAL;
    wr_u32_le(&tx[4], (uint32_t)encoder_offset);
    return myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_WRITE_ZERO_OFFSET, NULL);
}

int myact_write_zero_from_current(myact_ctx_t* ctx, int32_t* out_encoder_offset) {
    uint8_t tx[8] = { MYACT_CMD_WRITE_ZERO_FROM_NOW, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out_encoder_offset) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_WRITE_ZERO_FROM_NOW, &rx);
    if (rc < 0) return rc;
    *out_encoder_offset = rd_i32_le(&rx.data[4]);
    return MYACT_OK;
}

int myact_read_singleturn_encoder(myact_ctx_t* ctx, myact_singleturn_encoder_t* out) {
    uint8_t tx[8] = { MYACT_CMD_READ_ENCODER_SINGLE, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_READ_ENCODER_SINGLE, &rx);
    if (rc < 0) return rc;
    return parse_singleturn_encoder(&rx, myact_now_ms(ctx), out);
}

int myact_read_angle_multi(myact_ctx_t* ctx, int32_t* out_angle_0p01deg) {
    uint8_t tx[8] = { MYACT_CMD_READ_ANGLE_MULTI, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out_angle_0p01deg) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_READ_ANGLE_MULTI, &rx);
    if (rc < 0) return rc;
    *out_angle_0p01deg = rd_i32_le(&rx.data[4]);
    return MYACT_OK;
}

int myact_read_angle_single(myact_ctx_t* ctx, uint16_t* out_angle_0p01deg) {
    uint8_t tx[8] = { MYACT_CMD_READ_ANGLE_SINGLE, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out_angle_0p01deg) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_READ_ANGLE_SINGLE, &rx);
    if (rc < 0) return rc;
    *out_angle_0p01deg = rd_u16_le(&rx.data[6]);
    return MYACT_OK;
}

int myact_read_operating_mode(myact_ctx_t* ctx, myact_runmode_t* out_mode) {
    uint8_t tx[8] = { MYACT_CMD_GET_RUNMODE, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out_mode) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_GET_RUNMODE, &rx);
    if (rc < 0) return rc;
    *out_mode = (myact_runmode_t)rx.data[7];
    return MYACT_OK;
}


int myact_wait_runmode(myact_ctx_t* ctx,
                       myact_runmode_t expected,
                       uint32_t timeout_ms,
                       uint32_t poll_ms) {
    uint64_t now;
    uint64_t deadline;
    myact_runmode_t rm = MYACT_RUNMODE_UNKNOWN;
    int rc;

    if (!ctx) return MYACT_EINVAL;
    if (poll_ms == 0u) poll_ms = 1u;

    now = myact_now_ms(ctx);
    deadline = now + (uint64_t)timeout_ms;

    for (;;) {
        rc = myact_read_operating_mode(ctx, &rm);
        if (rc < 0) return rc;
        if (rm == expected) return MYACT_OK;

        now = myact_now_ms(ctx);
        if (now >= deadline) break;

        {
            uint64_t remain = deadline - now;
            uint32_t sleep_ms = (remain < (uint64_t)poll_ms) ? (uint32_t)remain : poll_ms;
            usleep((useconds_t)sleep_ms * 1000u);
        }
    }

    if (rm == MYACT_RUNMODE_UNKNOWN) {
        printf("command acknowledged but motor did not enter requested closed-loop mode.\n");
    }
    return MYACT_ETIMEDOUT;
}
int myact_read_runtime(myact_ctx_t* ctx, uint32_t* out_runtime_ms) {
    uint8_t tx[8] = { MYACT_CMD_RUNTIME, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out_runtime_ms) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_RUNTIME, &rx);
    if (rc < 0) return rc;
    *out_runtime_ms = rd_u32_le(&rx.data[4]);
    return MYACT_OK;
}

int myact_read_version_date(myact_ctx_t* ctx, uint32_t* out_yyyymmdd) {
    uint8_t tx[8] = { MYACT_CMD_VERSION_DATE, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out_yyyymmdd) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_VERSION_DATE, &rx);
    if (rc < 0) return rc;
    *out_yyyymmdd = rd_u32_le(&rx.data[4]);
    return MYACT_OK;
}

int myact_read_motion_commissioning_snapshot(myact_ctx_t* ctx,
                                             myact_motion_commissioning_snapshot_t* out) {
    int rc;
    if (!ctx || !out) return MYACT_EINVAL;
    memset(out, 0, sizeof(*out));

    rc = myact_pid_read(ctx, MYACT_PID_SPEED_KP, &out->speed_kp);
    if (rc < 0) return rc;
    rc = myact_pid_read(ctx, MYACT_PID_SPEED_KI, &out->speed_ki);
    if (rc < 0) return rc;
    rc = myact_pid_read(ctx, MYACT_PID_POSITION_KP, &out->position_kp);
    if (rc < 0) return rc;
    rc = myact_pid_read(ctx, MYACT_PID_POSITION_KI, &out->position_ki);
    if (rc < 0) return rc;
    rc = myact_pid_read(ctx, MYACT_PID_POSITION_KD, &out->position_kd);
    if (rc < 0) return rc;

    rc = myact_accel_read(ctx, MYACT_ACCEL_POSITION_ACC, &out->pos_acc_dps2);
    if (rc < 0) return rc;
    rc = myact_accel_read(ctx, MYACT_ACCEL_POSITION_DEC, &out->pos_dec_dps2);
    if (rc < 0) return rc;
    rc = myact_accel_read(ctx, MYACT_ACCEL_SPEED_ACC, &out->speed_acc_dps2);
    if (rc < 0) return rc;
    rc = myact_accel_read(ctx, MYACT_ACCEL_SPEED_DEC, &out->speed_dec_dps2);
    if (rc < 0) return rc;

    rc = myact_read_status1(ctx, &out->status1);
    if (rc < 0) return rc;
    rc = myact_read_operating_mode(ctx, &out->runmode);
    if (rc < 0) return rc;
    return MYACT_OK;
}

int myact_read_model(myact_ctx_t* ctx, char out_ascii7[8]) {
    uint8_t tx[8] = { MYACT_CMD_MODEL, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out_ascii7) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_MODEL, &rx);
    if (rc < 0) return rc;
    memcpy(out_ascii7, &rx.data[1], 7);
    out_ascii7[7] = '\0';
    return MYACT_OK;
}

/* status */

int myact_read_status1(myact_ctx_t* ctx, myact_status1_t* out) {
    uint8_t tx[8] = { MYACT_CMD_STATUS1, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_STATUS1, &rx);
    if (rc < 0) return rc;
    return parse_status1(&rx, myact_now_ms(ctx), out);
}

int myact_read_status2(myact_ctx_t* ctx, myact_status2_t* out) {
    uint8_t tx[8] = { MYACT_CMD_STATUS2, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_STATUS2, &rx);
    if (rc < 0) return rc;
    return parse_status2(&rx, myact_now_ms(ctx), out);
}

int myact_read_status3(myact_ctx_t* ctx, myact_status3_t* out) {
    uint8_t tx[8] = { MYACT_CMD_STATUS3, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out) return MYACT_EINVAL;
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_STATUS3, &rx);
    if (rc < 0) return rc;
    return parse_status3(&rx, myact_now_ms(ctx), out);
}

/* control send-only */

int myact_shutdown_send(myact_ctx_t* ctx) {
    uint8_t tx[8] = { MYACT_CMD_SHUTDOWN, 0,0,0,0,0,0,0 };
    return myact_send_frame(ctx, ctx->cmd_id, tx);
}

int myact_stop_send(myact_ctx_t* ctx) {
    uint8_t tx[8] = { MYACT_CMD_STOP, 0,0,0,0,0,0,0 };
    return myact_send_frame(ctx, ctx->cmd_id, tx);
}

int myact_torque_send(myact_ctx_t* ctx, int16_t iq_0p01a) {
    uint8_t tx[8] = { MYACT_CMD_TORQUE, 0,0,0,0,0,0,0 };
    if (!ctx) return MYACT_EINVAL;
    wr_u16_le(&tx[4], (uint16_t)iq_0p01a);
    return myact_send_frame(ctx, ctx->cmd_id, tx);
}

int myact_speed_send(myact_ctx_t* ctx, int32_t speed_0p01dps) {
    uint8_t tx[8] = { MYACT_CMD_SPEED, 0,0,0,0,0,0,0 };
    if (!ctx) return MYACT_EINVAL;
    wr_u32_le(&tx[4], (uint32_t)speed_0p01dps);
    if (ctx->log_fn) {
        char msg[180];
        (void)snprintf(msg, sizeof(msg),
                       "A2 tx speed_dps=%.2f raw_0p01dps=%ld frame=%02X %02X %02X %02X %02X %02X %02X %02X",
                       ((double)speed_0p01dps) / 100.0,
                       (long)speed_0p01dps,
                       tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7]);
        ctx->log_fn(MYACT_LOG_DEBUG, msg, ctx->log_user);
    }
    return myact_send_frame(ctx, ctx->cmd_id, tx);
}

int myact_speed_dps_send(myact_ctx_t* ctx, double speed_dps) {
    double raw_f;
    if (!ctx) return MYACT_EINVAL;
    if (!isfinite(speed_dps)) return MYACT_ERANGE;
    raw_f = round(speed_dps * 100.0);
    if (raw_f < (double)INT32_MIN || raw_f > (double)INT32_MAX) return MYACT_ERANGE;
    return myact_speed_send(ctx, (int32_t)raw_f);
}

int myact_position_abs_send(myact_ctx_t* ctx, uint16_t max_speed_dps, int32_t angle_0p01deg) {
    uint8_t tx[8] = { MYACT_CMD_POSITION_ABS, 0,0,0,0,0,0,0 };
    if (!ctx) return MYACT_EINVAL;
    wr_u16_le(&tx[2], max_speed_dps);
    wr_u32_le(&tx[4], (uint32_t)angle_0p01deg);
    if (ctx->log_fn) {
        char msg[220];
        (void)snprintf(msg, sizeof(msg),
                       "A4 tx target_deg=%.2f target_raw_0p01deg=%ld maxSpeed=%u frame=%02X %02X %02X %02X %02X %02X %02X %02X",
                       ((double)angle_0p01deg) / 100.0,
                       (long)angle_0p01deg,
                       (unsigned)max_speed_dps,
                       tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7]);
        ctx->log_fn(MYACT_LOG_DEBUG, msg, ctx->log_user);
    }
    return myact_send_frame(ctx, ctx->cmd_id, tx);
}

int myact_position_abs_deg_send(myact_ctx_t* ctx, double target_deg, uint16_t max_speed_dps) {
    double raw_f;
    if (!ctx) return MYACT_EINVAL;
    if (!isfinite(target_deg)) return MYACT_ERANGE;
    raw_f = round(target_deg * 100.0);
    if (raw_f < (double)INT32_MIN || raw_f > (double)INT32_MAX) return MYACT_ERANGE;
    return myact_position_abs_send(ctx, max_speed_dps, (int32_t)raw_f);
}

int myact_position_single_send(myact_ctx_t* ctx, uint8_t spin_direction, uint16_t max_speed_dps, uint16_t angle_0p01deg) {
    uint8_t tx[8] = { MYACT_CMD_POSITION_SINGLE, 0,0,0,0,0,0,0 };
    if (!ctx) return MYACT_EINVAL;
    if (!(spin_direction == 0u || spin_direction == 1u)) return MYACT_EINVAL;
    tx[1] = spin_direction;
    wr_u16_le(&tx[2], max_speed_dps);
    wr_u16_le(&tx[4], angle_0p01deg);
    return myact_send_frame(ctx, ctx->cmd_id, tx);
}

int myact_position_inc_send(myact_ctx_t* ctx, uint16_t max_speed_dps, int32_t delta_angle_0p01deg) {
    uint8_t tx[8] = { MYACT_CMD_POSITION_INC, 0,0,0,0,0,0,0 };
    if (!ctx) return MYACT_EINVAL;
    wr_u16_le(&tx[2], max_speed_dps);
    wr_u32_le(&tx[4], (uint32_t)delta_angle_0p01deg);
    return myact_send_frame(ctx, ctx->cmd_id, tx);
}

/* synchronous control */

int myact_shutdown(myact_ctx_t* ctx) {
    uint8_t tx[8] = { MYACT_CMD_SHUTDOWN, 0,0,0,0,0,0,0 };
    return myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_SHUTDOWN, NULL);
}

int myact_stop(myact_ctx_t* ctx) {
    uint8_t tx[8] = { MYACT_CMD_STOP, 0,0,0,0,0,0,0 };
    return myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_STOP, NULL);
}

int myact_torque(myact_ctx_t* ctx, int16_t iq_0p01a, myact_basic_feedback_t* out) {
    uint8_t tx[8] = { MYACT_CMD_TORQUE, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    myact_basic_feedback_t tmp;
    int rc;
    if (!ctx) return MYACT_EINVAL;
    if (!out) out = &tmp;
    wr_u16_le(&tx[4], (uint16_t)iq_0p01a);
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_TORQUE, &rx);
    if (rc < 0) return rc;
    return parse_basic_feedback(&rx, myact_now_ms(ctx), out);
}

int myact_speed(myact_ctx_t* ctx, int32_t speed_0p01dps, myact_basic_feedback_t* out) {
    uint8_t tx[8] = { MYACT_CMD_SPEED, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    myact_basic_feedback_t tmp;
    int rc;
    if (!ctx) return MYACT_EINVAL;
    if (!out) out = &tmp;
    wr_u32_le(&tx[4], (uint32_t)speed_0p01dps);
    if (ctx->log_fn) {
        char msg[180];
        (void)snprintf(msg, sizeof(msg),
                       "A2 txrx speed_dps=%.2f raw_0p01dps=%ld frame=%02X %02X %02X %02X %02X %02X %02X %02X",
                       ((double)speed_0p01dps) / 100.0,
                       (long)speed_0p01dps,
                       tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7]);
        ctx->log_fn(MYACT_LOG_DEBUG, msg, ctx->log_user);
    }
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_SPEED, &rx);
    if (rc < 0) return rc;
    return parse_basic_feedback(&rx, myact_now_ms(ctx), out);
}

int myact_position_abs(myact_ctx_t* ctx, uint16_t max_speed_dps, int32_t angle_0p01deg, myact_basic_feedback_t* out) {
    uint8_t tx[8] = { MYACT_CMD_POSITION_ABS, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    myact_basic_feedback_t tmp;
    int rc;
    if (!ctx) return MYACT_EINVAL;
    if (!out) out = &tmp;
    wr_u16_le(&tx[2], max_speed_dps);
    wr_u32_le(&tx[4], (uint32_t)angle_0p01deg);
    if (ctx->log_fn) {
        char msg[220];
        (void)snprintf(msg, sizeof(msg),
                       "A4 txrx target_deg=%.2f target_raw_0p01deg=%ld maxSpeed=%u frame=%02X %02X %02X %02X %02X %02X %02X %02X",
                       ((double)angle_0p01deg) / 100.0,
                       (long)angle_0p01deg,
                       (unsigned)max_speed_dps,
                       tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7]);
        ctx->log_fn(MYACT_LOG_DEBUG, msg, ctx->log_user);
    }
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_POSITION_ABS, &rx);
    if (rc < 0) return rc;
    return parse_basic_feedback(&rx, myact_now_ms(ctx), out);
}

int myact_position_single(myact_ctx_t* ctx, uint8_t spin_direction, uint16_t max_speed_dps, uint16_t angle_0p01deg, myact_singleturn_feedback_t* out) {
    uint8_t tx[8] = { MYACT_CMD_POSITION_SINGLE, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out) return MYACT_EINVAL;
    if (!(spin_direction == 0u || spin_direction == 1u)) return MYACT_EINVAL;
    tx[1] = spin_direction;
    wr_u16_le(&tx[2], max_speed_dps);
    wr_u16_le(&tx[4], angle_0p01deg);
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_POSITION_SINGLE, &rx);
    if (rc < 0) return rc;
    return parse_singleturn_feedback(&rx, myact_now_ms(ctx), out);
}

int myact_position_inc(myact_ctx_t* ctx, uint16_t max_speed_dps, int32_t delta_angle_0p01deg, myact_basic_feedback_t* out) {
    uint8_t tx[8] = { MYACT_CMD_POSITION_INC, 0,0,0,0,0,0,0 };
    myact_can_frame_t rx;
    myact_basic_feedback_t tmp;
    int rc;
    if (!ctx) return MYACT_EINVAL;
    if (!out) out = &tmp;
    wr_u16_le(&tx[2], max_speed_dps);
    wr_u32_le(&tx[4], (uint32_t)delta_angle_0p01deg);
    rc = myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_POSITION_INC, &rx);
    if (rc < 0) return rc;
    return parse_basic_feedback(&rx, myact_now_ms(ctx), out);
}

/* system / services */

int myact_system_reset(myact_ctx_t* ctx) {
    uint8_t tx[8] = { MYACT_CMD_SYSTEM_RESET, 0,0,0,0,0,0,0 };
    if (!ctx) return MYACT_EINVAL;
    return myact_send_frame(ctx, ctx->cmd_id, tx);
}

int myact_brake_release(myact_ctx_t* ctx) {
    uint8_t tx[8] = { MYACT_CMD_BRAKE_RELEASE, 0,0,0,0,0,0,0 };
    return myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_BRAKE_RELEASE, NULL);
}

int myact_fault_clear(myact_ctx_t* ctx) {
    return myact_shutdown(ctx);
}

int myact_brake_lock(myact_ctx_t* ctx) {
    uint8_t tx[8] = { MYACT_CMD_BRAKE_LOCK, 0,0,0,0,0,0,0 };
    return myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_BRAKE_LOCK, NULL);
}

int myact_set_comm_protect_ms(myact_ctx_t* ctx, uint32_t timeout_ms) {
    uint8_t tx[8] = { MYACT_CMD_COMM_PROTECT, 0,0,0,0,0,0,0 };
    if (!ctx) return MYACT_EINVAL;
    wr_u32_le(&tx[4], timeout_ms);
    return myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_COMM_PROTECT, NULL);
}

int myact_set_baud_code(myact_ctx_t* ctx, myact_baud_code_t code) {
    uint8_t tx[8] = { MYACT_CMD_SET_BAUD, 0,0,0,0,0,0,0 };
    if (!ctx) return MYACT_EINVAL;
    tx[7] = (uint8_t)code;
    /* Protocol says reply is random / need not be processed */
    return myact_send_frame(ctx, ctx->cmd_id, tx);
}

int myact_active_reply_config(myact_ctx_t* ctx, uint8_t reply_cmd, bool enable, uint16_t interval_10ms) {
    uint8_t tx[8] = {0};
    static const uint8_t allowed[] = { 0x60, 0x61, 0x62, 0x92, 0x9A, 0x9C, 0x9D, 0x9E };
    size_t i;
    bool supported = false;
    if (!ctx) return MYACT_EINVAL;
    for (i = 0; i < sizeof(allowed) / sizeof(allowed[0]); ++i) {
        if (allowed[i] == reply_cmd) { supported = true; break; }
    }
    if (!supported) return MYACT_ERANGE;
    tx[0] = MYACT_CMD_ACTIVE_REPLY;
    tx[1] = reply_cmd;
    tx[2] = enable ? 1u : 0u;
    if (enable) {
        tx[3] = (uint8_t)(interval_10ms & 0xFFu);
        tx[4] = (uint8_t)((interval_10ms >> 8) & 0xFFu);
    } else {
        tx[3] = 0u;
        tx[4] = 0u;
    }
    return myact_send_frame(ctx, ctx->cmd_id, tx);
}

int myact_active_reply_disable_all(myact_ctx_t* ctx) {
    static const uint8_t cmds[] = { 0x60, 0x61, 0x62, 0x92, 0x9A, 0x9C, 0x9D, 0x9E };
    int rc_final = MYACT_OK;
    size_t i;
    if (!ctx) return MYACT_EINVAL;
    for (i = 0; i < sizeof(cmds) / sizeof(cmds[0]); ++i) {
        int rc = myact_active_reply_config(ctx, cmds[i], false, 0);
        if (rc < 0 && rc_final == MYACT_OK) rc_final = rc;
        if (i + 1u < (sizeof(cmds) / sizeof(cmds[0]))) {
            (void)ctx->io.recv(&(myact_can_frame_t){0}, MYACT_ACTIVE_REPLY_DISABLE_SPACING_MS, ctx->io.user);
        }
    }
    (void)myact_drain_rx(ctx, MYACT_ACTIVE_REPLY_DRAIN_MS);
    return rc_final;
}

int myact_disable_all_active_replies(myact_ctx_t* ctx) {
    return myact_active_reply_disable_all(ctx);
}

int myact_drain_rx(myact_ctx_t* ctx, uint32_t duration_ms) {
    uint64_t start;
    if (!ctx) return MYACT_EINVAL;
    start = ctx->io.time_ms(ctx->io.user);
    while (ctx->io.time_ms(ctx->io.user) - start < duration_ms) {
        myact_can_frame_t fr;
        int rc = ctx->io.recv(&fr, 10, ctx->io.user);
        if (rc == MYACT_ETIMEDOUT) continue;
        if (rc < 0) return rc;
        (void)myact_parse_frame(ctx, &fr);
    }
    return MYACT_OK;
}

int myact_wait_quiet(myact_ctx_t* ctx, uint32_t quiet_ms, uint32_t max_wait_ms) {
    uint64_t start;
    uint64_t last_rx;
    if (!ctx) return MYACT_EINVAL;
    start = ctx->io.time_ms(ctx->io.user);
    last_rx = start;
    while (ctx->io.time_ms(ctx->io.user) - start < max_wait_ms) {
        myact_can_frame_t fr;
        int rc = ctx->io.recv(&fr, quiet_ms, ctx->io.user);
        uint64_t now = ctx->io.time_ms(ctx->io.user);
        if (rc == MYACT_ETIMEDOUT) {
            if (now - last_rx >= quiet_ms) return MYACT_OK;
            continue;
        }
        if (rc < 0) return rc;
        last_rx = now;
        (void)myact_parse_frame(ctx, &fr);
    }
    return MYACT_ESTATE;
}

int myact_function_control(myact_ctx_t* ctx, myact_function_index_t index, int32_t value) {
    uint8_t tx[8] = { MYACT_CMD_FUNCTION_CONTROL, 0,0,0,0,0,0,0 };
    if (!ctx) return MYACT_EINVAL;
    tx[1] = (uint8_t)index;
    wr_u32_le(&tx[4], (uint32_t)value);
    return myact_txrx(ctx, ctx->cmd_id, tx, ctx->reply_id, MYACT_CMD_FUNCTION_CONTROL, NULL);
}

int myact_clear_multi_turn(myact_ctx_t* ctx) {
    return myact_function_control(ctx, MYACT_FUNC_CLEAR_MULTI_TURN, 0);
}
int myact_set_canid_filter(myact_ctx_t* ctx, bool enable) {
    return myact_function_control(ctx, MYACT_FUNC_CANID_FILTER_ENABLE, enable ? 1 : 0);
}
int myact_set_error_status_broadcast(myact_ctx_t* ctx, bool enable) {
    return myact_function_control(ctx, MYACT_FUNC_ERROR_TX_ENABLE, enable ? 1 : 0);
}
int myact_set_multi_turn_save_on_poweroff(myact_ctx_t* ctx, bool enable) {
    return myact_function_control(ctx, MYACT_FUNC_SAVE_MULTI_TURN_ON_POWEROFF, enable ? 1 : 0);
}
int myact_set_canid_via_function(myact_ctx_t* ctx, uint8_t new_node_id) {
    return myact_function_control(ctx, MYACT_FUNC_SET_CANID, new_node_id);
}
int myact_set_positive_limit_raw(myact_ctx_t* ctx, int32_t raw_value) {
    return myact_function_control(ctx, MYACT_FUNC_SET_POSITIVE_LIMIT, raw_value);
}
int myact_set_negative_limit_raw(myact_ctx_t* ctx, int32_t raw_value) {
    return myact_function_control(ctx, MYACT_FUNC_SET_NEGATIVE_LIMIT, raw_value);
}
int myact_set_positive_limit_deg(myact_ctx_t* ctx, double deg_value) {
    return myact_set_positive_limit_raw(ctx, myact_deg_to_raw_0p01(deg_value));
}
int myact_set_negative_limit_deg(myact_ctx_t* ctx, double deg_value) {
    return myact_set_negative_limit_raw(ctx, myact_deg_to_raw_0p01(deg_value));
}

/* dedicated CAN-ID service */

int myact_canid_read(myact_ctx_t* ctx, uint8_t* out_node_id) {
    uint8_t tx[8] = { MYACT_CMD_CANID, 0x00, 0x01, 0,0,0,0,0 };
    myact_can_frame_t rx;
    uint16_t reply_id;
    int rc;

    if (!ctx || !out_node_id) return MYACT_EINVAL;
    rc = myact_txrx(ctx, MYACT_CANID_SERVICE_ID, tx, MYACT_CANID_SERVICE_ID, MYACT_CMD_CANID, &rx);
    if (rc < 0) return rc;

    reply_id = rd_u16_le(&rx.data[6]);
    if (reply_id < (MYACT_REPLY_ID_BASE + MYACT_NODE_ID_MIN) ||
        reply_id > (MYACT_REPLY_ID_BASE + MYACT_NODE_ID_MAX)) {
        return MYACT_EPROTO;
    }

    *out_node_id = (uint8_t)(reply_id - MYACT_REPLY_ID_BASE);
    ctx->cache.current_can_id = *out_node_id;
    ctx->cache.canid_valid = true;
    ctx->cache.canid_ts_ms = myact_now_ms(ctx);
    return MYACT_OK;
}

int myact_canid_write(myact_ctx_t* ctx, uint8_t new_node_id) {
    uint8_t tx[8] = { MYACT_CMD_CANID, 0x00, 0x00, 0,0,0,0,0 };
    int rc;
    if (!ctx) return MYACT_EINVAL;
    if (new_node_id < MYACT_NODE_ID_MIN || new_node_id > MYACT_NODE_ID_MAX) return MYACT_EINVAL;
    tx[7] = new_node_id;
    rc = myact_txrx(ctx, MYACT_CANID_SERVICE_ID, tx, MYACT_CANID_SERVICE_ID, MYACT_CMD_CANID, NULL);
    if (rc < 0) return rc;
    myact_set_node_id(ctx, new_node_id);
    ctx->cache.current_can_id = new_node_id;
    ctx->cache.canid_valid = true;
    ctx->cache.canid_ts_ms = myact_now_ms(ctx);
    return MYACT_OK;
}


/* motion mode */

int myact_motion_send(myact_ctx_t* ctx, float p_rad, float v_rad_s, float kp, float kd, float t_ff_nm) {
    uint8_t tx[8];
    int rc;
    if (!ctx) return MYACT_EINVAL;
    rc = myact_motion_pack(p_rad, v_rad_s, kp, kd, t_ff_nm, tx);
    if (rc < 0) return rc;
    return myact_send_frame(ctx, ctx->motion_cmd_id, tx);
}

int myact_motion_command(myact_ctx_t* ctx, float p_rad, float v_rad_s, float kp, float kd, float t_ff_nm, myact_motion_feedback_t* out) {
    uint8_t tx[8];
    myact_can_frame_t rx;
    int rc;
    if (!ctx || !out) return MYACT_EINVAL;
    rc = myact_motion_pack(p_rad, v_rad_s, kp, kd, t_ff_nm, tx);
    if (rc < 0) return rc;
    rc = myact_txrx(ctx, ctx->motion_cmd_id, tx, ctx->motion_reply_id, -1, &rx);
    if (rc < 0) return rc;
    rc = myact_motion_unpack_reply(&rx, out);
    if (rc == MYACT_OK) out->ts_ms = myact_now_ms(ctx);
    return rc;
}

int myact_timed_txrx(myact_ctx_t* ctx, uint8_t cmd, const uint8_t tx_data[8], myact_can_frame_t* out_reply, uint32_t* out_latency_us) {
    uint8_t data[8] = {0};
    uint64_t t0;
    uint64_t t1;
    int rc;
    if (!ctx || !tx_data || !out_reply) return MYACT_EINVAL;
    memcpy(data, tx_data, 8);
    data[0] = cmd;
    t0 = myact_now_ms(ctx);
    rc = myact_send_frame(ctx, ctx->cmd_id, data);
    if (rc < 0) return rc;
    rc = myact_recv_frame(ctx, ctx->timeout_ms, out_reply);
    if (rc < 0) return rc;
    t1 = myact_now_ms(ctx);
    if (out_latency_us) *out_latency_us = (uint32_t)((t1 - t0) * 1000u);
    return MYACT_OK;
}

/* unit helpers */

int16_t myact_current_a_to_raw(double amps) {
    long v = lround(amps * 100.0);
    if (v > 32767L) v = 32767L;
    if (v < -32768L) v = -32768L;
    return (int16_t)v;
}

double myact_current_raw_to_a(int16_t raw) {
    return ((double)raw) / 100.0;
}

int32_t myact_speed_dps_to_raw(double dps) {
    double s = dps * 100.0;
    if (s > 2147483647.0) s = 2147483647.0;
    if (s < -2147483648.0) s = -2147483648.0;
    return (int32_t)llround(s);
}

double myact_speed_raw_to_dps(int32_t raw_0p01dps) {
    return ((double)raw_0p01dps) / 100.0;
}

int32_t myact_deg_to_raw_0p01(double deg) {
    double s = deg * 100.0;
    if (s > 2147483647.0) s = 2147483647.0;
    if (s < -2147483648.0) s = -2147483648.0;
    return (int32_t)llround(s);
}

double myact_raw_0p01_to_deg(int32_t raw_0p01deg) {
    return ((double)raw_0p01deg) / 100.0;
}

double myact_rpm_to_dps(double rpm) {
    return rpm * 6.0;
}

double myact_dps_to_rpm(double dps) {
    return dps / 6.0;
}

double myact_output_deg_to_motor_deg(double output_deg, double gear_ratio) {
    return output_deg * gear_ratio;
}

double myact_motor_deg_to_output_deg(double motor_deg, double gear_ratio) {
    return motor_deg / gear_ratio;
}
