#ifndef X8_25_H
#define X8_25_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * MYACTUATOR V3 CAN low-level library.
 *
 * Design:
 * - single-threaded, no heap allocation, no internal threads
 * - transport-agnostic: application supplies CAN send/recv/time callbacks
 * - real-time safe send-only functions for torque/speed/position/motion mode
 * - bounded request/reply helpers for setup and diagnostics
 */

#define MYACT_NODE_ID_MIN              1u
#define MYACT_NODE_ID_MAX             32u
#define MYACT_DEFAULT_TIMEOUT_MS     100u
#define MYACT_ACTIVE_REPLY_DISABLE_SPACING_MS 6u
#define MYACT_ACTIVE_REPLY_DRAIN_MS          400u

/* X8-25 product constants (output-shaft side) */
#define MYACT_X8_25_GEAR_RATIO              9.0
#define MYACT_X8_25_RATED_VOLTAGE_V        48.0
#define MYACT_X8_25_RATED_SPEED_RPM       110.0
#define MYACT_X8_25_RATED_SPEED_DPS      (MYACT_X8_25_RATED_SPEED_RPM * 6.0)
#define MYACT_X8_25_RATED_TORQUE_NM       10.0
#define MYACT_X8_25_PEAK_TORQUE_NM        25.0
#define MYACT_X8_25_RATED_CURRENT_A        3.2
#define MYACT_X8_25_PEAK_CURRENT_A         8.0
#define MYACT_X8_25_ENCODER_BITS          18u


/* MIT motion packing ranges (X8-25 firmware format) */
#define MYACT_MOTION_P_MIN_RAD      (-12.5f)
#define MYACT_MOTION_P_MAX_RAD      (12.5f)
#define MYACT_MOTION_V_MIN_RAD_S    (-45.0f)
#define MYACT_MOTION_V_MAX_RAD_S    (45.0f)
#define MYACT_MOTION_KP_MIN         (0.0f)
#define MYACT_MOTION_KP_MAX         (500.0f)
#define MYACT_MOTION_KD_MIN         (0.0f)
#define MYACT_MOTION_KD_MAX         (5.0f)
#define MYACT_MOTION_T_MIN_NM       (-18.0f)
#define MYACT_MOTION_T_MAX_NM       (18.0f)

typedef enum {
    MYACT_OK          =  0,
    MYACT_EINVAL      = -1,
    MYACT_EIO         = -2,
    MYACT_ETIMEDOUT   = -3,
    MYACT_EPROTO      = -4,
    MYACT_ERANGE      = -5,
    MYACT_ESTATE      = -6,
    MYACT_ENOTSUP     = -7,
    MYACT_EBUS        = -8,
    MYACT_ENOMOTION   = -9,
    MYACT_EBRAKE      = -10
} myact_rc_t;

const char* myact_strerror(int rc);

typedef enum {
    MYACT_LOG_ERROR = 0,
    MYACT_LOG_WARN  = 1,
    MYACT_LOG_INFO  = 2,
    MYACT_LOG_DEBUG = 3
} myact_log_level_t;

typedef void (*myact_log_fn)(myact_log_level_t level, const char* msg, void* user);

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} myact_can_frame_t;

/*
 * Transport callback contract:
 *   send(frame, user) -> 0 on success, negative MYACT_* on failure
 *   recv(frame, timeout_ms, user) -> 0 on success, MYACT_ETIMEDOUT on timeout,
 *                                    or another negative MYACT_* code
 *   time_ms(user) -> monotonic milliseconds
 */
typedef int      (*myact_send_fn)(const myact_can_frame_t* frame, void* user);
typedef int      (*myact_recv_fn)(myact_can_frame_t* frame, uint32_t timeout_ms, void* user);
typedef uint64_t (*myact_time_ms_fn)(void* user);

typedef struct {
    myact_send_fn    send;
    myact_recv_fn    recv;
    myact_time_ms_fn time_ms;
    void*            user;
} myact_io_t;

typedef enum {
    MYACT_CMD_READ_PID            = 0x30,
    MYACT_CMD_WRITE_PID_RAM       = 0x31,
    MYACT_CMD_WRITE_PID_ROM       = 0x32,
    MYACT_CMD_READ_ACCEL          = 0x42,
    MYACT_CMD_WRITE_ACCEL_ROM     = 0x43,
    MYACT_CMD_READ_ENCODER_MULTI  = 0x60,
    MYACT_CMD_READ_ENCODER_RAW    = 0x61,
    MYACT_CMD_READ_ENCODER_OFFSET = 0x62,
    MYACT_CMD_WRITE_ZERO_OFFSET   = 0x63,
    MYACT_CMD_WRITE_ZERO_FROM_NOW = 0x64,
    MYACT_CMD_GET_RUNMODE         = 0x70,
    MYACT_CMD_SYSTEM_RESET        = 0x76,
    MYACT_CMD_BRAKE_RELEASE       = 0x77,
    MYACT_CMD_BRAKE_LOCK          = 0x78,
    MYACT_CMD_CANID               = 0x79,
    MYACT_CMD_SHUTDOWN            = 0x80,
    MYACT_CMD_STOP                = 0x81,
    MYACT_CMD_READ_ENCODER_SINGLE = 0x90,
    MYACT_CMD_READ_ANGLE_MULTI    = 0x92,
    MYACT_CMD_READ_ANGLE_SINGLE   = 0x94,
    MYACT_CMD_STATUS1             = 0x9A,
    MYACT_CMD_STATUS2             = 0x9C,
    MYACT_CMD_STATUS3             = 0x9D,
    MYACT_CMD_TORQUE              = 0xA1,
    MYACT_CMD_SPEED               = 0xA2,
    MYACT_CMD_POSITION_ABS        = 0xA4,
    MYACT_CMD_POSITION_SINGLE     = 0xA6,
    MYACT_CMD_POSITION_INC        = 0xA8,
    MYACT_CMD_RUNTIME             = 0xB1,
    MYACT_CMD_VERSION_DATE        = 0xB2,
    MYACT_CMD_COMM_PROTECT        = 0xB3,
    MYACT_CMD_SET_BAUD            = 0xB4,
    MYACT_CMD_MODEL               = 0xB5,
    MYACT_CMD_ACTIVE_REPLY        = 0xB6,
    MYACT_CMD_FUNCTION_CONTROL    = 0x20
} myact_command_t;

typedef enum {
    MYACT_RUNMODE_IDLE_OR_OFF = 0,
    MYACT_RUNMODE_UNKNOWN  = 0,
    MYACT_RUNMODE_CURRENT  = 1,
    MYACT_RUNMODE_SPEED    = 2,
    MYACT_RUNMODE_POSITION = 3
} myact_runmode_t;

typedef enum {
    MYACT_EVENT_NONE            = 0,
    MYACT_EVENT_ENCODER_MULTI   = 1u << 0,
    MYACT_EVENT_ENCODER_RAW     = 1u << 1,
    MYACT_EVENT_ENCODER_OFFSET  = 1u << 2,
    MYACT_EVENT_ENCODER_SINGLE  = 1u << 3,
    MYACT_EVENT_ANGLE_MULTI     = 1u << 4,
    MYACT_EVENT_ANGLE_SINGLE    = 1u << 5,
    MYACT_EVENT_STATUS1         = 1u << 6,
    MYACT_EVENT_STATUS2         = 1u << 7,
    MYACT_EVENT_STATUS3         = 1u << 8,
    MYACT_EVENT_BASIC_FB        = 1u << 9,
    MYACT_EVENT_SINGLETURN_FB   = 1u << 10,
    MYACT_EVENT_OPERATING_MODE  = 1u << 11,
    MYACT_EVENT_RUNTIME         = 1u << 12,
    MYACT_EVENT_VERSION         = 1u << 13,
    MYACT_EVENT_MODEL           = 1u << 14,
    MYACT_EVENT_CANID           = 1u << 15,
    MYACT_EVENT_MOTION_FB       = 1u << 16,
    MYACT_EVENT_RAW_REPLY       = 1u << 17
} myact_event_t;

enum {
    MYACT_ERR_MOTOR_STALL      = 0x0002,
    MYACT_ERR_LOW_VOLTAGE      = 0x0004,
    MYACT_ERR_OVER_VOLTAGE     = 0x0008,
    MYACT_ERR_OVER_CURRENT     = 0x0010,
    MYACT_ERR_POWER_OVERRUN    = 0x0040,
    MYACT_ERR_CALIB_WRITE_FAIL = 0x0080,
    MYACT_ERR_OVERSPEED        = 0x0100,
    MYACT_ERR_MOTOR_OVER_TEMP  = 0x1000,
    MYACT_ERR_ENCODER_CALIB    = 0x2000
};

typedef struct {
    bool     valid;
    uint64_t ts_ms;
    int8_t   motor_temp_c;
    bool     brake_release_command_state;
    uint16_t voltage_0p1v;
    uint16_t error_flags;
} myact_status1_t;

typedef struct {
    bool     valid;
    uint64_t ts_ms;
    int8_t   motor_temp_c;
    int16_t  iq_0p01a;
    int16_t  speed_dps;
    int16_t  angle_deg;
} myact_status2_t;

typedef struct {
    bool     valid;
    uint64_t ts_ms;
    int8_t   motor_temp_c;
    int16_t  phase_a_0p01a;
    int16_t  phase_b_0p01a;
    int16_t  phase_c_0p01a;
} myact_status3_t;

typedef struct {
    bool     valid;
    uint64_t ts_ms;
    uint8_t  source_cmd; /* A1/A2/A4/A8 */
    int8_t   motor_temp_c;
    int16_t  iq_0p01a;
    int16_t  speed_dps;
    int16_t  angle_deg;
} myact_basic_feedback_t;

typedef struct {
    bool     valid;
    uint64_t ts_ms;
    int8_t   motor_temp_c;
    int16_t  iq_0p01a;
    int16_t  speed_dps;
    uint16_t encoder;
} myact_singleturn_feedback_t;

typedef struct {
    bool     valid;
    uint64_t ts_ms;
    uint16_t encoder_pos;
    uint16_t encoder_raw;
    uint16_t encoder_offset;
} myact_singleturn_encoder_t;

typedef struct {
    bool     valid;
    uint64_t ts_ms;
    float    p_rad;
    float    v_rad_s;
    float    t_nm;
} myact_motion_feedback_t;

typedef struct {
    float speed_kp;
    float speed_ki;
    float position_kp;
    float position_ki;
    float position_kd;
    int32_t pos_acc_dps2;
    int32_t pos_dec_dps2;
    int32_t speed_acc_dps2;
    int32_t speed_dec_dps2;
    myact_status1_t status1;
    myact_runmode_t runmode;
} myact_motion_commissioning_snapshot_t;

typedef struct {
    myact_status1_t             status1;
    myact_status2_t             status2;
    myact_status3_t             status3;
    myact_basic_feedback_t      basic_feedback;
    myact_singleturn_feedback_t singleturn_feedback;
    myact_singleturn_encoder_t  singleturn_encoder;
    myact_motion_feedback_t     motion_feedback;

    bool     encoder_multi_valid;
    bool     encoder_raw_valid;
    bool     encoder_offset_valid;
    bool     angle_multi_valid;
    bool     angle_single_valid;
    bool     operating_mode_valid;
    bool     runtime_valid;
    bool     version_valid;
    bool     model_valid;
    bool     canid_valid;

    uint64_t encoder_multi_ts_ms;
    uint64_t encoder_raw_ts_ms;
    uint64_t encoder_offset_ts_ms;
    uint64_t angle_multi_ts_ms;
    uint64_t angle_single_ts_ms;
    uint64_t operating_mode_ts_ms;
    uint64_t runtime_ts_ms;
    uint64_t version_ts_ms;
    uint64_t model_ts_ms;
    uint64_t canid_ts_ms;

    int32_t  encoder_multi;
    int32_t  encoder_raw;
    int32_t  encoder_offset;
    int32_t  angle_multi_0p01deg;
    uint16_t angle_single_0p01deg;
    myact_runmode_t operating_mode;
    uint32_t runtime_ms;
    uint32_t version_date_yyyymmdd;
    char     model_ascii7[8];
    uint8_t  current_can_id;

    myact_can_frame_t last_raw_reply;
} myact_cache_t;

typedef struct {
    myact_io_t    io;
    uint8_t       node_id;
    uint32_t      timeout_ms;

    uint32_t      cmd_id;          /* 0x140 + node */
    uint32_t      reply_id;        /* 0x240 + node */
    uint32_t      motion_cmd_id;   /* 0x400 + node */
    uint32_t      motion_reply_id; /* 0x500 + node */

    myact_log_fn  log_fn;
    void*         log_user;
    myact_cache_t cache;
} myact_ctx_t;

/* lifecycle / context / config */
int  myact_init(myact_ctx_t* ctx, uint8_t node_id, const myact_io_t* io);
void myact_reset_cache(myact_ctx_t* ctx);
void myact_set_timeout_ms(myact_ctx_t* ctx, uint32_t timeout_ms);
void myact_set_logger(myact_ctx_t* ctx, myact_log_fn log_fn, void* user);
void myact_set_node_id(myact_ctx_t* ctx, uint8_t node_id);
const myact_cache_t* myact_cache(const myact_ctx_t* ctx);

/* service / poll helpers */
int myact_poll(myact_ctx_t* ctx, uint32_t timeout_ms);
int myact_send_raw_single(myact_ctx_t* ctx, const uint8_t data[8]);
int myact_send_raw_motion(myact_ctx_t* ctx, const uint8_t data[8]);
int myact_drain_rx(myact_ctx_t* ctx, uint32_t duration_ms);
int myact_wait_quiet(myact_ctx_t* ctx, uint32_t quiet_ms, uint32_t max_wait_ms);
int myact_timed_txrx(myact_ctx_t* ctx, uint8_t cmd, const uint8_t tx_data[8], myact_can_frame_t* out_reply, uint32_t* out_latency_us);

/* stop / shutdown / brake */
int myact_shutdown_send(myact_ctx_t* ctx);
int myact_stop_send(myact_ctx_t* ctx);
int myact_stop_and_shutdown(myact_ctx_t* ctx, uint32_t settle_ms);
int myact_shutdown(myact_ctx_t* ctx);
int myact_stop(myact_ctx_t* ctx);
int myact_brake_release(myact_ctx_t* ctx);
int myact_fault_clear(myact_ctx_t* ctx);
int myact_prepare_for_motion(myact_ctx_t* ctx,
                             uint32_t brake_timeout_ms,
                             uint32_t poll_ms);

/* read identity / status / basic feedback / angle */
int myact_read_status1(myact_ctx_t* ctx, myact_status1_t* out);
int myact_read_status2(myact_ctx_t* ctx, myact_status2_t* out);
int myact_read_status3(myact_ctx_t* ctx, myact_status3_t* out);
int myact_read_angle_multi(myact_ctx_t* ctx, int32_t* out_angle_0p01deg);
int myact_read_angle_single(myact_ctx_t* ctx, uint16_t* out_angle_0p01deg);
int myact_read_runtime(myact_ctx_t* ctx, uint32_t* out_runtime_ms);
int myact_read_version_date(myact_ctx_t* ctx, uint32_t* out_yyyymmdd);
int myact_read_model(myact_ctx_t* ctx, char out_ascii7[8]);
int myact_read_motion_commissioning_snapshot(myact_ctx_t* ctx,
                                             myact_motion_commissioning_snapshot_t* out);

/* telemetry + runtime moves */
int myact_active_reply_config(myact_ctx_t* ctx, uint8_t reply_cmd, bool enable, uint16_t interval_10ms);
int myact_active_reply_disable_all(myact_ctx_t* ctx);
int myact_disable_all_active_replies(myact_ctx_t* ctx); /* deprecated alias */
int myact_speed_send(myact_ctx_t* ctx, int32_t speed_0p01dps); /* native A2 raw: 0.01 dps/LSB */
int myact_speed_dps_send(myact_ctx_t* ctx, double speed_dps); /* native A2 wrapper: dps -> raw */
int myact_speed(myact_ctx_t* ctx, int32_t speed_0p01dps, myact_basic_feedback_t* out); /* native A2 */
int myact_position_abs_send(myact_ctx_t* ctx, uint16_t max_speed_dps, int32_t angle_0p01deg); /* native A4 */
int myact_position_abs_deg_send(myact_ctx_t* ctx, double target_deg, uint16_t max_speed_dps); /* native A4 wrapper: deg -> raw 0.01deg */
int myact_position_abs(myact_ctx_t* ctx, uint16_t max_speed_dps, int32_t angle_0p01deg, myact_basic_feedback_t* out); /* native A4 */
int myact_position_inc_send(myact_ctx_t* ctx, uint16_t max_speed_dps, int32_t delta_angle_0p01deg); /* A8 */
int myact_position_inc(myact_ctx_t* ctx, uint16_t max_speed_dps, int32_t delta_angle_0p01deg, myact_basic_feedback_t* out); /* A8 */
int myact_motion_send(myact_ctx_t* ctx, float p_rad, float v_rad_s, float kp, float kd, float t_ff_nm); /* MIT */
int myact_motion_command(myact_ctx_t* ctx, float p_rad, float v_rad_s, float kp, float kd, float t_ff_nm, myact_motion_feedback_t* out); /* MIT */

int myact_torque_send(myact_ctx_t* ctx, int16_t iq_0p01a); /* native A1 */
int myact_system_reset(myact_ctx_t* ctx);
int myact_read_operating_mode(myact_ctx_t* ctx, myact_runmode_t* out_mode);
int myact_wait_runmode(myact_ctx_t* ctx,
                       myact_runmode_t expected,
                       uint32_t timeout_ms,
                       uint32_t poll_ms);
int myact_read_encoder_multi(myact_ctx_t* ctx, int32_t* out_encoder);
int myact_read_encoder_raw(myact_ctx_t* ctx, int32_t* out_encoder_raw);
int myact_read_encoder_offset(myact_ctx_t* ctx, int32_t* out_encoder_offset);
int myact_motion_pack(float p_rad, float v_rad_s, float kp, float kd, float t_ff_nm, uint8_t out_data[8]);
int myact_motion_unpack_reply(const myact_can_frame_t* fr, myact_motion_feedback_t* out);

/* Pure frame pack helpers (no CAN I/O). */
int myact_pack_a1_torque(int16_t iq_0p01a, uint8_t out[8]);
int myact_pack_a2_speed(int32_t speed_0p01dps, uint8_t out[8]);
int myact_pack_a4_abs(uint16_t speed_dps, int32_t target_0p01deg, uint8_t out[8]);
int myact_pack_a8_inc(uint16_t speed_dps, int32_t delta_0p01deg, uint8_t out[8]);

/* unit helpers */
int16_t  myact_current_a_to_raw(double amps);
double   myact_current_raw_to_a(int16_t raw);
int32_t  myact_speed_dps_to_raw(double dps);
double   myact_speed_raw_to_dps(int32_t raw_0p01dps);
int32_t  myact_deg_to_raw_0p01(double deg);
double   myact_raw_0p01_to_deg(int32_t raw_0p01deg);
double   myact_rpm_to_dps(double rpm);
double   myact_dps_to_rpm(double dps);
double   myact_output_deg_to_motor_deg(double output_deg, double gear_ratio);
double   myact_motor_deg_to_output_deg(double motor_deg, double gear_ratio);
double   myact_clamp(double x, double x_min, double x_max);

#ifdef __cplusplus
}
#endif

#endif /* X8_25_H */
