/*
 * Build example (Linux, adjust include/lib paths for your PCAN-Basic install):
 *
 *   gcc -O2 -Wall -Wextra -std=c11 \
 *       myactuator_can.c myactuator_can_test_pcan.c \
 *       -I/path/to/pcan/include -L/path/to/pcan/lib \
 *       -lpcanbasic -lm -o myactuator_can_test_pcan
 *
 * Some installs use -lPCANBasic instead of -lpcanbasic.
 *
 * Usage:
 *   ./myactuator_can_test_pcan usb1 1 1M
 *
 * Notes:
 * - This test is intentionally safe by default:
 *   * small torque current
 *   * small speed
 *   * small position steps
 *   * no automatic ROM writes, zero-offset writes, or CAN-ID changes
 * - Brake tests are not executed automatically because the standard X8-25 listed
 *   on the official product page is the "N" (no brake) variant.
 */

#include "myactuator_can.h"
#include <PCANBasic.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

typedef struct {
    TPCANHandle ch;
} pcan_bus_t;

static uint64_t now_ms(void* user) {
    struct timespec ts;
    (void)user;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static void msleep(unsigned ms) {
    usleep(ms * 1000u);
}

static const char* pcan_err_text(TPCANStatus st, char buf[256]) {
    if (CAN_GetErrorText(st, 0, buf) == PCAN_ERROR_OK) return buf;
    snprintf(buf, 256, "PCAN error 0x%08X", (unsigned)st);
    return buf;
}

static int pcan_to_myact(TPCANStatus st) {
    if (st == PCAN_ERROR_OK) return MYACT_OK;
    if (st == PCAN_ERROR_QRCVEMPTY) return MYACT_ETIMEDOUT;
    if (st & (PCAN_ERROR_BUSOFF | PCAN_ERROR_BUSHEAVY | PCAN_ERROR_BUSLIGHT)) return MYACT_EBUS;
    return MYACT_EIO;
}

static int pcan_send_cb(const myact_can_frame_t* fr, void* user) {
    TPCANMsg msg;
    TPCANStatus st;
    pcan_bus_t* bus = (pcan_bus_t*)user;

    if (!fr || !bus) return MYACT_EINVAL;
    memset(&msg, 0, sizeof(msg));
    msg.ID = fr->id;
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    msg.LEN = fr->dlc;
    memcpy(msg.DATA, fr->data, 8);

    st = CAN_Write(bus->ch, &msg);
    return pcan_to_myact(st);
}

static int pcan_recv_cb(myact_can_frame_t* out, uint32_t timeout_ms, void* user) {
    pcan_bus_t* bus = (pcan_bus_t*)user;
    uint64_t t0 = now_ms(NULL);
    TPCANStatus st;

    if (!out || !bus) return MYACT_EINVAL;

    for (;;) {
        TPCANMsg msg;
        TPCANTimestamp ts;

        st = CAN_Read(bus->ch, &msg, &ts);
        if (st == PCAN_ERROR_OK) {
            if (msg.MSGTYPE & PCAN_MESSAGE_STATUS) {
                return MYACT_EBUS;
            }
            if (!(msg.MSGTYPE & PCAN_MESSAGE_STANDARD)) {
                continue;
            }

            memset(out, 0, sizeof(*out));
            out->id = msg.ID;
            out->dlc = msg.LEN;
            memcpy(out->data, msg.DATA, 8);
            return MYACT_OK;
        }

        if (st == PCAN_ERROR_QRCVEMPTY) {
            if ((now_ms(NULL) - t0) >= timeout_ms) return MYACT_ETIMEDOUT;
            usleep(1000);
            continue;
        }

        return pcan_to_myact(st);
    }
}

static void log_cb(myact_log_level_t level, const char* msg, void* user) {
    const char* tag = "LOG";
    (void)user;
    if (level == MYACT_LOG_ERROR) tag = "ERR";
    else if (level == MYACT_LOG_WARN) tag = "WRN";
    else if (level == MYACT_LOG_INFO) tag = "INF";
    else if (level == MYACT_LOG_DEBUG) tag = "DBG";
    fprintf(stderr, "[myact][%s] %s\n", tag, msg);
}

static TPCANHandle parse_channel(const char* s) {
    struct { const char* name; TPCANHandle h; } map[] = {
        { "usb1", PCAN_USBBUS1 }, { "usb2", PCAN_USBBUS2 },
        { "usb3", PCAN_USBBUS3 }, { "usb4", PCAN_USBBUS4 },
        { "usb5", PCAN_USBBUS5 }, { "usb6", PCAN_USBBUS6 },
        { "usb7", PCAN_USBBUS7 }, { "usb8", PCAN_USBBUS8 }
    };
    size_t i;

    if (strncmp(s, "0x", 2) == 0 || strncmp(s, "0X", 2) == 0) {
        return (TPCANHandle)strtoul(s, NULL, 0);
    }

    for (i = 0; i < sizeof(map) / sizeof(map[0]); ++i) {
        if (strcmp(s, map[i].name) == 0) return map[i].h;
    }
    return PCAN_NONEBUS;
}

static TPCANBaudrate parse_baud(const char* s) {
    if (!s) return PCAN_BAUD_1M;
    if (strcmp(s, "1M") == 0 || strcmp(s, "1m") == 0)   return PCAN_BAUD_1M;
    if (strcmp(s, "500K") == 0 || strcmp(s, "500k") == 0) return PCAN_BAUD_500K;
    return PCAN_BAUD_1M;
}

static void print_status1(const myact_status1_t* s) {
    printf("status1: temp=%d C  brake=%s  bus=%.1f V  err=0x%04X\n",
           (int)s->motor_temp_c,
           s->brake_released ? "released" : "locked/off",
           s->voltage_0p1v / 10.0,
           (unsigned)s->error_flags);
}

static void print_status2(const myact_status2_t* s) {
    printf("status2: temp=%d C  iq=%.2f A  speed=%d dps  angle=%d deg\n",
           (int)s->motor_temp_c,
           myact_current_raw_to_a(s->iq_0p01a),
           (int)s->speed_dps,
           (int)s->angle_deg);
}

static void print_status3(const myact_status3_t* s) {
    printf("status3: temp=%d C  ia=%.2f A ib=%.2f A ic=%.2f A\n",
           (int)s->motor_temp_c,
           myact_current_raw_to_a(s->phase_a_0p01a),
           myact_current_raw_to_a(s->phase_b_0p01a),
           myact_current_raw_to_a(s->phase_c_0p01a));
}

static int read_identity(myact_ctx_t* ctx) {
    char model[8];
    uint32_t ver = 0;
    uint32_t runtime = 0;
    myact_runmode_t mode = MYACT_RUNMODE_UNKNOWN;
    int32_t angle_mt = 0;
    uint16_t angle_st = 0;
    int32_t enc_mt = 0;
    int32_t enc_raw = 0;
    int32_t enc_off = 0;
    myact_status1_t s1;
    myact_status2_t s2;
    myact_status3_t s3;
    int rc;

    rc = myact_read_model(ctx, model);
    if (rc < 0) return rc;
    rc = myact_read_version_date(ctx, &ver);
    if (rc < 0) return rc;
    rc = myact_read_runtime(ctx, &runtime);
    if (rc < 0) return rc;
    rc = myact_read_operating_mode(ctx, &mode);
    if (rc < 0) return rc;
    rc = myact_read_angle_multi(ctx, &angle_mt);
    if (rc < 0) return rc;
    rc = myact_read_angle_single(ctx, &angle_st);
    if (rc < 0) return rc;
    rc = myact_read_encoder_multi(ctx, &enc_mt);
    if (rc < 0) return rc;
    rc = myact_read_encoder_raw(ctx, &enc_raw);
    if (rc < 0) return rc;
    rc = myact_read_encoder_offset(ctx, &enc_off);
    if (rc < 0) return rc;
    rc = myact_read_status1(ctx, &s1);
    if (rc < 0) return rc;
    rc = myact_read_status2(ctx, &s2);
    if (rc < 0) return rc;
    rc = myact_read_status3(ctx, &s3);
    if (rc < 0) return rc;

    printf("model: %s\n", model);
    printf("version date: %" PRIu32 "\n", ver);
    printf("runtime: %" PRIu32 " ms\n", runtime);
    printf("runmode: %u\n", (unsigned)mode);
    printf("multi-turn angle: %.2f deg\n", myact_raw_0p01_to_deg(angle_mt));
    printf("single-turn angle: %.2f deg\n", angle_st / 100.0);
    printf("encoder multi: %" PRId32 "\n", enc_mt);
    printf("encoder raw:   %" PRId32 "\n", enc_raw);
    printf("encoder offs:  %" PRId32 "\n", enc_off);
    print_status1(&s1);
    print_status2(&s2);
    print_status3(&s3);
    return MYACT_OK;
}

static int test_active_reply(myact_ctx_t* ctx) {
    int rc;
    int got = 0;

    printf("\n[active-reply test] enabling 0x9C every 100 ms\n");
    rc = myact_active_reply_config(ctx, MYACT_CMD_STATUS2, true, 10);
    if (rc < 0) return rc;

    while (got < 3) {
        int ev = myact_poll(ctx, 300);
        if (ev == MYACT_ETIMEDOUT) continue;
        if (ev < 0) return ev;
        if (ev & MYACT_EVENT_STATUS2) {
            const myact_cache_t* c = myact_cache(ctx);
            printf("  active 0x9C #%d -> iq=%.2f A speed=%d dps angle=%d deg\n",
                   got + 1,
                   myact_current_raw_to_a(c->status2.iq_0p01a),
                   (int)c->status2.speed_dps,
                   (int)c->status2.angle_deg);
            ++got;
        }
    }

    printf("[active-reply test] disabling 0x9C\n");
    return myact_active_reply_config(ctx, MYACT_CMD_STATUS2, false, 10);
}

static int test_torque(myact_ctx_t* ctx) {
    myact_basic_feedback_t fb;
    int rc;
    int16_t iq = myact_current_a_to_raw(0.20);

    printf("\n[torque test] +0.20 A for 300 ms, then stop\n");
    rc = myact_torque(ctx, iq, &fb);
    if (rc < 0) return rc;
    printf("  feedback: iq=%.2f A speed=%d dps angle=%d deg\n",
           myact_current_raw_to_a(fb.iq_0p01a),
           (int)fb.speed_dps,
           (int)fb.angle_deg);
    msleep(300);
    return myact_stop(ctx);
}

static int test_speed(myact_ctx_t* ctx) {
    myact_basic_feedback_t fb;
    int rc;
    int32_t v = myact_speed_dps_to_raw(10.0);

    printf("\n[speed test] +10 dps for 700 ms, then stop\n");
    rc = myact_speed(ctx, v, &fb);
    if (rc < 0) return rc;
    printf("  feedback: iq=%.2f A speed=%d dps angle=%d deg\n",
           myact_current_raw_to_a(fb.iq_0p01a),
           (int)fb.speed_dps,
           (int)fb.angle_deg);
    msleep(700);
    return myact_stop(ctx);
}

static int test_abs_position(myact_ctx_t* ctx) {
    int rc;
    int32_t cur = 0;
    myact_basic_feedback_t fb;

    rc = myact_read_angle_multi(ctx, &cur);
    if (rc < 0) return rc;

    printf("\n[absolute-position test] current=%.2f deg -> target=+5.00 deg offset\n",
           myact_raw_0p01_to_deg(cur));

    rc = myact_position_abs(ctx, 30, cur + myact_deg_to_raw_0p01(5.0), &fb);
    if (rc < 0) return rc;

    printf("  immediate fb: iq=%.2f A speed=%d dps angle=%d deg\n",
           myact_current_raw_to_a(fb.iq_0p01a),
           (int)fb.speed_dps,
           (int)fb.angle_deg);
    msleep(1200);
    rc = myact_read_angle_multi(ctx, &cur);
    if (rc < 0) return rc;

    printf("  after move: %.2f deg\n", myact_raw_0p01_to_deg(cur));
    return MYACT_OK;
}

static int test_inc_position(myact_ctx_t* ctx) {
    myact_basic_feedback_t fb;
    int rc;

    printf("\n[incremental-position test] +2.00 deg @ max 20 dps\n");
    rc = myact_position_inc(ctx, 20, myact_deg_to_raw_0p01(2.0), &fb);
    if (rc < 0) return rc;
    printf("  immediate fb: iq=%.2f A speed=%d dps angle=%d deg\n",
           myact_current_raw_to_a(fb.iq_0p01a),
           (int)fb.speed_dps,
           (int)fb.angle_deg);
    msleep(700);
    return MYACT_OK;
}

static int test_motion_mode(myact_ctx_t* ctx) {
    uint16_t st = 0;
    float p_rad;
    myact_motion_feedback_t fb;
    int rc;

    rc = myact_read_angle_single(ctx, &st);
    if (rc < 0) return rc;

    p_rad = (float)(st / 100.0) * (float)M_PI / 180.0f;

    printf("\n[motion-mode test] holding current single-turn position, kp=5 kd=0.2\n");
    rc = myact_motion_command(ctx, p_rad, 0.0f, 5.0f, 0.2f, 0.0f, &fb);
    if (rc < 0) return rc;

    printf("  motion fb: p=%.3f rad  v=%.3f rad/s  t=%.3f N*m\n",
           fb.p_rad, fb.v_rad_s, fb.t_nm);
    return MYACT_OK;
}

int main(int argc, char** argv) {
    pcan_bus_t bus;
    myact_io_t io;
    myact_ctx_t ctx;
    TPCANHandle ch;
    TPCANBaudrate baud;
    TPCANStatus st;
    char ebuf[256];
    DWORD on = PCAN_PARAMETER_ON;
    int node_id;
    int rc;

    if (argc < 3) {
        fprintf(stderr, "usage: %s <usb1|0x51...> <node_id 1..32> [1M|500K]\n", argv[0]);
        return 2;
    }

    ch = parse_channel(argv[1]);
    node_id = atoi(argv[2]);
    baud = parse_baud((argc >= 4) ? argv[3] : "1M");

    if (ch == PCAN_NONEBUS || node_id < 1 || node_id > 32) {
        fprintf(stderr, "invalid channel or node id\n");
        return 2;
    }

    st = CAN_Initialize(ch, baud, 0, 0, 0);
    if (st != PCAN_ERROR_OK) {
        fprintf(stderr, "CAN_Initialize failed: %s\n", pcan_err_text(st, ebuf));
        return 1;
    }

    (void)CAN_SetValue(ch, PCAN_BUSOFF_AUTORESET, &on, sizeof(on));

    bus.ch = ch;
    io.send = pcan_send_cb;
    io.recv = pcan_recv_cb;
    io.time_ms = now_ms;
    io.user = &bus;

    rc = myact_init(&ctx, (uint8_t)node_id, &io);
    if (rc < 0) {
        fprintf(stderr, "myact_init failed: %s\n", myact_strerror(rc));
        CAN_Uninitialize(ch);
        return 1;
    }

    myact_set_timeout_ms(&ctx, 300);
    myact_set_logger(&ctx, log_cb, NULL);

    printf("=== MYACTUATOR X8-25 PCAN test ===\n");
    printf("channel=%s  node=%d  baud=%s\n", argv[1], node_id, (baud == PCAN_BAUD_500K) ? "500K" : "1M");
    printf("safe defaults: 0.20 A torque, 10 dps speed, small position steps\n");

    rc = read_identity(&ctx);
    if (rc < 0) goto fail;

    rc = test_active_reply(&ctx);
    if (rc < 0) goto fail;

    rc = test_torque(&ctx);
    if (rc < 0) goto fail;

    rc = test_speed(&ctx);
    if (rc < 0) goto fail;

    rc = test_abs_position(&ctx);
    if (rc < 0) goto fail;

    rc = test_inc_position(&ctx);
    if (rc < 0) goto fail;

    rc = test_motion_mode(&ctx);
    if (rc < 0) goto fail;

    (void)myact_stop(&ctx);
    (void)myact_shutdown(&ctx);

    printf("\nall default tests completed successfully\n");
    CAN_Uninitialize(ch);
    return 0;

fail:
    fprintf(stderr, "\nTEST FAILED: %s (%d)\n", myact_strerror(rc), rc);
    (void)myact_stop(&ctx);
    (void)myact_shutdown(&ctx);
    CAN_Uninitialize(ch);
    return 1;
}
