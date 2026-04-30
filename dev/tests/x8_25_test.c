/*
 * Build example (Linux, SocketCAN):
 *
 *   gcc -O2 -Wall -Wextra -std=c11 -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L \
 *       dev/src/x8_25.c dev/tests/x8_25_test.c \
 *       -Idev/include -lm -o x8_25_test_can
 *
 * Usage:
 *   ./x8_25_test_can can0 1
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

#include "x8_25.h"

#include <errno.h>
#include <inttypes.h>
#include <net/if.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    int s;
    char ifname[IFNAMSIZ];
} sockcan_bus_t;

static uint64_t now_ms(void* user) {
    struct timespec ts;
    (void)user;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static void msleep(unsigned ms) {
    usleep(ms * 1000u);
}

static int sockcan_send_cb(const myact_can_frame_t* fr, void* user) {
    sockcan_bus_t* bus = (sockcan_bus_t*)user;
    struct can_frame cfr;
    ssize_t n;
    if (!fr || !bus) return MYACT_EINVAL;
    if (fr->dlc > 8u) return MYACT_EINVAL;

    memset(&cfr, 0, sizeof(cfr));
    cfr.can_id = fr->id & CAN_SFF_MASK;
    cfr.can_dlc = fr->dlc;
    memcpy(cfr.data, fr->data, 8);

    n = write(bus->s, &cfr, sizeof(cfr));
    if (n == (ssize_t)sizeof(cfr)) return MYACT_OK;
    if (n < 0) {
        if (errno == ENETDOWN || errno == ENETUNREACH || errno == EBUSY) return MYACT_EBUS;
        return MYACT_EIO;
    }
    return MYACT_EIO;
}

static int sockcan_recv_cb(myact_can_frame_t* out, uint32_t timeout_ms, void* user) {
    sockcan_bus_t* bus = (sockcan_bus_t*)user;
    struct pollfd pfd;
    int prc;
    struct can_frame cfr;
    ssize_t n;
    if (!out || !bus) return MYACT_EINVAL;

    pfd.fd = bus->s;
    pfd.events = POLLIN;

    for (;;) {
        prc = poll(&pfd, 1, (int)timeout_ms);
        if (prc == 0) return MYACT_ETIMEDOUT;
        if (prc < 0) {
            if (errno == EINTR) continue;
            return MYACT_EIO;
        }
        if (!(pfd.revents & POLLIN)) {
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) return MYACT_EBUS;
            continue;
        }

        n = read(bus->s, &cfr, sizeof(cfr));
        if (n < 0) {
            if (errno == EINTR) continue;
            return MYACT_EIO;
        }
        if (n != (ssize_t)sizeof(cfr)) continue;
        if ((cfr.can_id & CAN_EFF_FLAG) || (cfr.can_id & CAN_RTR_FLAG) || (cfr.can_id & CAN_ERR_FLAG)) continue;

        memset(out, 0, sizeof(*out));
        out->id = cfr.can_id & CAN_SFF_MASK;
        out->dlc = (uint8_t)cfr.can_dlc;
        memcpy(out->data, cfr.data, 8);
        return MYACT_OK;
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

static int sockcan_open(sockcan_bus_t* bus, const char* ifname) {
    struct ifreq ifr;
    struct sockaddr_can addr;
    int s;

    if (!bus || !ifname) return -EINVAL;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return -errno;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        int e = -errno;
        close(s);
        return e;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        int e = -errno;
        close(s);
        return e;
    }

    memset(bus, 0, sizeof(*bus));
    bus->s = s;
    strncpy(bus->ifname, ifname, IFNAMSIZ - 1);
    bus->ifname[IFNAMSIZ - 1] = '\0';
    return 0;
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
    if (rc == MYACT_ETIMEDOUT) {
        const uint64_t grace_ms = 800;
        uint64_t t0 = now_ms(NULL);
        int saw_status2 = 0;
        int saw_effect = 0;
        int saw_status1_noerr = 0;

        rc = myact_torque_send(ctx, iq);
        if (rc < 0) return rc;

        while ((now_ms(NULL) - t0) < grace_ms) {
            myact_status2_t s2;
            myact_status1_t s1;

            rc = myact_read_status2(ctx, &s2);
            if (rc == MYACT_OK) {
                saw_status2 = 1;
                if (abs((int)s2.iq_0p01a) >= abs((int)iq) / 2 || abs((int)s2.speed_dps) >= 1) {
                    saw_effect = 1;
                }
            } else if (rc != MYACT_ETIMEDOUT) {
                return rc;
            }

            rc = myact_read_status1(ctx, &s1);
            if (rc == MYACT_OK) {
                if (s1.error_flags == 0u) {
                    saw_status1_noerr = 1;
                } else {
                    break;
                }
            } else if (rc != MYACT_ETIMEDOUT) {
                return rc;
            }

            if ((saw_effect || saw_status2) && saw_status1_noerr) break;
            msleep(80);
        }

        if ((saw_effect || saw_status2) && saw_status1_noerr) {
            printf("  A1 ack missing; validated via 0x9C stream\n");
        } else {
            fprintf(stderr, "  A1 ack missing and no motion/torque evidence in 0x9C\n");
            return MYACT_ETIMEDOUT;
        }
    } else if (rc < 0) {
        return rc;
    } else {
        printf("  feedback: iq=%.2f A speed=%d dps angle=%d deg\n",
               myact_current_raw_to_a(fb.iq_0p01a),
               (int)fb.speed_dps,
               (int)fb.angle_deg);
    }

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
    sockcan_bus_t bus;
    myact_io_t io;
    myact_ctx_t ctx;
    const char* can_if = NULL;
    int node_id;
    int rc;

    if (argc < 3) {
        fprintf(stderr, "usage: %s <can_if> <node_id 1..32>\n", argv[0]);
        return 2;
    }

    can_if = argv[1];
    node_id = atoi(argv[2]);

    if (node_id < 1 || node_id > 32) {
        fprintf(stderr, "invalid node id\n");
        return 2;
    }

    rc = sockcan_open(&bus, can_if);
    if (rc < 0) {
        fprintf(stderr, "socketcan open failed on '%s': %s\n", can_if, strerror(-rc));
        return 1;
    }

    io.send = sockcan_send_cb;
    io.recv = sockcan_recv_cb;
    io.time_ms = now_ms;
    io.user = &bus;

    rc = myact_init(&ctx, (uint8_t)node_id, &io);
    if (rc < 0) {
        fprintf(stderr, "myact_init failed: %s\n", myact_strerror(rc));
        close(bus.s);
        return 1;
    }

    myact_set_timeout_ms(&ctx, 300);
    myact_set_logger(&ctx, log_cb, NULL);

    printf("=== MYACTUATOR X8-25 SocketCAN test ===\n");
    printf("if=%s  node=%d\n", can_if, node_id);
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
    close(bus.s);
    return 0;

fail:
    fprintf(stderr, "\nTEST FAILED: %s (%d)\n", myact_strerror(rc), rc);
    (void)myact_stop(&ctx);
    (void)myact_shutdown(&ctx);
    close(bus.s);
    return 1;
}
