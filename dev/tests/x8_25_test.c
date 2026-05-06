#include "x8_25.h"

#include <errno.h>
#include <net/if.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define DEFAULT_MIN_MOTION_VOLTAGE_V 24.0

enum {
    RET_OK = 0,
    RET_COMMS_FAIL = 1,
    RET_BAD_CLI = 2
};

enum {
    MOTION_TORQUE = 1u << 0,
    MOTION_SPEED = 1u << 1,
    MOTION_INC = 1u << 2
};

typedef struct {
    int s;
    char ifname[IFNAMSIZ];
} sockcan_bus_t;
typedef struct {
    bool motion;
    bool motion_torque;
    bool motion_speed;
    bool motion_inc;
    bool motion_all;
    bool no_motion_countdown;
    bool force_motion_brake;
    bool force_motion_low_voltage;
    bool reset_first;
    bool raw_motion_smoke;
    double min_motion_voltage;
} test_opts_t;

static uint64_t now_ms(void* user) {
    struct timespec ts;
    (void)user;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static void msleep(unsigned ms) { usleep(ms * 1000u); }

static int sockcan_send_cb(const myact_can_frame_t* fr, void* user) {
    sockcan_bus_t* b = (sockcan_bus_t*)user;
    struct can_frame c = {0};
    if (!fr || !b || fr->dlc > 8) return MYACT_EINVAL;
    c.can_id = fr->id & CAN_SFF_MASK;
    c.can_dlc = fr->dlc;
    memcpy(c.data, fr->data, 8);
    return write(b->s, &c, sizeof(c)) == (ssize_t)sizeof(c) ? MYACT_OK : MYACT_EIO;
}

static int sockcan_recv_cb(myact_can_frame_t* out, uint32_t to_ms, void* user) {
    sockcan_bus_t* b = (sockcan_bus_t*)user;
    struct pollfd p = {.fd = b->s, .events = POLLIN};
    struct can_frame c;
    int r;
    ssize_t n;
    if (!out || !b) return MYACT_EINVAL;
    r = poll(&p, 1, (int)to_ms);
    if (r == 0) return MYACT_ETIMEDOUT;
    if (r < 0) return MYACT_EIO;
    n = read(b->s, &c, sizeof(c));
    if (n != (ssize_t)sizeof(c)) return MYACT_EIO;
    out->id = c.can_id & CAN_SFF_MASK;
    out->dlc = (uint8_t)c.can_dlc;
    memcpy(out->data, c.data, 8);
    return MYACT_OK;
}

static int sockcan_open(sockcan_bus_t* bus, const char* ifname) {
    struct ifreq ifr;
    struct sockaddr_can addr;
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return -errno;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
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
    bus->s = s;
    return 0;
}

static int prepare_sync(myact_ctx_t* ctx) {
    int rc = myact_disable_all_active_replies(ctx);
    if (rc < 0) return rc;
    rc = myact_drain_rx(ctx, 300);
    if (rc < 0) return rc;
    return myact_wait_quiet(ctx, 300, 1500);
}

static void cleanup_send_only(myact_ctx_t* ctx) {
    (void)myact_disable_all_active_replies(ctx);
    (void)myact_drain_rx(ctx, 500);
    (void)myact_stop_send(ctx);
    msleep(100);
    (void)myact_shutdown_send(ctx);
}

static int parse_cli(int argc, char** argv, test_opts_t* o) {
    int i;
    memset(o, 0, sizeof(*o));
    o->min_motion_voltage = DEFAULT_MIN_MOTION_VOLTAGE_V;
    for (i = 3; i < argc; ++i) {
        if (!strcmp(argv[i], "--motion")) o->motion = true;
        else if (!strcmp(argv[i], "--motion-torque")) o->motion_torque = true;
        else if (!strcmp(argv[i], "--motion-speed")) o->motion_speed = true;
        else if (!strcmp(argv[i], "--motion-inc")) o->motion_inc = true;
        else if (!strcmp(argv[i], "--motion-all")) o->motion_all = true;
        else if (!strcmp(argv[i], "--no-motion-countdown")) o->no_motion_countdown = true;
        else if (!strcmp(argv[i], "--force-motion-brake")) o->force_motion_brake = true;
        else if (!strcmp(argv[i], "--force-motion-low-voltage")) o->force_motion_low_voltage = true;
        else if (!strcmp(argv[i], "--reset-first")) o->reset_first = true;
        else if (!strcmp(argv[i], "--raw-motion-smoke")) o->raw_motion_smoke = true;
        else if (!strcmp(argv[i], "--min-motion-voltage") && i + 1 < argc) o->min_motion_voltage = atof(argv[++i]);
        else return RET_BAD_CLI;
    }
    return RET_OK;
}

static unsigned pick_motion_mask(const test_opts_t* o) {
    unsigned mask = 0;
    if (o->motion_all) return MOTION_TORQUE | MOTION_SPEED | MOTION_INC;
    if (o->motion_torque) mask |= MOTION_TORQUE;
    if (o->motion_speed) mask |= MOTION_SPEED;
    if (o->motion_inc) mask |= MOTION_INC;
    if (mask == 0) mask = MOTION_INC;
    return mask;
}

static int read_full_diagnostics(myact_ctx_t* ctx, myact_status1_t* s1, int32_t* angle_multi_0p01) {
    char model[8] = {0};
    uint32_t version = 0, runtime_ms = 0;
    myact_runmode_t mode = MYACT_RUNMODE_UNKNOWN;
    uint16_t angle_single_0p01 = 0;
    int32_t enc_multi = 0, enc_raw = 0, enc_offset = 0;
    myact_status2_t s2;
    myact_status3_t s3;
    int rc;

    rc = myact_read_model(ctx, model); if (rc < 0) return rc;
    rc = myact_read_version_date(ctx, &version); if (rc < 0) return rc;
    rc = myact_read_runtime(ctx, &runtime_ms); if (rc < 0) return rc;
    rc = myact_read_operating_mode(ctx, &mode); if (rc < 0) return rc;
    rc = myact_read_angle_multi(ctx, angle_multi_0p01); if (rc < 0) return rc;
    rc = myact_read_angle_single(ctx, &angle_single_0p01); if (rc < 0) return rc;
    rc = myact_read_encoder_multi(ctx, &enc_multi); if (rc < 0) return rc;
    rc = myact_read_encoder_raw(ctx, &enc_raw); if (rc < 0) return rc;
    rc = myact_read_encoder_offset(ctx, &enc_offset); if (rc < 0) return rc;
    rc = myact_read_status1(ctx, s1); if (rc < 0) return rc;
    rc = myact_read_status2(ctx, &s2); if (rc < 0) return rc;
    rc = myact_read_status3(ctx, &s3); if (rc < 0) return rc;

    printf("diag model:%s version:%u runtime_ms:%u runmode:%u\n", model, version, runtime_ms, (unsigned)mode);
    printf("diag angle_multi:%.2f angle_single:%.2f enc60:%d enc61:%d enc62:%d\n",
           myact_raw_0p01_to_deg(*angle_multi_0p01), angle_single_0p01 / 100.0, enc_multi, enc_raw, enc_offset);
    printf("diag status1 brake:%s bus:%.1fV err:0x%04X\n",
           s1->brake_release_command_state ? "released" : "locked", s1->voltage_0p1v / 10.0, s1->error_flags);
    return MYACT_OK;
}

static int motion_preflight(myact_ctx_t* ctx, const test_opts_t* o, myact_status1_t* s1) {
    int rc = prepare_sync(ctx);
    if (rc < 0) return rc;

    rc = myact_read_status1(ctx, s1);
    if (rc < 0) return rc;

    if (!s1->brake_release_command_state) {
        (void)myact_brake_release(ctx);
        msleep(300);
        rc = myact_read_status1(ctx, s1);
        if (rc < 0) return rc;
    }

    if (!s1->brake_release_command_state && !o->force_motion_brake) {
        printf("MOTION SKIPPED: brake release command state is still locked/off\n");
        return MYACT_ESTATE;
    }

    if ((s1->voltage_0p1v / 10.0) < o->min_motion_voltage && !o->force_motion_low_voltage) {
        printf("MOTION SKIPPED: bus voltage %.1f V below configured minimum %.1f V\n",
               s1->voltage_0p1v / 10.0, o->min_motion_voltage);
        return MYACT_ESTATE;
    }

    return MYACT_OK;
}

static int run_motion_tests(myact_ctx_t* ctx, unsigned mask) {
    myact_basic_feedback_t fb;
    int32_t angle_after_0p01;
    int rc;

    if (mask & MOTION_TORQUE) {
        rc = myact_torque(ctx, myact_current_a_to_raw(0.20), &fb);
        if (rc < 0) return rc;
        msleep(300);
        (void)myact_stop_send(ctx);
    }

    if (mask & MOTION_SPEED) {
        rc = myact_speed(ctx, myact_speed_dps_to_raw(10.0), &fb);
        if (rc < 0) return rc;
        msleep(700);
        (void)myact_stop_send(ctx);
    }

    if (mask & MOTION_INC) {
        rc = myact_position_inc(ctx, 20, myact_deg_to_raw_0p01(2.0), &fb);
        if (rc < 0) return rc;
        msleep(1200);
        rc = myact_read_angle_multi(ctx, &angle_after_0p01);
        if (rc < 0) return rc;
        printf("post-inc angle_multi: %.2f deg\n", myact_raw_0p01_to_deg(angle_after_0p01));
    }

    return MYACT_OK;
}

int main(int argc, char** argv) {
    sockcan_bus_t bus;
    myact_io_t io;
    myact_ctx_t ctx;
    test_opts_t opts;
    myact_status1_t s1;
    int32_t angle_multi_0p01 = 0;
    unsigned motion_mask;
    int rc;

    if (argc < 3) {
        fprintf(stderr, "usage: %s <can_if> <node_id> [flags...]\n", argv[0]);
        return RET_BAD_CLI;
    }

    rc = parse_cli(argc, argv, &opts);
    if (rc != RET_OK) {
        fprintf(stderr, "invalid option\n");
        return RET_BAD_CLI;
    }

    rc = sockcan_open(&bus, argv[1]);
    if (rc < 0) {
        fprintf(stderr, "open fail\n");
        return RET_COMMS_FAIL;
    }

    io.send = sockcan_send_cb;
    io.recv = sockcan_recv_cb;
    io.time_ms = now_ms;
    io.user = &bus;

    rc = myact_init(&ctx, (uint8_t)atoi(argv[2]), &io);
    if (rc < 0) {
        close(bus.s);
        return RET_COMMS_FAIL;
    }

    myact_set_timeout_ms(&ctx, 300);

    if (opts.reset_first) {
        (void)myact_system_reset(&ctx);
        msleep(2000);
    }

    rc = prepare_sync(&ctx);
    if (rc < 0) goto done;
    rc = read_full_diagnostics(&ctx, &s1, &angle_multi_0p01);
    if (rc < 0) goto done;

    if (!opts.motion && !opts.raw_motion_smoke) {
        printf("DIAG-ONLY mode: no motion requested. Use --motion to run motion tests.\n");
        rc = RET_OK;
        goto done;
    }

    if (opts.raw_motion_smoke) {
        int32_t before = angle_multi_0p01;
        int32_t after = 0;
        rc = motion_preflight(&ctx, &opts, &s1);
        if (rc < 0) { rc = RET_OK; goto done; }
        rc = myact_position_inc(&ctx, 10, myact_deg_to_raw_0p01(1.0), NULL);
        if (rc < 0) goto done;
        msleep(1000);
        rc = myact_read_angle_multi(&ctx, &after);
        if (rc < 0) goto done;
        printf("raw-motion-smoke delta: %.2f deg\n", myact_raw_0p01_to_deg(after - before));
        rc = RET_OK;
        goto done;
    }

    motion_mask = pick_motion_mask(&opts);
    printf("motion plan: angle_multi=%.2f deg bus=%.1fV brake=%s test=0x%X ",
           myact_raw_0p01_to_deg(angle_multi_0p01), s1.voltage_0p1v / 10.0,
           s1.brake_release_command_state ? "released" : "locked", motion_mask);
    printf("targets: torque:+0.20A(300ms) speed:+10dps(700ms) inc:+2.00deg@20dps(wait1200ms)\n");

    if (!opts.no_motion_countdown) msleep(2000);

    rc = motion_preflight(&ctx, &opts, &s1);
    if (rc < 0) { rc = RET_OK; goto done; }

    rc = run_motion_tests(&ctx, motion_mask);

done:
    cleanup_send_only(&ctx);
    close(bus.s);
    return rc < 0 ? RET_COMMS_FAIL : rc;
}
