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
    bool raw_motion_smoke_sendonly;
    bool no_shutdown_on_exit;
    bool enable_hold;
    bool brake_latency_test;
    bool brake_persistence_test;
    bool rt_inc_burst;
    bool speed_step_test;
    bool torque_step_test;
    bool profile_abs_relative_test;
    bool mit_mode_test;
    bool active_telemetry_rt_test;
    bool bench_all;
    bool run_mit_in_bench_all;
    double min_motion_voltage;
    double smoke_delta_deg;
    double smoke_speed_dps;
    unsigned smoke_wait_ms;
    unsigned brake_release_retries;
    unsigned brake_release_wait_ms;
    unsigned brake_release_timeout_ms;
    unsigned brake_poll_ms;
    unsigned cycles;
    double inc_delta_deg;
    double inc_speed_dps;
    unsigned inc_wait_ms;
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

static void test_axis_disable(myact_ctx_t* ctx, bool shutdown) {
    (void)myact_disable_all_active_replies(ctx);
    (void)myact_drain_rx(ctx, 500);
    (void)myact_stop_send(ctx);
    if (shutdown) {
        msleep(100);
        (void)myact_shutdown_send(ctx);
    }
}

static int parse_cli(int argc, char** argv, test_opts_t* o) {
    int i;
    memset(o, 0, sizeof(*o));
    o->min_motion_voltage = DEFAULT_MIN_MOTION_VOLTAGE_V;
    o->brake_release_retries = 3;
    o->brake_release_wait_ms = 1000;
    o->brake_release_timeout_ms = 2000;
    o->brake_poll_ms = 50;
    o->smoke_delta_deg = 10.0;
    o->smoke_speed_dps = 60.0;
    o->smoke_wait_ms = 1500;
    o->cycles = 3;
    o->inc_delta_deg = 10.0;
    o->inc_speed_dps = 60.0;
    o->inc_wait_ms = 1500;
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
        else if (!strcmp(argv[i], "--raw-motion-smoke-sendonly")) o->raw_motion_smoke_sendonly = true;
        else if (!strcmp(argv[i], "--no-shutdown-on-exit")) o->no_shutdown_on_exit = true;
        else if (!strcmp(argv[i], "--enable-hold")) o->enable_hold = true;
        else if (!strcmp(argv[i], "--brake-latency-test")) o->brake_latency_test = true;
        else if (!strcmp(argv[i], "--brake-persistence-test")) o->brake_persistence_test = true;
        else if (!strcmp(argv[i], "--rt-inc-burst")) o->rt_inc_burst = true;
        else if (!strcmp(argv[i], "--speed-step-test")) o->speed_step_test = true;
        else if (!strcmp(argv[i], "--torque-step-test")) o->torque_step_test = true;
        else if (!strcmp(argv[i], "--profile-abs-relative-test")) o->profile_abs_relative_test = true;
        else if (!strcmp(argv[i], "--mit-mode-test")) o->mit_mode_test = true;
        else if (!strcmp(argv[i], "--active-telemetry-rt-test")) o->active_telemetry_rt_test = true;
        else if (!strcmp(argv[i], "--bench-all")) o->bench_all = true;
        else if (!strcmp(argv[i], "--smoke-delta-deg") && i + 1 < argc) o->smoke_delta_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--smoke-speed-dps") && i + 1 < argc) o->smoke_speed_dps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--smoke-wait-ms") && i + 1 < argc) o->smoke_wait_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--brake-release-timeout-ms") && i + 1 < argc) o->brake_release_timeout_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--brake-poll-ms") && i + 1 < argc) o->brake_poll_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--cycles") && i + 1 < argc) o->cycles = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--inc-delta-deg") && i + 1 < argc) o->inc_delta_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--inc-speed-dps") && i + 1 < argc) o->inc_speed_dps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--inc-wait-ms") && i + 1 < argc) o->inc_wait_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--min-motion-voltage") && i + 1 < argc) o->min_motion_voltage = atof(argv[++i]);
        else if (!strcmp(argv[i], "--brake-release-retries") && i + 1 < argc) o->brake_release_retries = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--brake-release-wait-ms") && i + 1 < argc) o->brake_release_wait_ms = (unsigned)atoi(argv[++i]);
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
    unsigned attempt;
    double voltage_v;
    bool brake_released;
    int rc = prepare_sync(ctx);
    if (rc < 0) return rc;

    rc = myact_read_status1(ctx, s1);
    if (rc < 0) return rc;

    printf("brake preflight: before=%s/%s\n",
           s1->brake_release_command_state ? "released" : "locked",
           s1->brake_release_command_state ? "on" : "off");

    for (attempt = 1; attempt <= o->brake_release_retries && !s1->brake_release_command_state; ++attempt) {
        rc = myact_brake_release(ctx);
        if (rc < 0) return rc;
        printf("brake release attempt %u/%u: sent 0x77, reply OK, waiting %u ms\n",
               attempt, o->brake_release_retries, o->brake_release_wait_ms);
        msleep(o->brake_release_wait_ms);
        rc = myact_read_status1(ctx, s1);
        if (rc < 0) return rc;
        printf("brake preflight: after=%s/%s\n",
               s1->brake_release_command_state ? "released" : "locked",
               s1->brake_release_command_state ? "on" : "off");
    }

    brake_released = s1->brake_release_command_state;
    if (!brake_released && !o->force_motion_brake) {
        printf("MOTION SKIPPED: brake release command state is still locked/off after %u attempts\n",
               o->brake_release_retries);
        return MYACT_ESTATE;
    }

    voltage_v = s1->voltage_0p1v / 10.0;
    if (voltage_v < o->min_motion_voltage && !o->force_motion_low_voltage) {
        printf("MOTION SKIPPED: bus voltage %.1f V below configured minimum %.1f V\n",
               voltage_v, o->min_motion_voltage);
        return MYACT_ESTATE;
    }
    if (voltage_v < o->min_motion_voltage && o->force_motion_low_voltage) {
        printf("WARNING: low-voltage motion forced at %.1f V; use rated supply for real torque/speed characterization.\n", voltage_v);
        printf("rated voltage is 48 V-class; current bench voltage is low.\n");
        return MYACT_OK;
    }

    return MYACT_OK;
}

static int test_axis_enable(myact_ctx_t* ctx, const test_opts_t* opts, myact_status1_t* s1) {
    uint64_t t0;
    int rc = prepare_sync(ctx);
    if (rc < 0) return rc;
    rc = myact_read_status1(ctx, s1);
    if (rc < 0) return rc;
    if (s1->brake_release_command_state) return MYACT_OK;
    t0 = now_ms(NULL);
    rc = myact_brake_release(ctx);
    if (rc < 0) return rc;
    for (;;) {
        msleep(opts->brake_poll_ms);
        rc = myact_read_status1(ctx, s1);
        if (rc < 0) return rc;
        if (s1->brake_release_command_state) {
            printf("brake latency: %llums\n", (unsigned long long)(now_ms(NULL) - t0));
            return MYACT_OK;
        }
        if (now_ms(NULL) - t0 >= opts->brake_release_timeout_ms) break;
    }
    if (opts->force_motion_brake) return MYACT_OK;
    return MYACT_ESTATE;
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

    if (!opts.motion && !opts.raw_motion_smoke && !opts.raw_motion_smoke_sendonly &&
        !opts.enable_hold && !opts.brake_latency_test && !opts.brake_persistence_test &&
        !opts.rt_inc_burst) {
        printf("DIAG-ONLY mode: no motion requested. Use --motion to run motion tests.\n");
        rc = RET_OK;
        goto done;
    }

    if (opts.brake_latency_test) {
        rc = test_axis_enable(&ctx, &opts, &s1);
        if (rc < 0) fprintf(stderr, "brake-latency-test failed: %s (%d)\n", myact_strerror(rc), rc);
        rc = (rc < 0) ? RET_COMMS_FAIL : RET_OK;
        goto done;
    }

    if (opts.enable_hold) {
        rc = test_axis_enable(&ctx, &opts, &s1);
        if (rc < 0) { rc = RET_COMMS_FAIL; goto done; }
        (void)myact_stop_send(&ctx);
        printf("enable-hold: brake released and motor held enabled.\n");
        rc = RET_OK;
        goto done;
    }

    if (opts.raw_motion_smoke || opts.raw_motion_smoke_sendonly) {
        int32_t before = angle_multi_0p01;
        int32_t after = 0;
        myact_basic_feedback_t fb;
        memset(&fb, 0, sizeof(fb));
        rc = prepare_sync(&ctx);
        if (rc < 0) goto done;
        rc = myact_read_angle_multi(&ctx, &before);
        if (rc < 0) goto done;
        printf("raw-motion-smoke: before angle read OK\n");
        rc = motion_preflight(&ctx, &opts, &s1);
        printf("raw-motion-smoke: brake preflight returned %s (%d)\n", myact_strerror(rc), rc);
        if (rc < 0) {
            printf("raw-motion-smoke: skipping A8 because motion_preflight failed\n");
            rc = RET_COMMS_FAIL;
            goto done;
        }
        printf("raw-motion-smoke: sending A8 %+0.2f deg at %.0f dps\n", opts.smoke_delta_deg, opts.smoke_speed_dps);
        if (opts.raw_motion_smoke_sendonly) {
            rc = myact_position_inc_send(&ctx, (uint16_t)opts.smoke_speed_dps, myact_deg_to_raw_0p01(opts.smoke_delta_deg));
            if (rc < 0) {
                fprintf(stderr, "raw-motion-smoke: A8 failed: %s (%d)\n", myact_strerror(rc), rc);
                rc = RET_COMMS_FAIL;
                goto done;
            }
            msleep(opts.smoke_wait_ms);
        } else {
            memset(&fb, 0, sizeof(fb));
            rc = myact_position_inc(&ctx, (uint16_t)opts.smoke_speed_dps, myact_deg_to_raw_0p01(opts.smoke_delta_deg), &fb);
            if (rc < 0) {
                fprintf(stderr, "raw-motion-smoke: A8 failed: %s (%d)\n", myact_strerror(rc), rc);
                rc = RET_COMMS_FAIL;
                goto done;
            }
            msleep(opts.smoke_wait_ms);
        }
        printf("raw-motion-smoke: A8 reply OK\n");
        (void)myact_stop_send(&ctx);
        printf("raw-motion-smoke: stop sent\n");
        rc = myact_read_angle_multi(&ctx, &after);
        if (rc < 0) goto done;
        printf("raw-motion-smoke: after angle read OK\n");
        printf("raw-motion-smoke before: %.2f deg\n", myact_raw_0p01_to_deg(before));
        printf("raw-motion-smoke after: %.2f deg\n", myact_raw_0p01_to_deg(after));
        printf("raw-motion-smoke delta: %.2f deg\n", myact_raw_0p01_to_deg(after - before));
        rc = RET_OK;
        goto done;
    }

    if (opts.rt_inc_burst) {
        unsigned i;
        int32_t original = 0, cur = 0;
        rc = test_axis_enable(&ctx, &opts, &s1);
        if (rc < 0) { rc = RET_COMMS_FAIL; goto done; }
        rc = myact_read_angle_multi(&ctx, &original);
        if (rc < 0) { rc = RET_COMMS_FAIL; goto done; }
        for (i = 0; i < opts.cycles; ++i) {
            rc = myact_position_inc_send(&ctx, (uint16_t)opts.inc_speed_dps, myact_deg_to_raw_0p01(opts.inc_delta_deg));
            if (rc < 0) { rc = RET_COMMS_FAIL; goto done; }
            msleep(opts.inc_wait_ms);
            rc = myact_read_angle_multi(&ctx, &cur); if (rc < 0) { rc = RET_COMMS_FAIL; goto done; }
            printf("rt-inc-burst cycle %u +: angle=%.2f delta=%.2f\n", i + 1, myact_raw_0p01_to_deg(cur), myact_raw_0p01_to_deg(cur - original));
            rc = myact_position_inc_send(&ctx, (uint16_t)opts.inc_speed_dps, myact_deg_to_raw_0p01(-opts.inc_delta_deg));
            if (rc < 0) { rc = RET_COMMS_FAIL; goto done; }
            msleep(opts.inc_wait_ms);
            rc = myact_read_angle_multi(&ctx, &cur); if (rc < 0) { rc = RET_COMMS_FAIL; goto done; }
            printf("rt-inc-burst cycle %u -: angle=%.2f delta=%.2f\n", i + 1, myact_raw_0p01_to_deg(cur), myact_raw_0p01_to_deg(cur - original));
        }
        rc = RET_OK;
        goto done;
    }

    motion_mask = pick_motion_mask(&opts);
    printf("motion plan: angle_multi=%.2f deg bus=%.1fV brake=%s test=0x%X ",
           myact_raw_0p01_to_deg(angle_multi_0p01), s1.voltage_0p1v / 10.0,
           s1.brake_release_command_state ? "released" : "locked", motion_mask);
    if (motion_mask == MOTION_INC) {
        printf("selected=incremental delta=+2.00 deg max_speed=20 dps wait=1200 ms\n");
    } else {
        printf("targets:");
        if (motion_mask & MOTION_TORQUE) printf(" torque:+0.20A(300ms)");
        if (motion_mask & MOTION_SPEED) printf(" speed:+10dps(700ms)");
        if (motion_mask & MOTION_INC) printf(" inc:+2.00deg@20dps(wait1200ms)");
        printf("\n");
    }

    if (!opts.no_motion_countdown) msleep(2000);

    rc = motion_preflight(&ctx, &opts, &s1);
    if (rc < 0) { rc = RET_OK; goto done; }

    rc = run_motion_tests(&ctx, motion_mask);

done:
    if (opts.no_shutdown_on_exit) {
        test_axis_disable(&ctx, false);
        printf("WARNING: leaving motor not shutdown; brake may remain released.\n");
    } else {
        test_axis_disable(&ctx, true);
    }
    close(bus.s);
    return rc < 0 ? RET_COMMS_FAIL : rc;
}
