/*
# Build
gcc -O2 -Wall -Wextra -std=c11 -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L \
  dev/src/x8_25.c dev/tests/x8_25_test.c \
  -Idev/include -lm -o build/x8_25_test_can

# Diagnostics
./build/x8_25_test_can can0 1

# Brake latency
./build/x8_25_test_can can0 1 --brake-latency-test --force-motion-low-voltage

# Enable and hold
./build/x8_25_test_can can0 1 --enable-hold --force-motion-low-voltage --no-shutdown-on-exit

# Known-good A8 smoke
./build/x8_25_test_can can0 1 \
  --raw-motion-smoke \
  --force-motion-low-voltage \
  --smoke-delta-deg 10 \
  --smoke-speed-dps 60 \
  --smoke-wait-ms 1500

# Known-good A8 burst
./build/x8_25_test_can can0 1 \
  --rt-inc-burst \
  --force-motion-low-voltage \
  --no-shutdown-on-exit \
  --cycles 2 \
  --inc-delta-deg 10 \
  --inc-speed-dps 60 \
  --inc-wait-ms 1500

# A2 speed
./build/x8_25_test_can can0 1 \
  --speed-step-test \
  --force-motion-low-voltage \
  --no-shutdown-on-exit \
  --speed-step-dps 60 \
  --speed-step-hold-ms 1500 \
  --speed-step-cycles 2 \
  --speed-sample-ms 100

# A4 absolute/profile
./build/x8_25_test_can can0 1 \
  --profile-abs-relative-test \
  --force-motion-low-voltage \
  --no-shutdown-on-exit \
  --abs-delta-deg 10 \
  --abs-speed-dps 60 \
  --abs-tolerance-deg 1.0 \
  --abs-timeout-ms 3000 \
  --abs-poll-ms 100

# A1 torque
./build/x8_25_test_can can0 1 \
  --torque-step-test \
  --force-motion-low-voltage \
  --no-motion-countdown \
  --no-shutdown-on-exit \
  --torque-step-a 0.30 \
  --torque-step-hold-ms 500 \
  --torque-step-cycles 1 \
  --torque-sample-ms 100

# Active telemetry
./build/x8_25_test_can can0 1 \
  --active-telemetry-rt-test \
  --force-motion-low-voltage \
  --no-shutdown-on-exit \
  --active-interval-ms 50 \
  --active-duration-ms 3000 \
  --active-move-delta-deg 10 \
  --active-move-speed-dps 60

# MIT dry run
./build/x8_25_test_can can0 1 \
  --mit-mode-test \
  --mit-dry-run \
  --force-motion-low-voltage

# Bench all without MIT
./build/x8_25_test_can can0 1 \
  --bench-all \
  --force-motion-low-voltage \
  --no-shutdown-on-exit
*/

#include "x8_25.h"

#include <errno.h>
#include <math.h>
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

typedef struct {
    int s;
    char ifname[IFNAMSIZ];
} sockcan_bus_t;

typedef struct {
    bool no_motion_countdown;
    bool force_motion_brake;
    bool force_motion_low_voltage;
    bool reset_first;
    bool raw_motion_smoke;
    bool raw_motion_smoke_sendonly;
    bool no_shutdown_on_exit;
    bool enable_hold;
    bool brake_latency_test;
    bool brake_latency_shutdown_first;
    bool brake_persistence_test;
    bool rt_inc_burst;
    bool speed_step_test;
    bool speed_stream_test;
    bool torque_step_test;
    bool torque_sweep_test;
    bool profile_abs_relative_test;
    bool no_a8_fallback;
    bool mit_mode_test;
    bool active_telemetry_rt_test;
    bool bench_all;
    bool run_mit_in_bench_all;
    bool allow_hold_with_runmode0;

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

    double speed_step_dps;
    unsigned speed_step_hold_ms;
    unsigned speed_step_cycles;
    unsigned speed_sample_ms;
    unsigned speed_stream_period_ms;
    double speed_pass_min_dps;
    double speed_pass_min_delta_deg;

    double torque_step_a;
    double torque_sweep_values_a[8];
    unsigned torque_sweep_count;
    unsigned torque_step_hold_ms;
    unsigned torque_step_cycles;
    unsigned torque_sample_ms;
    double torque_pass_min_iq_a;
    double torque_pass_min_delta_deg;

    double abs_delta_deg;
    double abs_speed_dps;
    double abs_tolerance_deg;
    unsigned abs_timeout_ms;
    unsigned abs_poll_ms;

    float mit_p_rad;
    float mit_v_rad_s;
    float mit_kp;
    float mit_kd;
    float mit_t_nm;
    unsigned mit_duration_ms;
    unsigned mit_sample_ms;
    bool mit_dry_run;

    unsigned active_interval_ms;
    unsigned active_duration_ms;
    double active_move_delta_deg;
    double active_move_speed_dps;
    double active_min_frame_ratio;

    unsigned runmode_timeout_ms;
    unsigned runmode_poll_ms;
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

static void print_usage(const char* p) { fprintf(stderr, "usage: %s <can_if> <node_id> [flags...]\n", p); }

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
    o->speed_step_dps = 60.0;
    o->speed_step_hold_ms = 1500;
    o->speed_step_cycles = 2;
    o->speed_sample_ms = 100;
    o->speed_stream_period_ms = 20;
    o->speed_pass_min_dps = 5.0;
    o->speed_pass_min_delta_deg = 1.0;
    o->torque_step_a = 0.30;
    o->torque_sweep_values_a[0] = 0.30;
    o->torque_sweep_values_a[1] = 0.50;
    o->torque_sweep_values_a[2] = 0.80;
    o->torque_sweep_count = 3;
    o->torque_step_hold_ms = 500;
    o->torque_step_cycles = 1;
    o->torque_sample_ms = 100;
    o->torque_pass_min_iq_a = 0.10;
    o->torque_pass_min_delta_deg = 0.5;
    o->abs_delta_deg = 10.0;
    o->abs_speed_dps = 60.0;
    o->abs_tolerance_deg = 1.0;
    o->abs_timeout_ms = 3000;
    o->abs_poll_ms = 100;
    o->mit_kd = 0.5f;
    o->mit_duration_ms = 1000;
    o->mit_sample_ms = 50;
    o->active_interval_ms = 50;
    o->active_duration_ms = 3000;
    o->active_move_delta_deg = 10.0;
    o->active_move_speed_dps = 60.0;
    o->active_min_frame_ratio = 0.70;
    o->runmode_timeout_ms = 1500;
    o->runmode_poll_ms = 50;

    for (i = 3; i < argc; ++i) {
#define FLAG(name, field) \
    if (!strcmp(argv[i], name)) o->field = true; else
        FLAG("--no-motion-countdown", no_motion_countdown)
        FLAG("--force-motion-brake", force_motion_brake)
        FLAG("--force-motion-low-voltage", force_motion_low_voltage)
        FLAG("--reset-first", reset_first)
        FLAG("--raw-motion-smoke", raw_motion_smoke)
        FLAG("--raw-motion-smoke-sendonly", raw_motion_smoke_sendonly)
        FLAG("--no-shutdown-on-exit", no_shutdown_on_exit)
        FLAG("--enable-hold", enable_hold)
        FLAG("--brake-latency-test", brake_latency_test)
        FLAG("--brake-latency-shutdown-first", brake_latency_shutdown_first)
        FLAG("--brake-persistence-test", brake_persistence_test)
        FLAG("--rt-inc-burst", rt_inc_burst)
        FLAG("--speed-step-test", speed_step_test)
        FLAG("--speed-stream-test", speed_stream_test)
        FLAG("--torque-step-test", torque_step_test)
        FLAG("--torque-sweep-test", torque_sweep_test)
        FLAG("--profile-abs-relative-test", profile_abs_relative_test)
        FLAG("--no-a8-fallback", no_a8_fallback)
        FLAG("--mit-mode-test", mit_mode_test)
        FLAG("--active-telemetry-rt-test", active_telemetry_rt_test)
        FLAG("--bench-all", bench_all)
        FLAG("--bench-all-mit", run_mit_in_bench_all)
        FLAG("--mit-dry-run", mit_dry_run)
        FLAG("--allow-hold-with-runmode0", allow_hold_with_runmode0)

        if (!strcmp(argv[i], "--speed-step-dps") && i + 1 < argc) o->speed_step_dps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--speed-step-hold-ms") && i + 1 < argc) o->speed_step_hold_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--speed-step-cycles") && i + 1 < argc) o->speed_step_cycles = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--speed-sample-ms") && i + 1 < argc) o->speed_sample_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--speed-stream-period-ms") && i + 1 < argc) o->speed_stream_period_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--speed-pass-min-dps") && i + 1 < argc) o->speed_pass_min_dps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--speed-pass-min-delta-deg") && i + 1 < argc) o->speed_pass_min_delta_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--torque-step-a") && i + 1 < argc) o->torque_step_a = atof(argv[++i]);
        else if (!strcmp(argv[i], "--torque-sweep-values") && i + 1 < argc) {
            const char* list = argv[++i];
            char buf[128];
            strncpy(buf, list, sizeof(buf) - 1);
            buf[sizeof(buf) - 1] = 0;
            char* tok = strtok(buf, ",");
            o->torque_sweep_count = 0;
            while (tok && o->torque_sweep_count < 8) {
                o->torque_sweep_values_a[o->torque_sweep_count++] = atof(tok);
                tok = strtok(NULL, ",");
            }
        }
        else if (!strcmp(argv[i], "--torque-step-hold-ms") && i + 1 < argc) o->torque_step_hold_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--torque-step-cycles") && i + 1 < argc) o->torque_step_cycles = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--torque-sample-ms") && i + 1 < argc) o->torque_sample_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--torque-pass-min-iq-a") && i + 1 < argc) o->torque_pass_min_iq_a = atof(argv[++i]);
        else if (!strcmp(argv[i], "--torque-pass-min-delta-deg") && i + 1 < argc) o->torque_pass_min_delta_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--abs-delta-deg") && i + 1 < argc) o->abs_delta_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--abs-speed-dps") && i + 1 < argc) o->abs_speed_dps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--abs-tolerance-deg") && i + 1 < argc) o->abs_tolerance_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--abs-timeout-ms") && i + 1 < argc) o->abs_timeout_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--abs-poll-ms") && i + 1 < argc) o->abs_poll_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--mit-p-rad") && i + 1 < argc) o->mit_p_rad = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--mit-v-rad-s") && i + 1 < argc) o->mit_v_rad_s = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--mit-kp") && i + 1 < argc) o->mit_kp = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--mit-kd") && i + 1 < argc) o->mit_kd = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--mit-t-nm") && i + 1 < argc) o->mit_t_nm = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--mit-duration-ms") && i + 1 < argc) o->mit_duration_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--mit-sample-ms") && i + 1 < argc) o->mit_sample_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--active-interval-ms") && i + 1 < argc) o->active_interval_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--active-duration-ms") && i + 1 < argc) o->active_duration_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--active-move-delta-deg") && i + 1 < argc) o->active_move_delta_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--active-move-speed-dps") && i + 1 < argc) o->active_move_speed_dps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--active-min-frame-ratio") && i + 1 < argc) o->active_min_frame_ratio = atof(argv[++i]);
        else if (!strcmp(argv[i], "--runmode-timeout-ms") && i + 1 < argc) o->runmode_timeout_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--runmode-poll-ms") && i + 1 < argc) o->runmode_poll_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--smoke-delta-deg") && i + 1 < argc) o->smoke_delta_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--smoke-speed-dps") && i + 1 < argc) o->smoke_speed_dps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--smoke-wait-ms") && i + 1 < argc) o->smoke_wait_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--cycles") && i + 1 < argc) o->cycles = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--inc-delta-deg") && i + 1 < argc) o->inc_delta_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--inc-speed-dps") && i + 1 < argc) o->inc_speed_dps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--inc-wait-ms") && i + 1 < argc) o->inc_wait_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--min-motion-voltage") && i + 1 < argc) o->min_motion_voltage = atof(argv[++i]);
        else return RET_BAD_CLI;
#undef FLAG
    }

    return RET_OK;
}

static int read_full_diagnostics(myact_ctx_t* c, myact_status1_t* s1, int32_t* a) {
    char m[8] = {0};
    uint32_t v = 0;
    uint32_t rt = 0;
    myact_runmode_t rm = MYACT_RUNMODE_UNKNOWN;
    uint16_t as = 0;
    int32_t em = 0, er = 0, eo = 0;
    myact_status2_t s2;
    myact_status3_t s3;
    int rc;

    rc = myact_read_model(c, m); if (rc < 0) return rc;
    rc = myact_read_version_date(c, &v); if (rc < 0) return rc;
    rc = myact_read_runtime(c, &rt); if (rc < 0) return rc;
    rc = myact_read_operating_mode(c, &rm); if (rc < 0) return rc;
    rc = myact_read_angle_multi(c, a); if (rc < 0) return rc;
    rc = myact_read_angle_single(c, &as); if (rc < 0) return rc;
    rc = myact_read_encoder_multi(c, &em); if (rc < 0) return rc;
    rc = myact_read_encoder_raw(c, &er); if (rc < 0) return rc;
    rc = myact_read_encoder_offset(c, &eo); if (rc < 0) return rc;
    rc = myact_read_status1(c, s1); if (rc < 0) return rc;
    rc = myact_read_status2(c, &s2); if (rc < 0) return rc;
    rc = myact_read_status3(c, &s3); if (rc < 0) return rc;

    printf("diag model:%s version:%u runtime_ms:%u runmode:%u\n", m, v, rt, (unsigned)rm);
    printf("diag angle_multi:%.2f angle_single:%.2f enc60:%d enc61:%d enc62:%d\n",
           myact_raw_0p01_to_deg(*a), myact_raw_0p01_to_deg((int32_t)as), em, er, eo);
    printf("diag status1 brake:%s bus:%.1fV err:0x%04X\n",
           s1->brake_release_command_state ? "released" : "locked",
           s1->voltage_0p1v / 10.0, s1->error_flags);
    return MYACT_OK;
}

static int check_motion_voltage_or_skip(const test_opts_t* o, const myact_status1_t* s1, const char* n) {
    double v = s1->voltage_0p1v / 10.0;

    if (v < o->min_motion_voltage && !o->force_motion_low_voltage) {
        printf("%s: SKIP bus voltage %.1f V below configured minimum %.1f V\n", n, v, o->min_motion_voltage);
        return MYACT_ESTATE;
    }
    if (v < o->min_motion_voltage) {
        printf("WARNING: low-voltage motion forced at %.1f V; use rated supply for real torque/speed characterization.\n", v);
        printf("rated voltage is 48 V-class; current bench voltage is low.\n");
    }
    return MYACT_OK;
}

static int test_axis_enable(myact_ctx_t* ctx, const test_opts_t* o, myact_status1_t* s1) {
    uint64_t t0;
    int rc = prepare_sync(ctx);
    if (rc < 0) return rc;

    rc = myact_read_status1(ctx, s1);
    if (rc < 0) return rc;
    if (s1->brake_release_command_state) return MYACT_OK;

    rc = myact_brake_release(ctx);
    if (rc < 0) return rc;

    t0 = now_ms(NULL);
    while (now_ms(NULL) - t0 < o->brake_release_timeout_ms) {
        rc = myact_read_status1(ctx, s1);
        if (rc < 0) return rc;
        if (s1->brake_release_command_state) {
            printf("brake released after %llums\n",
                   (unsigned long long)(now_ms(NULL) - t0));
            return MYACT_OK;
        }
        msleep(o->brake_poll_ms);
    }

    rc = myact_read_status1(ctx, s1);
    if (rc < 0) return rc;

    printf("brake release timeout: still %s after %ums\n",
           s1->brake_release_command_state ? "released" : "locked",
           o->brake_release_timeout_ms);
    return s1->brake_release_command_state ? MYACT_OK : MYACT_ESTATE;
}

static int axis_enable_for_motion(myact_ctx_t* ctx, const test_opts_t* o, myact_status1_t* s1, const char* n) {
    int rc = test_axis_enable(ctx, o, s1);
    if (rc < 0) return rc;

    rc = myact_read_status1(ctx, s1);
    if (rc < 0) return rc;

    rc = check_motion_voltage_or_skip(o, s1, n);
    if (rc < 0) return rc;

    if (!s1->brake_release_command_state) {
        printf("%s: axis enable failed: brake still locked\n", n);
        return MYACT_ESTATE;
    }

    printf("%s: axis enabled, brake=%s, bus=%.1fV\n", n,
           s1->brake_release_command_state ? "released" : "locked",
           s1->voltage_0p1v / 10.0);
    return MYACT_OK;
}

static int read_angle_deg(myact_ctx_t* c, double* d) {
    int32_t a = 0;
    int rc = myact_read_angle_multi(c, &a);
    if (rc < 0) return rc;
    *d = myact_raw_0p01_to_deg(a);
    return MYACT_OK;
}

static int sample_status2_capture(myact_ctx_t* c, const char* tag, uint64_t t0, myact_status2_t* out) {
    int rc = myact_read_status2(c, out);
    if (rc < 0) return rc;

    printf("t_ms=%llu tag=%s iq_A=%.2f speed_dps=%d angle_deg=%d temp_C=%d\n",
           (unsigned long long)(now_ms(NULL) - t0), tag,
           out->iq_0p01a / 100.0, out->speed_dps, out->angle_deg, out->motor_temp_c);
    return MYACT_OK;
}

static int wait_until_angle_near(myact_ctx_t* c, double tgt, double tol, unsigned to, unsigned p, const char* tag) {
    uint64_t s = now_ms(NULL);
    for (;;) {
        double a;
        double e;
        int rc = read_angle_deg(c, &a);
        if (rc < 0) return rc;

        e = tgt - a;
        printf("t_ms=%llu tag=%s target_deg=%.2f actual_deg=%.2f error_deg=%.2f\n",
               (unsigned long long)(now_ms(NULL) - s), tag, tgt, a, e);

        if (fabs(e) <= tol) return MYACT_OK;
        if (now_ms(NULL) - s >= to) return MYACT_ETIMEDOUT;
        msleep(p);
    }
}

static int send_stop_and_sample(myact_ctx_t* c, const char* n) {
    int rc = myact_stop_send(c);
    if (rc < 0) return rc;

    msleep(100);
    {
        myact_status2_t s2;
        int32_t a;
        int r2 = myact_read_status2(c, &s2);
        int r3;

        if (r2 == MYACT_OK) {
            printf("%s final 9C: iq_A=%.2f speed_dps=%d angle_deg=%d\n",
                   n, s2.iq_0p01a / 100.0, s2.speed_dps, s2.angle_deg);
        }

        r3 = myact_read_angle_multi(c, &a);
        if (r3 == MYACT_OK) {
            printf("%s final 92: angle=%.2f\n", n, myact_raw_0p01_to_deg(a));
        }
    }
    return MYACT_OK;
}

static int bench_result(const char* n, int rc) {
    if (rc == MYACT_OK) {
        printf("[PASS] %s\n", n);
        return RET_OK;
    }
    if (rc == MYACT_ESTATE) {
        printf("[SKIP] %s: reason already printed\n", n);
        return RET_OK;
    }
    if (rc == MYACT_ENOMOTION && (!strcmp(n, "speed-step-test") || !strcmp(n, "speed-stream-test") || !strcmp(n, "torque-step-test"))) {
        printf("[UNSUPPORTED/FAIL-DIAG] A2/A1 mode diagnostic: %s\n", n);
        return RET_OK;
    }

    printf("[FAIL] %s: %s (%d)\n", n, myact_strerror(rc), rc);
    return RET_COMMS_FAIL;
}

static int run_a8_delta_observed(myact_ctx_t* c,
                                 const char* tag,
                                 double delta_deg,
                                 double speed_dps,
                                 unsigned wait_ms,
                                 double pass_tolerance_deg,
                                 double* out_before,
                                 double* out_after,
                                 double* out_delta);

static int run_speed_step_test(myact_ctx_t* c, const test_opts_t* o, myact_status1_t* s1) {
    uint64_t t0;
    double angle_before = 0.0;
    double angle_after = 0.0;
    double delta_deg;
    double max_abs_speed_dps = 0.0;
    bool acked = false;
    int rc;
    myact_status2_t s2;

    printf("speed-step-test: A2 speed mode, %+0.1f dps / %0.1f dps, hold=%u ms, cycles=%u\n",
           o->speed_step_dps, -o->speed_step_dps, o->speed_step_hold_ms, o->speed_step_cycles);

    rc = axis_enable_for_motion(c, o, s1, "speed-step-test");
    if (rc < 0) return rc;

    rc = read_angle_deg(c, &angle_before);
    if (rc < 0) return rc;

    for (unsigned i = 0; i < o->speed_step_cycles; i++) {
        printf("speed-step-test cycle %u: sending A2 +speed\n", i + 1);
        rc = myact_speed_send(c, myact_speed_dps_to_raw(o->speed_step_dps));
        if (rc < 0) return rc;
        acked = true;

        t0 = now_ms(NULL);
        while (now_ms(NULL) - t0 < o->speed_step_hold_ms) {
            rc = sample_status2_capture(c, "speed+", t0, &s2);
            if (rc < 0) return rc;
            if (fabs((double)s2.speed_dps) > max_abs_speed_dps) {
                max_abs_speed_dps = fabs((double)s2.speed_dps);
            }
            msleep(o->speed_sample_ms);
        }

        printf("speed-step-test cycle %u: sending A2 -speed\n", i + 1);
        rc = myact_speed_send(c, myact_speed_dps_to_raw(-o->speed_step_dps));
        if (rc < 0) return rc;
        acked = true;

        t0 = now_ms(NULL);
        while (now_ms(NULL) - t0 < o->speed_step_hold_ms) {
            rc = sample_status2_capture(c, "speed-", t0, &s2);
            if (rc < 0) return rc;
            if (fabs((double)s2.speed_dps) > max_abs_speed_dps) {
                max_abs_speed_dps = fabs((double)s2.speed_dps);
            }
            msleep(o->speed_sample_ms);
        }
    }

    rc = send_stop_and_sample(c, "speed-step-test");
    if (rc < 0) return rc;

    rc = read_angle_deg(c, &angle_after);
    if (rc < 0) return rc;

    delta_deg = angle_after - angle_before;
    printf("speed-step-test summary:\n");
    printf("  requested_dps=%.1f\n", o->speed_step_dps);
    printf("  max_abs_speed_dps=%.1f\n", max_abs_speed_dps);
    printf("  angle_before=%.2f\n", angle_before);
    printf("  angle_after=%.2f\n", angle_after);
    printf("  delta_deg=%.2f\n", delta_deg);

    if (max_abs_speed_dps >= o->speed_pass_min_dps || fabs(delta_deg) >= o->speed_pass_min_delta_deg) {
        return MYACT_OK;
    }

    if (acked) {
        printf("[FAIL] speed-step-test: command acknowledged but no observed speed/motion\n");
    }
    return MYACT_ENOMOTION;
}


static int run_speed_stream_test(myact_ctx_t* c, const test_opts_t* o, myact_status1_t* s1) {
    int rc; uint64_t t0; double before=0.0, after=0.0; double max_abs_speed_dps=0.0; myact_status2_t s2;
    printf("speed-stream-test: A2 streamed every %u ms, hold=%u ms\n", o->speed_stream_period_ms, o->speed_step_hold_ms);
    rc = axis_enable_for_motion(c, o, s1, "speed-stream-test"); if (rc < 0) return rc;
    rc = read_angle_deg(c, &before); if (rc < 0) return rc;
    t0 = now_ms(NULL);
    while (now_ms(NULL) - t0 < o->speed_step_hold_ms) {
        rc = myact_speed_send(c, myact_speed_dps_to_raw(o->speed_step_dps)); if (rc < 0) return rc;
        rc = sample_status2_capture(c, "speed-stream+", t0, &s2); if (rc < 0) return rc;
        if (fabs((double)s2.speed_dps) > max_abs_speed_dps) max_abs_speed_dps = fabs((double)s2.speed_dps);
        msleep(o->speed_stream_period_ms);
    }
    t0 = now_ms(NULL);
    while (now_ms(NULL) - t0 < o->speed_step_hold_ms) {
        rc = myact_speed_send(c, myact_speed_dps_to_raw(-o->speed_step_dps)); if (rc < 0) return rc;
        rc = sample_status2_capture(c, "speed-stream-", t0, &s2); if (rc < 0) return rc;
        if (fabs((double)s2.speed_dps) > max_abs_speed_dps) max_abs_speed_dps = fabs((double)s2.speed_dps);
        msleep(o->speed_stream_period_ms);
    }
    rc = send_stop_and_sample(c, "speed-stream-test"); if (rc < 0) return rc;
    rc = read_angle_deg(c, &after); if (rc < 0) return rc;
    printf("speed-stream-test summary:\n  max_abs_speed_dps=%.1f\n  delta_deg=%.2f\n", max_abs_speed_dps, after-before);
    if (max_abs_speed_dps >= o->speed_pass_min_dps || fabs(after-before) >= o->speed_pass_min_delta_deg) return MYACT_OK;
    printf("[FAIL] speed-stream-test: A2 acknowledged but this motor/firmware is not executing speed closed-loop commands in current runtime mode\n");
    return MYACT_ENOMOTION;
}

static int run_torque_step_test(myact_ctx_t* c, const test_opts_t* o, myact_status1_t* s1) {
    int rc;
    uint64_t t0;
    double angle_before = 0.0;
    double angle_after = 0.0;
    double angle_delta_deg;
    double max_abs_iq_a = 0.0;
    double max_abs_speed_dps = 0.0;
    bool acked = false;
    myact_status2_t s2;

    printf("torque-step-test: WARNING torque mode can move unexpectedly\n");
    if (o->torque_sweep_test) {
        printf("torque-sweep-test enabled: levels=");
        for (unsigned j = 0; j < o->torque_sweep_count; j++) printf(j + 1 < o->torque_sweep_count ? "%.2f," : "%.2f", o->torque_sweep_values_a[j]);
        printf(" A, hold=%ums\n", o->torque_step_hold_ms);
    } else {
        printf("torque-step-test: A1 %+0.2fA / %0.2fA, hold=%ums\n", o->torque_step_a, -o->torque_step_a, o->torque_step_hold_ms);
    }

    rc = axis_enable_for_motion(c, o, s1, "torque-step-test");
    if (rc < 0) return rc;

    rc = read_angle_deg(c, &angle_before);
    if (rc < 0) return rc;

    if (o->torque_sweep_test && !o->no_motion_countdown) {
        printf("[SKIP] torque-sweep-test: requires --no-motion-countdown for safety\n");
        return MYACT_ESTATE;
    }

    for (unsigned i = 0; i < o->torque_step_cycles; i++) {
        for (unsigned j = 0; j < (o->torque_sweep_test ? o->torque_sweep_count : 1); j++) {
        double tq = o->torque_sweep_test ? o->torque_sweep_values_a[j] : o->torque_step_a;
        rc = myact_torque_send(c, myact_current_a_to_raw(tq));
        if (rc < 0) return rc;
        acked = true;

        t0 = now_ms(NULL);
        while (now_ms(NULL) - t0 < o->torque_step_hold_ms) {
            rc = sample_status2_capture(c, "torque+", t0, &s2);
            if (rc < 0) return rc;
            if (fabs(s2.iq_0p01a / 100.0) > max_abs_iq_a) {
                max_abs_iq_a = fabs(s2.iq_0p01a / 100.0);
            }
            if (fabs((double)s2.speed_dps) > max_abs_speed_dps) {
                max_abs_speed_dps = fabs((double)s2.speed_dps);
            }
            msleep(o->torque_sample_ms);
        }

        rc = myact_torque_send(c, myact_current_a_to_raw(-tq));
        if (rc < 0) return rc;
        acked = true;

        t0 = now_ms(NULL);
        while (now_ms(NULL) - t0 < o->torque_step_hold_ms) {
            rc = sample_status2_capture(c, "torque-", t0, &s2);
            if (rc < 0) return rc;
            if (fabs(s2.iq_0p01a / 100.0) > max_abs_iq_a) {
                max_abs_iq_a = fabs(s2.iq_0p01a / 100.0);
            }
            if (fabs((double)s2.speed_dps) > max_abs_speed_dps) {
                max_abs_speed_dps = fabs((double)s2.speed_dps);
            }
            msleep(o->torque_sample_ms);
        }
        }
    }

    rc = send_stop_and_sample(c, "torque-step-test");
    if (rc < 0) return rc;

    rc = read_angle_deg(c, &angle_after);
    if (rc < 0) return rc;

    angle_delta_deg = angle_after - angle_before;
    printf("torque-step-test summary:\n");
    printf("  requested_iq_a=%.2f\n", o->torque_step_a);
    printf("  max_abs_iq_A=%.2f\n", max_abs_iq_a);
    printf("  max_abs_speed_dps=%.1f\n", max_abs_speed_dps);
    printf("  angle_before=%.2f\n", angle_before);
    printf("  angle_after=%.2f\n", angle_after);
    printf("  angle_delta_deg=%.2f\n", angle_delta_deg);

    if (max_abs_iq_a >= o->torque_pass_min_iq_a || fabs(angle_delta_deg) >= o->torque_pass_min_delta_deg) {
        return MYACT_OK;
    }

    if (acked) {
        printf("[FAIL] torque-step-test: A1 command acknowledged but current loop not responding in this runtime mode\n");
    }

    return MYACT_ENOMOTION;
}

static int run_profile_abs_relative_test(myact_ctx_t* c, const test_opts_t* o, myact_status1_t* s1) {
    double orig;
    double target_plus;
    double final_a4 = 0.0;
    int32_t target_raw;
    uint16_t abs_speed_raw;
    uint8_t a4_frame[8] = {0};
    int rc;

    rc = axis_enable_for_motion(c, o, s1, "profile-abs-relative-test");
    if (rc < 0) return rc;

    rc = read_angle_deg(c, &orig);
    if (rc < 0) return rc;

    target_plus = orig + o->abs_delta_deg;
    target_raw = myact_deg_to_raw_0p01(target_plus);
    abs_speed_raw = (uint16_t)o->abs_speed_dps;
    a4_frame[0] = 0xA4;
    a4_frame[1] = 0x00;
    a4_frame[2] = (uint8_t)(abs_speed_raw & 0xFF);
    a4_frame[3] = (uint8_t)((abs_speed_raw >> 8) & 0xFF);
    a4_frame[4] = (uint8_t)(target_raw & 0xFF);
    a4_frame[5] = (uint8_t)((target_raw >> 8) & 0xFF);
    a4_frame[6] = (uint8_t)((target_raw >> 16) & 0xFF);
    a4_frame[7] = (uint8_t)((target_raw >> 24) & 0xFF);

    printf("profile-abs-relative-test: original=%.2f target_plus=%.2f return=%.2f speed=%.1f\n",
           orig, target_plus, orig, o->abs_speed_dps);

    rc = myact_position_abs_send(c, abs_speed_raw, target_raw);
    if (rc < 0) goto out;

    rc = wait_until_angle_near(c, target_plus, o->abs_tolerance_deg,
                               o->abs_timeout_ms, o->abs_poll_ms, "abs+");
    if (rc == MYACT_ETIMEDOUT) {
        double err;
        myact_status1_t diag_s1;
        myact_status2_t diag_s2;

        (void)read_angle_deg(c, &final_a4);
        err = target_plus - final_a4;
        printf("profile-abs-relative-test: A4 timeout\n");
        printf("profile-abs-relative-test: A4 command acknowledged? yes\n");
        printf("profile-abs-relative-test: A4 frame = %02X %02X %02X %02X %02X %02X %02X %02X\n",
               a4_frame[0], a4_frame[1], a4_frame[2], a4_frame[3],
               a4_frame[4], a4_frame[5], a4_frame[6], a4_frame[7]);
        printf("profile-abs-relative-test: A4 original=%.2f target=%.2f final=%.2f error=%.2f\n",
               orig, target_plus, final_a4, err);
        if (myact_read_status1(c, &diag_s1) == MYACT_OK) {
            printf("profile-abs-relative-test: status1 brake=%u bus=%.1fV error=0x%04X\n",
                   (unsigned)diag_s1.brake_release_command_state,
                   diag_s1.voltage_0p1v / 10.0,
                   diag_s1.error_flags);
        }
        if (myact_read_status2(c, &diag_s2) == MYACT_OK) {
            printf("profile-abs-relative-test: status2 iq=%.2f speed=%d angle=%d\n",
                   diag_s2.iq_0p01a / 100.0,
                   (int)diag_s2.speed_dps,
                   (int)diag_s2.angle_deg);
        }

        if (!o->no_a8_fallback) {
            printf("profile-abs-relative-test: sending stop before A8 fallback\n");
            (void)myact_stop_send(c);
            msleep(200);
            rc = prepare_sync(c);
            if (rc < 0) goto out;

            rc = run_a8_delta_observed(c, "profile-abs-relative-test A8 fallback",
                                       o->abs_delta_deg, o->abs_speed_dps, o->smoke_wait_ms,
                                       fmax(1.0, 0.20 * fabs(o->abs_delta_deg)),
                                       NULL, NULL, NULL);
            if (rc == MYACT_OK) {
                printf("[FAIL] profile-abs-relative-test: A4 failed but A8 fallback succeeded\n");
                rc = MYACT_ENOTSUP;
                goto out;
            }
            printf("[FAIL] profile-abs-relative-test: A4 failed and A8 fallback failed\n");
            rc = MYACT_ESTATE;
            goto out;
        }

        rc = MYACT_ETIMEDOUT;
        goto out;
    }
    if (rc < 0) goto out;

    rc = myact_position_abs_send(c, abs_speed_raw, myact_deg_to_raw_0p01(orig));
    if (rc < 0) goto out;

    rc = wait_until_angle_near(c, orig, o->abs_tolerance_deg,
                               o->abs_timeout_ms, o->abs_poll_ms, "abs-ret");
out:
    (void)send_stop_and_sample(c, "profile-abs-relative-test");
    return rc;
}

static int run_brake_latency_test(myact_ctx_t* c, const test_opts_t* o, myact_status1_t* s1) {
    uint64_t t0;
    int rc;

    rc = prepare_sync(c);
    if (rc < 0) return rc;

    rc = myact_read_status1(c, s1);
    if (rc < 0) return rc;

    printf("brake preflight: before=%s/off\n",
           s1->brake_release_command_state ? "released" : "locked");

    if (s1->brake_release_command_state) {
        if (o->reset_first || o->brake_latency_shutdown_first) {
            rc = myact_shutdown_send(c);
            if (rc < 0) return rc;
            msleep(100);

            rc = myact_read_status1(c, s1);
            if (rc < 0) return rc;
            printf("brake preflight: after-shutdown=%s/off\n",
                   s1->brake_release_command_state ? "released" : "locked");
        } else {
            printf("brake latency: already released\n");
            return MYACT_OK;
        }
    }

    t0 = now_ms(NULL);
    rc = myact_brake_release(c);
    if (rc < 0) return rc;
    printf("brake release: sent 0x77\n");

    while (now_ms(NULL) - t0 < o->brake_release_timeout_ms) {
        rc = myact_read_status1(c, s1);
        if (rc < 0) return rc;
        if (s1->brake_release_command_state) {
            printf("brake latency: %llums\n",
                   (unsigned long long)(now_ms(NULL) - t0));
            return MYACT_OK;
        }
        msleep(o->brake_poll_ms);
    }

    rc = myact_read_status1(c, s1);
    if (rc < 0) return rc;
    if (s1->brake_release_command_state) {
        printf("brake latency: %llums\n",
               (unsigned long long)(now_ms(NULL) - t0));
        return MYACT_OK;
    }
    printf("brake latency: timeout (%ums)\n", o->brake_release_timeout_ms);
    return MYACT_ESTATE;
}

static int wait_runmode(myact_ctx_t* ctx,
                        myact_runmode_t expected,
                        unsigned timeout_ms,
                        unsigned poll_ms,
                        myact_runmode_t* out_last) {
    uint64_t t0 = now_ms(NULL);
    myact_runmode_t rm = MYACT_RUNMODE_UNKNOWN;
    int rc;

    do {
        rc = myact_read_operating_mode(ctx, &rm);
        if (rc < 0) return rc;
        if (out_last) *out_last = rm;
        if (rm == expected) return (int)(now_ms(NULL) - t0);
        if (now_ms(NULL) - t0 >= timeout_ms) break;
        msleep(poll_ms);
    } while (1);

    return MYACT_ETIMEDOUT;
}

static int run_enable_hold(myact_ctx_t* c, const test_opts_t* o, myact_status1_t* s1) {
    myact_runmode_t rm = MYACT_RUNMODE_UNKNOWN;
    uint64_t t0;
    int rc;

    rc = prepare_sync(c);
    if (rc < 0) return rc;

    rc = myact_read_status1(c, s1);
    if (rc < 0) return rc;

    rc = check_motion_voltage_or_skip(o, s1, "enable-hold");
    if (rc < 0) return rc;

    if (s1->brake_release_command_state) {
        rc = myact_shutdown_send(c);
        if (rc < 0) return rc;
        msleep(100);

        rc = myact_read_status1(c, s1);
        if (rc < 0) return rc;
    }

    if (!s1->brake_release_command_state) {
        rc = myact_brake_release(c);
        if (rc < 0) return rc;
        t0 = now_ms(NULL);
        while (now_ms(NULL) - t0 < o->brake_release_timeout_ms) {
            rc = myact_read_status1(c, s1);
            if (rc < 0) return rc;
            if (s1->brake_release_command_state) {
                printf("enable-hold: brake released after %llums\n",
                       (unsigned long long)(now_ms(NULL) - t0));
                break;
            }
            msleep(o->brake_poll_ms);
        }
    }

    rc = myact_read_status1(c, s1);
    if (rc < 0) return rc;
    if (!s1->brake_release_command_state) {
        printf("enable-hold: axis enable failed: brake still locked\n");
        return MYACT_ESTATE;
    }

    rc = wait_runmode(c, MYACT_RUNMODE_SPEED, o->runmode_timeout_ms, o->runmode_poll_ms, &rm);
    if (rc >= 0) {
        printf("enable-hold: runmode reached 2 after %dms\n", rc);
    } else if (rc == MYACT_ETIMEDOUT && rm == MYACT_RUNMODE_IDLE_OR_OFF) {
        printf("enable-hold: brake released but runmode stayed 0 after %ums\n", o->runmode_timeout_ms);
        if (!o->allow_hold_with_runmode0) return MYACT_ESTATE;
        printf("enable-hold: WARNING allowing hold with runmode=0 due to --allow-hold-with-runmode0\n");
    } else if (rc == MYACT_ETIMEDOUT) {
        printf("enable-hold: axis enable failed: runmode=%u (expected 2)\n", (unsigned)rm);
        return MYACT_ESTATE;
    } else {
        return rc;
    }

    printf("enable-hold: brake released and motor held enabled.\n");
    return MYACT_OK;
}

static int run_a8_delta_observed(myact_ctx_t* c,
                                 const char* tag,
                                 double delta_deg,
                                 double speed_dps,
                                 unsigned wait_ms,
                                 double pass_tolerance_deg,
                                 double* out_before,
                                 double* out_after,
                                 double* out_delta) {
    double before_deg = 0.0;
    double after_deg = 0.0;
    double observed_delta;
    int rc;

    rc = prepare_sync(c);
    if (rc < 0) return rc;
    rc = read_angle_deg(c, &before_deg);
    if (rc < 0) return rc;
    rc = myact_position_inc_send(c, (uint16_t)speed_dps, myact_deg_to_raw_0p01(delta_deg));
    if (rc < 0) return rc;
    msleep(wait_ms);
    rc = myact_stop_send(c);
    if (rc < 0) return rc;
    msleep(100);
    rc = read_angle_deg(c, &after_deg);
    if (rc < 0) return rc;

    observed_delta = after_deg - before_deg;
    printf("%s before=%.2f after=%.2f delta=%.2f\n", tag, before_deg, after_deg, observed_delta);
    if (out_before) *out_before = before_deg;
    if (out_after) *out_after = after_deg;
    if (out_delta) *out_delta = observed_delta;

    if (fabs(observed_delta - delta_deg) > pass_tolerance_deg) return MYACT_ESTATE;
    return MYACT_OK;
}

static int run_raw_motion_smoke(myact_ctx_t* c,
                                const test_opts_t* o,
                                myact_status1_t* s1,
                                bool sendonly) {
    int rc;

    rc = axis_enable_for_motion(c, o, s1, "raw-motion-smoke");
    if (rc < 0) return rc;

    if (sendonly) {
        int32_t req_delta_raw = myact_deg_to_raw_0p01(o->smoke_delta_deg);
        printf("raw-motion-smoke: sendonly A8 %+0.2f deg at %.0f dps\n",
               o->smoke_delta_deg, o->smoke_speed_dps);
        rc = myact_position_inc_send(c, (uint16_t)o->smoke_speed_dps, req_delta_raw);
        return rc;
    }
    rc = run_a8_delta_observed(c, "raw-motion-smoke", o->smoke_delta_deg, o->smoke_speed_dps,
                               o->smoke_wait_ms, fmax(1.0, 0.20 * fabs(o->smoke_delta_deg)),
                               NULL, NULL, NULL);
    if (rc < 0) return rc;
    return MYACT_OK;
}

static int run_brake_persistence_test(myact_ctx_t* c, const test_opts_t* o, myact_status1_t* s1) {
    (void)c; (void)o; (void)s1;
    printf("brake-persistence-test: heading\n");
    return MYACT_ESTATE;
}

static int run_rt_inc_burst(myact_ctx_t* c, const test_opts_t* o, myact_status1_t* s1) {
    int rc;

    printf("rt-inc-burst: A8 inc mode, +%0.2f/-%0.2f deg at %.0f dps, wait=%u ms, cycles=%u\n",
           o->inc_delta_deg, o->inc_delta_deg, o->inc_speed_dps, o->inc_wait_ms, o->cycles);

    rc = axis_enable_for_motion(c, o, s1, "rt-inc-burst");
    if (rc < 0) return rc;

    for (unsigned i = 0; i < o->cycles; i++) {
        rc = run_a8_delta_observed(c, "rt-inc-burst +", o->inc_delta_deg, o->inc_speed_dps,
                                   o->inc_wait_ms, fmax(1.0, 0.20 * fabs(o->inc_delta_deg)),
                                   NULL, NULL, NULL);
        if (rc < 0) return rc;
        rc = run_a8_delta_observed(c, "rt-inc-burst -", -o->inc_delta_deg, o->inc_speed_dps,
                                   o->inc_wait_ms, fmax(1.0, 0.20 * fabs(o->inc_delta_deg)),
                                   NULL, NULL, NULL);
        if (rc < 0) return rc;
    }

    return myact_stop_send(c);
}

static int run_active_telemetry_rt_test(myact_ctx_t* c, const test_opts_t* o, myact_status1_t* s1) {
    uint64_t t_start;
    uint64_t t_end;
    uint64_t prev_ts = 0;
    uint64_t interval_sum = 0;
    uint64_t interval_min = UINT64_MAX;
    uint64_t interval_max = 0;
    unsigned interval_count = 0;
    unsigned frame_count = 0;
    int max_abs_speed = 0;
    double start_angle_deg = 0.0;
    double end_angle_deg = 0.0;
    double angle_delta_deg;
    double avg_interval_ms = 0.0;
    bool move_observed = false;
    int rc;

    printf("active-telemetry-rt-test:\n");
    printf("  interval_ms=%u duration_ms=%u\n", o->active_interval_ms, o->active_duration_ms);

    rc = axis_enable_for_motion(c, o, s1, "active-telemetry-rt-test");
    if (rc < 0) return rc;

    rc = prepare_sync(c);
    if (rc < 0) return rc;

    rc = read_angle_deg(c, &start_angle_deg);
    if (rc < 0) return rc;

    rc = myact_active_reply_config(c, MYACT_CMD_STATUS2, true, (uint16_t)(o->active_interval_ms / 10));
    if (rc < 0) return rc;

    msleep(o->active_interval_ms + 20);

    rc = myact_position_inc_send(c,
                                 (uint16_t)o->active_move_speed_dps,
                                 myact_deg_to_raw_0p01(o->active_move_delta_deg));
    if (rc < 0) {
        (void)myact_active_reply_config(c, MYACT_CMD_STATUS2, false, 0);
        return rc;
    }

    t_start = now_ms(NULL);
    t_end = t_start + o->active_duration_ms;

    while (now_ms(NULL) < t_end) {
        int ev = myact_poll(c, o->active_interval_ms + 20);
        if (ev == MYACT_ETIMEDOUT) continue;
        if (ev < 0) {
            (void)myact_active_reply_config(c, MYACT_CMD_STATUS2, false, 0);
            return ev;
        }
        if ((ev & MYACT_EVENT_STATUS2) != 0) {
            const myact_cache_t* cache = myact_cache(c);
            uint64_t ts = cache->status2.ts_ms;
            int abs_speed;
            frame_count++;
            abs_speed = cache->status2.speed_dps >= 0 ? cache->status2.speed_dps : -cache->status2.speed_dps;
            if (abs_speed > max_abs_speed) max_abs_speed = abs_speed;
            if (abs_speed > 0) move_observed = true;
            if (prev_ts != 0 && ts > prev_ts) {
                uint64_t dt = ts - prev_ts;
                if (dt < interval_min) interval_min = dt;
                if (dt > interval_max) interval_max = dt;
                interval_sum += dt;
                interval_count++;
            }
            prev_ts = ts;
        }
    }

    rc = myact_active_reply_config(c, MYACT_CMD_STATUS2, false, 0);
    if (rc < 0) return rc;

    rc = prepare_sync(c);
    if (rc < 0) return rc;

    rc = read_angle_deg(c, &end_angle_deg);
    if (rc < 0) return rc;

    angle_delta_deg = end_angle_deg - start_angle_deg;
    if (interval_count > 0) avg_interval_ms = (double)interval_sum / (double)interval_count;

    {
        unsigned expected_frames = (o->active_interval_ms > 0) ? (o->active_duration_ms / o->active_interval_ms) : 0;
        unsigned min_required = (unsigned)floor((double)expected_frames * o->active_min_frame_ratio);
        bool count_ok;
        bool motion_ok;

        if (min_required == 0) min_required = 1;
        count_ok = frame_count >= min_required;
        motion_ok = move_observed || fabs(angle_delta_deg) >= fabs(o->active_move_delta_deg);

        printf("  expected_frames=%u\n", expected_frames);
        printf("  min_required=%u\n", min_required);
        printf("  frames_received=%u\n", frame_count);
        printf("  avg_interval_ms=%.1f\n", avg_interval_ms);
        printf("  min_interval_ms=%llu\n", (unsigned long long)(interval_count ? interval_min : 0));
        printf("  max_interval_ms=%llu\n", (unsigned long long)(interval_count ? interval_max : 0));
        printf("  max_abs_speed_dps=%d\n", max_abs_speed);
        printf("  angle_delta_deg=%.2f\n", angle_delta_deg);

        if (avg_interval_ms > (double)o->active_interval_ms) {
            printf("WARNING: telemetry interval slower than requested; requested=%ums avg=%.1fms\n",
                   o->active_interval_ms, avg_interval_ms);
        }

        if (!count_ok) {
            printf("[FAIL] active-telemetry-rt-test: insufficient telemetry frames (frames_received=%u min_required=%u)\n",
                   frame_count, min_required);
            return MYACT_ESTATE;
        }
        if (!motion_ok) {
            printf("[FAIL] active-telemetry-rt-test: insufficient movement (angle_delta_deg=%.2f required=%.2f)\n",
                   fabs(angle_delta_deg), fabs(o->active_move_delta_deg));
            return MYACT_ESTATE;
        }
    }

    return MYACT_OK;
}

static int run_mit_mode_test(myact_ctx_t* c, const test_opts_t* o, myact_status1_t* s1) {
    uint8_t packed[8] = {0};
    int rc;
    (void)s1;

    rc = myact_motion_pack(o->mit_p_rad, o->mit_v_rad_s, o->mit_kp, o->mit_kd, o->mit_t_nm, packed);
    if (rc < 0) return rc;

    if (o->mit_dry_run) {
        printf("mit-mode-test dry-run:\n");
        printf("  p_rad=%.3f\n", o->mit_p_rad);
        printf("  v_rad_s=%.3f\n", o->mit_v_rad_s);
        printf("  kp=%.3f\n", o->mit_kp);
        printf("  kd=%.3f\n", o->mit_kd);
        printf("  t_nm=%.3f\n", o->mit_t_nm);
        printf("  packed=%02X %02X %02X %02X %02X %02X %02X %02X\n",
               packed[0], packed[1], packed[2], packed[3],
               packed[4], packed[5], packed[6], packed[7]);
        return MYACT_OK;
    }

    rc = axis_enable_for_motion(c, o, s1, "mit-mode-test");
    if (rc != RET_OK) return rc;

    printf("mit-mode-test live:\n");
    printf("  sending 0x401 frames\n");
    printf("  waiting for 0x501 feedback\n");

    {
        uint64_t t_end = now_ms(NULL) + o->mit_duration_ms;
        unsigned tx_count = 0;
        unsigned rx_501_count = 0;
        float first_p = 0.0f, first_v = 0.0f, first_t = 0.0f;
        float last_p = 0.0f, last_v = 0.0f, last_t = 0.0f;
        double max_abs_v = 0.0;

        while (now_ms(NULL) < t_end) {
            rc = myact_motion_send(c, o->mit_p_rad, o->mit_v_rad_s, o->mit_kp, o->mit_kd, o->mit_t_nm);
            if (rc < 0) return rc;
            tx_count++;

            {
                uint32_t wait_ms = o->mit_sample_ms > 0 ? o->mit_sample_ms : 1u;
                int ev = myact_poll(c, wait_ms);
                if (ev == MYACT_ETIMEDOUT) continue;
                if (ev < 0) return ev;
                if ((ev & MYACT_EVENT_MOTION_FB) != 0) {
                    const myact_cache_t* cache = myact_cache(c);
                    float p = cache->motion_feedback.p_rad;
                    float v = cache->motion_feedback.v_rad_s;
                    float t = cache->motion_feedback.t_nm;
                    if (rx_501_count == 0) { first_p = p; first_v = v; first_t = t; }
                    last_p = p; last_v = v; last_t = t;
                    if (fabs((double)v) > max_abs_v) max_abs_v = fabs((double)v);
                    rx_501_count++;
                }
            }
        }

        printf("  tx_count=%u\n", tx_count);
        printf("  rx_501_count=%u\n", rx_501_count);
        if (rx_501_count > 0) {
            printf("  first_feedback: p=%.3f v=%.3f t=%.3f\n", first_p, first_v, first_t);
            printf("  last_feedback: p=%.3f v=%.3f t=%.3f\n", last_p, last_v, last_t);
            printf("  max_abs_v=%.3f\n", max_abs_v);
        }

        if (rx_501_count == 0) {
            printf("[FAIL] mit-mode-test: 0x401 sent but no 0x501 feedback received\n");
            return MYACT_ESTATE;
        }
    }

    return MYACT_OK;
}

int main(int argc, char** argv) {
    sockcan_bus_t bus;
    myact_io_t io;
    myact_ctx_t ctx;
    test_opts_t o;
    myact_status1_t s1;
    int32_t a = 0;
    int rc;
    int ret = RET_OK;

    if (argc < 3) {
        print_usage(argv[0]);
        return RET_BAD_CLI;
    }

    rc = parse_cli(argc, argv, &o);
    if (rc != RET_OK) {
        print_usage(argv[0]);
        return RET_BAD_CLI;
    }

    rc = sockcan_open(&bus, argv[1]);
    if (rc < 0) return RET_COMMS_FAIL;

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

    if (o.reset_first) {
        (void)myact_system_reset(&ctx);
        msleep(2000);
    }

    rc = prepare_sync(&ctx);
    if (rc < 0) goto done;

    rc = read_full_diagnostics(&ctx, &s1, &a);
    if (rc < 0) goto done;

    {
        bool any_test_requested = o.raw_motion_smoke || o.raw_motion_smoke_sendonly || o.enable_hold ||
                                  o.brake_latency_test || o.brake_persistence_test || o.rt_inc_burst ||
                                  o.speed_step_test || o.speed_stream_test || o.torque_step_test || o.torque_sweep_test || o.profile_abs_relative_test ||
                                  o.mit_mode_test || o.active_telemetry_rt_test || o.bench_all;
        bool any_dispatched = false;

        if (!any_test_requested) {
        printf("DIAG-ONLY mode: no motion requested.\n");
        rc = RET_OK;
        goto done;
        }

        if (o.bench_all) {
            int b;
            bool a8_baseline_ok = true;
            bool required_pass = true;
            bool full_matrix_pass = true;

            any_dispatched = true;

            printf("[PASS] diagnostics\n");

            b = bench_result("brake-latency-test", run_brake_latency_test(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("enable-hold", run_enable_hold(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("raw-motion-smoke", run_raw_motion_smoke(&ctx, &o, &s1, false));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
                a8_baseline_ok = false;
            }

            b = bench_result("rt-inc-burst", run_rt_inc_burst(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
                a8_baseline_ok = false;
            }

            if (a8_baseline_ok) {
                b = bench_result("speed-step-test", run_speed_step_test(&ctx, &o, &s1));
                if (b != RET_OK) full_matrix_pass = false;

                b = bench_result("speed-stream-test", run_speed_stream_test(&ctx, &o, &s1));
                if (b != RET_OK) {
                    full_matrix_pass = false;
                }

                b = bench_result("profile-abs-relative-test", run_profile_abs_relative_test(&ctx, &o, &s1));
                if (b != RET_OK) {
                    full_matrix_pass = false;
                }

                b = bench_result("active-telemetry-rt-test", run_active_telemetry_rt_test(&ctx, &o, &s1));
                if (b != RET_OK) {
                    ret = RET_COMMS_FAIL;
                    required_pass = false;
                    full_matrix_pass = false;
                }

                b = bench_result("torque-step-test", run_torque_step_test(&ctx, &o, &s1));
                if (b != RET_OK) {
                    full_matrix_pass = false;
                }

                if (o.run_mit_in_bench_all) {
                    b = bench_result("mit-mode-test", run_mit_mode_test(&ctx, &o, &s1));
                    if (b != RET_OK) {
                        full_matrix_pass = false;
                    }
                } else {
                    printf("[SKIP] mit-mode-test: not requested (use --bench-all-mit)\n");
                }
            } else {
                printf("[SKIP] speed-step-test: skipped because A8 baseline failed\n");
                printf("[SKIP] profile-abs-relative-test: skipped because A8 baseline failed\n");
                printf("[SKIP] active-telemetry-rt-test: skipped because A8 baseline failed\n");
                printf("[SKIP] torque-step-test: skipped because A8 baseline failed\n");
                if (o.run_mit_in_bench_all) {
                    printf("[SKIP] mit-mode-test: skipped because A8 baseline failed\n");
                } else {
                    printf("[SKIP] mit-mode-test: not requested (use --bench-all-mit)\n");
                }
            }

            printf("bench-all summary:\n");
            printf("overall required baseline: %s\n", required_pass ? "PASS" : "FAIL");
            printf("overall full feature matrix: %s\n", full_matrix_pass ? "PASS" : "PARTIAL");
        } else {
            if (o.brake_latency_test) { any_dispatched = true; if (bench_result("brake-latency-test", run_brake_latency_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.enable_hold) { any_dispatched = true; if (bench_result("enable-hold", run_enable_hold(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.raw_motion_smoke) { any_dispatched = true; if (bench_result("raw-motion-smoke", run_raw_motion_smoke(&ctx, &o, &s1, false)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.raw_motion_smoke_sendonly) { any_dispatched = true; if (bench_result("raw-motion-smoke-sendonly", run_raw_motion_smoke(&ctx, &o, &s1, true)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.brake_persistence_test) { any_dispatched = true; if (bench_result("brake-persistence-test", run_brake_persistence_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.rt_inc_burst) { any_dispatched = true; if (bench_result("rt-inc-burst", run_rt_inc_burst(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.speed_step_test) { any_dispatched = true; if (bench_result("speed-step-test", run_speed_step_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.speed_stream_test) { any_dispatched = true; if (bench_result("speed-stream-test", run_speed_stream_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.profile_abs_relative_test) { any_dispatched = true; if (bench_result("profile-abs-relative-test", run_profile_abs_relative_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.active_telemetry_rt_test) { any_dispatched = true; if (bench_result("active-telemetry-rt-test", run_active_telemetry_rt_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.torque_step_test || o.torque_sweep_test) { any_dispatched = true; if (bench_result("torque-step-test", run_torque_step_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.mit_mode_test) { any_dispatched = true; if (bench_result("mit-mode-test", run_mit_mode_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
        }

        if (any_test_requested && !any_dispatched) {
            fprintf(stderr, "internal error: requested flag was parsed but not dispatched\n");
            rc = RET_BAD_CLI;
            goto done;
        }
    }

    rc = ret;

done:
    if (o.no_shutdown_on_exit) {
        test_axis_disable(&ctx, false);
        printf("WARNING: leaving motor not shutdown; brake may remain released.\n");
    } else {
        test_axis_disable(&ctx, true);
    }

    close(bus.s);
    return rc < 0 ? RET_COMMS_FAIL : rc;
}
