/*
# Build
gcc -O2 -Wall -Wextra -std=c11 -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L \
  dev/src/x8_25.c dev/tests/x8_25_test.c \
  -Idev/include -lm -o build/x8_25_test_can

# diag only
./build/x8_25_test_can can0 1

# brake latency
./build/x8_25_test_can can0 1 \
  --brake-latency-test \
  --brake-release-timeout-ms 2000 \
  --brake-poll-ms 50

# enable-hold
./build/x8_25_test_can can0 1 \
  --enable-hold \
  --runmode-timeout-ms 1500 \
  --runmode-poll-ms 50

# A2 speed
./build/x8_25_test_can can0 1 \
  --speed-step-test \
  --speed-step-dps 60 \
  --speed-step-hold-ms 1500 \
  --speed-step-cycles 2 \
  --speed-sample-ms 100 \
  --speed-pass-min-dps 5.0 \
  --speed-pass-min-delta-deg 1.0

# A4 native absolute
./build/x8_25_test_can can0 1 \
  --native-a4-abs-test \
  --abs-delta-deg 10 \
  --abs-speed-dps 60 \
  --abs-timeout-ms 3000 \
  --abs-poll-ms 100 \
  --abs-tolerance-deg 1.0

# A8 relative
./build/x8_25_test_can can0 1 \
  --rt-inc-burst \
  --cycles 2 \
  --inc-delta-deg 10 \
  --inc-speed-dps 60 \
  --inc-wait-ms 1500

# torque mode
./build/x8_25_test_can can0 1 \
  --torque-step-test \
  --torque-step-a 0.30 \
  --torque-step-hold-ms 500 \
  --torque-step-cycles 1 \
  --torque-sample-ms 100 \
  --torque-pass-min-iq-a 0.10 \
  --torque-pass-min-delta-deg 0.5

# MIT dry
./build/x8_25_test_can can0 1 \
  --mit-mode-test \
  --mit-dry-run \
  --mit-duration-ms 1000 \
  --mit-sample-ms 50

# MIT live
./build/x8_25_test_can can0 1 \
  --mit-mode-test \
  --mit-p-rad 0.10 \
  --mit-v-rad-s 0.00 \
  --mit-kp 5.00 \
  --mit-kd 0.50 \
  --mit-t-nm 0.20 \
  --mit-duration-ms 1000 \
  --mit-sample-ms 50

# active telemetry stream
./build/x8_25_test_can can0 1 \
  --active-telemetry-rt-test \
  --active-interval-ms 50 \
  --active-duration-ms 3000 \
  --active-move-delta-deg 10 \
  --active-move-speed-dps 60 \
  --active-min-frame-ratio 0.70

# combined run (single-file coverage path)
./build/x8_25_test_can can0 1 \
  --brake-latency-test \
  --enable-hold \
  --speed-step-test \
  --native-a4-abs-test \
  --rt-inc-burst \
  --torque-step-test \
  --mit-mode-test \
  --mit-dry-run \
  --active-telemetry-rt-test \
  --speed-step-dps 60 \
  --abs-delta-deg 10 \
  --inc-delta-deg 10 \
  --torque-step-a 0.30 \
  --abs-timeout-ms 3000 \
  --active-interval-ms 50 \
  --runmode-timeout-ms 1500 \
  --speed-pass-min-dps 5.0 \
  --abs-tolerance-deg 1.0
*/

#include "x8_25.h"
#include "x8_25_commissioning.h"

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
    bool raw_a1_smoke;
    bool raw_a2_smoke;
    bool raw_a4_smoke;
    bool raw_a8_smoke;
    bool raw_smoke_all;
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
    bool native_a4_abs_test;
    bool mit_mode_test;
    bool active_telemetry_rt_test;
    bool production_qual;
    bool experimental_all;
    bool run_mit_in_experimental_all;
    bool allow_hold_with_runmode0;
    bool commissioning_write_enable;
    bool write_pid_ram;
    bool write_pid_rom;
    bool set_speed_accel_valid;
    bool set_speed_decel_valid;
    bool set_position_accel_valid;
    bool set_position_decel_valid;
    bool set_speed_kp_valid;
    bool set_speed_ki_valid;
    bool set_position_kp_valid;
    bool set_position_ki_valid;
    bool set_position_kd_valid;
    bool set_positive_limit_deg_valid;
    bool set_negative_limit_deg_valid;
    bool selftest_pack;
    bool trace_frames;

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
    int set_speed_accel_dps2;
    int set_speed_decel_dps2;
    int set_position_accel_dps2;
    int set_position_decel_dps2;
    float set_speed_kp;
    float set_speed_ki;
    float set_position_kp;
    float set_position_ki;
    float set_position_kd;
    double set_positive_limit_deg;
    double set_negative_limit_deg;
} test_opts_t;


typedef struct {
    bool enabled;
    uint8_t expected_reply_opcode;
} trace_state_t;

static trace_state_t g_trace = {false, 0};
static uint8_t expected_reply_opcode_for_tx(uint8_t op);
static void trace_frame_line(const char* dir, const myact_can_frame_t* fr, uint8_t expected_reply);

static uint64_t now_ms(void* user) {
    struct timespec ts;
    (void)user;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static void msleep(unsigned ms) { usleep(ms * 1000u); }

static bool pid_near_zero(float a, float b, float c) {
    return fabs((double)a) < 1e-6 && fabs((double)b) < 1e-6 && fabs((double)c) < 1e-6;
}

static void print_motion_commissioning_snapshot(const char* tag,
                                                int rc,
                                                const myact_motion_commissioning_snapshot_t* snap) {
    if (rc < 0 || !snap) {
        printf("%s: PARAM_READ_FAILED rc=%d (%s)\n", tag, rc, myact_strerror(rc));
        return;
    }

    printf("%s: commissioning snapshot\n", tag);
    printf("  speed_pid kp=%.6f ki=%.6f\n", (double)snap->speed_kp, (double)snap->speed_ki);
    printf("  position_pid kp=%.6f ki=%.6f kd=%.6f\n", (double)snap->position_kp, (double)snap->position_ki, (double)snap->position_kd);
    printf("  accel pos_acc=%ld pos_dec=%ld speed_acc=%ld speed_dec=%ld dps2\n",
           (long)snap->pos_acc_dps2, (long)snap->pos_dec_dps2, (long)snap->speed_acc_dps2, (long)snap->speed_dec_dps2);
    printf("  status1 brake=%u bus=%.1fV error=0x%04X runmode=%u\n",
           (unsigned)snap->status1.brake_release_command_state,
           snap->status1.voltage_0p1v / 10.0,
           snap->status1.error_flags,
           (unsigned)snap->runmode);
    if (fabs((double)snap->speed_kp) < 1e-6 && fabs((double)snap->speed_ki) < 1e-6) {
        printf("WARN: speed loop PID appears zero; native A2 may ACK but produce no torque.\n");
    }
    if (pid_near_zero(snap->position_kp, snap->position_ki, snap->position_kd)) {
        printf("WARN: position loop PID appears zero; native A4 may ACK but produce no torque.\n");
    }
    if (snap->pos_acc_dps2 == 0 || snap->pos_dec_dps2 == 0 || snap->speed_acc_dps2 == 0 || snap->speed_dec_dps2 == 0) {
        printf("INFO: zero acceleration means direct/current-limited behavior, not necessarily a command-frame error.\n");
    }
}


static int sockcan_send_cb(const myact_can_frame_t* fr, void* user) {
    sockcan_bus_t* b = (sockcan_bus_t*)user;
    struct can_frame c = {0};

    if (!fr || !b || fr->dlc > 8) return MYACT_EINVAL;

    c.can_id = fr->id & CAN_SFF_MASK;
    c.can_dlc = fr->dlc;
    memcpy(c.data, fr->data, 8);
    if (g_trace.enabled) {
        g_trace.expected_reply_opcode = (fr->dlc > 0) ? expected_reply_opcode_for_tx(fr->data[0]) : 0;
        trace_frame_line("TX", fr, g_trace.expected_reply_opcode);
    }
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
    trace_frame_line("RX", out, g_trace.expected_reply_opcode);
    g_trace.expected_reply_opcode = 0;
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
    int rc = myact_active_reply_disable_all(ctx);
    if (rc < 0) return rc;

    rc = myact_drain_rx(ctx, 300);
    if (rc < 0) return rc;

    return myact_wait_quiet(ctx, 300, 1500);
}

static void test_axis_disable(myact_ctx_t* ctx, bool shutdown) {
    (void)myact_active_reply_disable_all(ctx);
    (void)myact_drain_rx(ctx, 500);
    (void)myact_stop_send(ctx);
    if (shutdown) {
        msleep(100);
        (void)myact_shutdown_send(ctx);
    }
}

static void print_usage(const char* p) { fprintf(stderr, "usage: %s <can_if> <node_id> [flags...]\n", p); }

static const char* opcode_name(uint8_t op) {
    switch (op) {
        case 0x19: return "shutdown";
        case 0x76: return "stop";
        case 0x77: return "brake-release";
        case 0x80: return "read-multi-angle";
        case 0x88: return "read-status1";
        case 0x9A: return "read-status2";
        case 0x9C: return "read-status3";
        case 0xA2: return "speed-cmd";
        case 0xA4: return "abs-pos-cmd";
        case 0xA8: return "inc-pos-cmd";
        default: return "unknown";
    }
}

static uint8_t expected_reply_opcode_for_tx(uint8_t op) {
    switch (op) {
        case 0x19: case 0x76: case 0x77:
        case 0x80: case 0x88: case 0x9A: case 0x9C:
        case 0xA2: case 0xA4: case 0xA8:
            return op;
        default:
            return 0;
    }
}

static void trace_frame_line(const char* dir, const myact_can_frame_t* fr, uint8_t expected_reply) {
    size_t i;
    struct timespec ts;
    if (!g_trace.enabled || !fr) return;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    printf("[%lld.%09ld] %s id=%03X dlc=%u data=",
           (long long)ts.tv_sec, ts.tv_nsec, dir, fr->id & CAN_SFF_MASK, (unsigned)fr->dlc);
    for (i = 0; i < fr->dlc; ++i) printf("%s%02X", (i == 0) ? "" : " ", fr->data[i]);
    if (fr->dlc == 0) printf("--");
    if (fr->dlc > 0) printf(" opcode=0x%02X(%s)", fr->data[0], opcode_name(fr->data[0]));
    else printf(" opcode=--");
    if (expected_reply) printf(" expected=0x%02X(%s)", expected_reply, opcode_name(expected_reply));
    else printf(" expected=--");
    printf("\n");
}


static bool commissioning_set_requested(const test_opts_t* o) {
    return o->set_speed_accel_valid || o->set_speed_decel_valid ||
           o->set_position_accel_valid || o->set_position_decel_valid ||
           o->set_speed_kp_valid || o->set_speed_ki_valid ||
           o->set_position_kp_valid || o->set_position_ki_valid || o->set_position_kd_valid ||
           o->set_positive_limit_deg_valid || o->set_negative_limit_deg_valid;
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
        FLAG("--raw-a1-smoke", raw_a1_smoke)
        FLAG("--raw-a2-smoke", raw_a2_smoke)
        FLAG("--raw-a4-smoke", raw_a4_smoke)
        FLAG("--raw-a8-smoke", raw_a8_smoke)
        FLAG("--raw-smoke-all", raw_smoke_all)
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
        FLAG("--native-a4-abs-test", native_a4_abs_test)
        FLAG("--mit-mode-test", mit_mode_test)
        FLAG("--active-telemetry-rt-test", active_telemetry_rt_test)
        FLAG("--production-qual", production_qual)
        FLAG("--bench-all", production_qual)
        FLAG("--experimental-all", experimental_all)
        FLAG("--bench-all-mit", run_mit_in_experimental_all)
        FLAG("--mit-dry-run", mit_dry_run)
        FLAG("--allow-hold-with-runmode0", allow_hold_with_runmode0)
        FLAG("--commissioning-write-enable", commissioning_write_enable)
        FLAG("--write-pid-ram", write_pid_ram)
        FLAG("--write-pid-rom", write_pid_rom)
        FLAG("--selftest-pack", selftest_pack)
        FLAG("--trace-frames", trace_frames)

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
        else if (!strcmp(argv[i], "--brake-release-timeout-ms") && i + 1 < argc) o->brake_release_timeout_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--brake-poll-ms") && i + 1 < argc) o->brake_poll_ms = (unsigned)atoi(argv[++i]);
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
        else if (!strcmp(argv[i], "--set-speed-accel-dps2") && i + 1 < argc) { o->set_speed_accel_dps2 = atoi(argv[++i]); o->set_speed_accel_valid = true; }
        else if (!strcmp(argv[i], "--set-speed-decel-dps2") && i + 1 < argc) { o->set_speed_decel_dps2 = atoi(argv[++i]); o->set_speed_decel_valid = true; }
        else if (!strcmp(argv[i], "--set-position-accel-dps2") && i + 1 < argc) { o->set_position_accel_dps2 = atoi(argv[++i]); o->set_position_accel_valid = true; }
        else if (!strcmp(argv[i], "--set-position-decel-dps2") && i + 1 < argc) { o->set_position_decel_dps2 = atoi(argv[++i]); o->set_position_decel_valid = true; }
        else if (!strcmp(argv[i], "--set-speed-kp") && i + 1 < argc) { o->set_speed_kp = (float)atof(argv[++i]); o->set_speed_kp_valid = true; }
        else if (!strcmp(argv[i], "--set-speed-ki") && i + 1 < argc) { o->set_speed_ki = (float)atof(argv[++i]); o->set_speed_ki_valid = true; }
        else if (!strcmp(argv[i], "--set-position-kp") && i + 1 < argc) { o->set_position_kp = (float)atof(argv[++i]); o->set_position_kp_valid = true; }
        else if (!strcmp(argv[i], "--set-position-ki") && i + 1 < argc) { o->set_position_ki = (float)atof(argv[++i]); o->set_position_ki_valid = true; }
        else if (!strcmp(argv[i], "--set-position-kd") && i + 1 < argc) { o->set_position_kd = (float)atof(argv[++i]); o->set_position_kd_valid = true; }
        else if (!strcmp(argv[i], "--set-positive-limit-deg") && i + 1 < argc) { o->set_positive_limit_deg = atof(argv[++i]); o->set_positive_limit_deg_valid = true; }
        else if (!strcmp(argv[i], "--set-negative-limit-deg") && i + 1 < argc) { o->set_negative_limit_deg = atof(argv[++i]); o->set_negative_limit_deg_valid = true; }
        else return RET_BAD_CLI;
#undef FLAG
    }

    if (commissioning_set_requested(o) && !o->commissioning_write_enable) {
        fprintf(stderr, "Refusing commissioning parameter writes: --commissioning-write-enable is required.\n");
        return RET_BAD_CLI;
    }
    if (o->set_speed_accel_valid && (o->set_speed_accel_dps2 < 100 || o->set_speed_accel_dps2 > 60000)) return RET_BAD_CLI;
    if (o->set_speed_decel_valid && (o->set_speed_decel_dps2 < 100 || o->set_speed_decel_dps2 > 60000)) return RET_BAD_CLI;
    if (o->set_position_accel_valid && (o->set_position_accel_dps2 < 100 || o->set_position_accel_dps2 > 60000)) return RET_BAD_CLI;
    if (o->set_position_decel_valid && (o->set_position_decel_dps2 < 100 || o->set_position_decel_dps2 > 60000)) return RET_BAD_CLI;
    if (o->write_pid_ram && o->write_pid_rom) return RET_BAD_CLI;

    return RET_OK;
}

static int commissioning_write_float(myact_ctx_t* c, const char* label, myact_pid_index_t index, float target, bool write_rom) {
    float old_v = 0.0f, new_v = 0.0f;
    int rc = myact_pid_read(c, index, &old_v);
    if (rc < 0) return rc;
    rc = write_rom ? myact_pid_write_rom(c, index, target) : myact_pid_write_ram(c, index, target);
    if (rc < 0) return rc;
    rc = myact_pid_read(c, index, &new_v);
    if (rc < 0) return rc;
    printf("%s old=%.6f new=%.6f\n", label, (double)old_v, (double)new_v);
    return MYACT_OK;
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
    bool low_voltage_condition = v < 40.0;
    bool is_small_forced_test = !strcmp(n, "speed-step-test") || !strcmp(n, "native-a4-abs-test");

    if (v < o->min_motion_voltage && !o->force_motion_low_voltage) {
        printf("%s: SKIP bus voltage %.1f V below configured minimum %.1f V\n", n, v, o->min_motion_voltage);
        return MYACT_ESTATE;
    }
    if (v < o->min_motion_voltage) {
        printf("WARNING: low-voltage motion forced at %.1f V; use rated supply for real torque/speed characterization.\n", v);
        printf("rated voltage is 48 V-class; current bench voltage is low.\n");
    }
    if (low_voltage_condition) {
        if (o->force_motion_low_voltage) {
            printf("LOW_VOLTAGE_CONDITION: protocol can be tested, but torque/speed performance is not valid.\n");
            printf("LOW_VOLTAGE_CONDITION: repeat native A2/A4 validation at rated 48 V before final capability decision.\n");
            if (!is_small_forced_test) {
                printf("%s: SKIP under low voltage forced mode; only small A2/A4 validation tests are allowed below 40.0 V\n", n);
                return MYACT_ESTATE;
            }
        } else {
            printf("%s: SKIP bus voltage %.1f V is below 40.0 V low-voltage protocol-only threshold (use --force-motion-low-voltage for limited tests)\n", n, v);
            return MYACT_ESTATE;
        }
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

static int send_stop_and_sample(myact_ctx_t* c, const char* n) {
    int rc = myact_speed_send(c, 0);
    if (rc < 0) return rc;
    msleep(50);
    rc = myact_stop_send(c);
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
    if (rc == MYACT_ENOTSUP) {
        printf("[ENOTSUP] %s: %s (%d)\n", n, myact_strerror(rc), rc);
        return RET_OK;
    }
    if (rc == MYACT_ENOMOTION && (!strcmp(n, "speed-step-test") || !strcmp(n, "speed-stream-test") || !strcmp(n, "torque-step-test") || !strcmp(n, "native-a4-abs-test"))) {
        printf("[ACK_NO_MOTION] %s: command ACK observed but actuator produced no measurable motion\n", n);
        if (!strcmp(n, "speed-step-test") || !strcmp(n, "native-a4-abs-test")) {
            printf("classification hint: protocol/frame failure vs runmode failure vs no-motion-under-low-voltage vs parameter/configuration suspect\n");
            printf("note: do not permanently mark A2/A4 unsupported based only on 22.7 V tests.\n");
            printf("recommendation: repeat native A2/A4 validation at rated 48 V before final capability decision.\n");
        }
        printf("[ENOMOTION] %s: %s (%d)\n", n, myact_strerror(rc), rc);
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
    double pos_angle_start = 0.0;
    double pos_angle_end = 0.0;
    double neg_angle_start = 0.0;
    double neg_angle_end = 0.0;
    double delta_deg;
    double pos_delta_deg = 0.0;
    double neg_delta_deg = 0.0;
    double max_abs_speed_dps = 0.0;
    double max_abs_iq_a = 0.0;
    int ret = MYACT_OK;
    int rc;
    myact_status2_t s2;
    myact_motion_commissioning_snapshot_t snap;
    int snap_rc;
    myact_runmode_t runmode_before = MYACT_RUNMODE_UNKNOWN;
    myact_runmode_t runmode_after_command = MYACT_RUNMODE_UNKNOWN;
    myact_runmode_t runmode_after_stop = MYACT_RUNMODE_UNKNOWN;
    bool low_voltage_condition = false;
    bool pos_sign_ok = false;
    bool neg_sign_ok = false;
    bool must_stop = false;

    printf("speed-step-test: native A2 speed mode, %+0.1f dps / %0.1f dps, hold=%u ms, cycles=%u\n",
           o->speed_step_dps, -o->speed_step_dps, o->speed_step_hold_ms, o->speed_step_cycles);

    rc = prepare_sync(c);
    if (rc < 0) return rc;

    snap_rc = myact_read_motion_commissioning_snapshot(c, &snap);
    print_motion_commissioning_snapshot("speed-step-test", snap_rc, &snap);

    rc = myact_read_status1(c, s1);
    if (rc < 0) return rc;

    if (!s1->brake_release_command_state) {
        printf("speed-step-test: brake locked, sending 0x77 release command\n");
        rc = myact_brake_release(c);
        if (rc < 0) return rc;
        t0 = now_ms(NULL);
        while (now_ms(NULL) - t0 < o->brake_release_timeout_ms) {
            rc = myact_read_status1(c, s1);
            if (rc < 0) return rc;
            if (s1->brake_release_command_state) break;
            msleep(o->brake_poll_ms);
        }
        if (!s1->brake_release_command_state) {
            printf("[FAIL] speed-step-test: brake release command state did not reach 1 before timeout\n");
            return MYACT_ESTATE;
        }
    }

    rc = check_motion_voltage_or_skip(o, s1, "speed-step-test");
    if (rc < 0) return rc;
    low_voltage_condition = (s1->voltage_0p1v / 10.0) < 40.0;

    rc = read_angle_deg(c, &angle_before);
    if (rc < 0) return rc;

    for (unsigned i = 0; i < o->speed_step_cycles; i++) {
        rc = read_angle_deg(c, &pos_angle_start);
        if (rc < 0) { ret = rc; goto cleanup; }

        printf("speed-step-test cycle %u: sending A2 +speed\n", i + 1);
        rc = myact_speed_dps_send(c, o->speed_step_dps);
        if (rc < 0) { ret = rc; goto cleanup; }
        must_stop = true;

        (void)myact_read_operating_mode(c, &runmode_before);
        (void)myact_wait_runmode(c, MYACT_RUNMODE_SPEED, o->runmode_timeout_ms, o->runmode_poll_ms);
        (void)myact_read_operating_mode(c, &runmode_after_command);
        if (runmode_after_command != MYACT_RUNMODE_SPEED) {
            printf("RUNMODE_DIAGNOSTIC_MISMATCH expected=%u observed=%u\n",
                   (unsigned)MYACT_RUNMODE_SPEED, (unsigned)runmode_after_command);
        }

        t0 = now_ms(NULL);
        while (now_ms(NULL) - t0 < o->speed_step_hold_ms) {
            rc = sample_status2_capture(c, "speed+", t0, &s2);
            if (rc < 0) { ret = rc; goto cleanup; }
            if (s2.speed_dps > 0) pos_sign_ok = true;
            if (fabs((double)s2.speed_dps) > max_abs_speed_dps) max_abs_speed_dps = fabs((double)s2.speed_dps);
            if (fabs(s2.iq_0p01a / 100.0) > max_abs_iq_a) max_abs_iq_a = fabs(s2.iq_0p01a / 100.0);
            msleep(o->speed_sample_ms);
        }

        rc = myact_speed_send(c, 0);
        if (rc < 0) { ret = rc; goto cleanup; }
        msleep(50);
        rc = read_angle_deg(c, &pos_angle_end);
        if (rc < 0) { ret = rc; goto cleanup; }
        pos_delta_deg += pos_angle_end - pos_angle_start;

        rc = read_angle_deg(c, &neg_angle_start);
        if (rc < 0) { ret = rc; goto cleanup; }

        printf("speed-step-test cycle %u: sending A2 -speed\n", i + 1);
        rc = myact_speed_dps_send(c, -o->speed_step_dps);
        if (rc < 0) { ret = rc; goto cleanup; }

        (void)myact_wait_runmode(c, MYACT_RUNMODE_SPEED, o->runmode_timeout_ms, o->runmode_poll_ms);
        (void)myact_read_operating_mode(c, &runmode_after_command);
        if (runmode_after_command != MYACT_RUNMODE_SPEED) {
            printf("RUNMODE_DIAGNOSTIC_MISMATCH expected=%u observed=%u\n",
                   (unsigned)MYACT_RUNMODE_SPEED, (unsigned)runmode_after_command);
        }

        t0 = now_ms(NULL);
        while (now_ms(NULL) - t0 < o->speed_step_hold_ms) {
            rc = sample_status2_capture(c, "speed-", t0, &s2);
            if (rc < 0) { ret = rc; goto cleanup; }
            if (s2.speed_dps < 0) neg_sign_ok = true;
            if (fabs((double)s2.speed_dps) > max_abs_speed_dps) max_abs_speed_dps = fabs((double)s2.speed_dps);
            if (fabs(s2.iq_0p01a / 100.0) > max_abs_iq_a) max_abs_iq_a = fabs(s2.iq_0p01a / 100.0);
            msleep(o->speed_sample_ms);
        }

        rc = myact_speed_send(c, 0);
        if (rc < 0) { ret = rc; goto cleanup; }
        msleep(50);
        rc = read_angle_deg(c, &neg_angle_end);
        if (rc < 0) { ret = rc; goto cleanup; }
        neg_delta_deg += neg_angle_end - neg_angle_start;
    }

    rc = read_angle_deg(c, &angle_after);
    if (rc < 0) { ret = rc; goto cleanup; }

    delta_deg = angle_after - angle_before;
    printf("speed-step-test summary:\n");
    printf("  max_abs_speed_dps=%.1f (pass>=%.1f)\n", max_abs_speed_dps, o->speed_pass_min_dps);
    printf("  max_abs_iq_A=%.2f\n", max_abs_iq_a);
    printf("  angle_before=%.2f angle_after=%.2f delta_deg=%.2f\n", angle_before, angle_after, delta_deg);
    printf("  pos_delta_deg=%.2f neg_delta_deg=%.2f\n", pos_delta_deg, neg_delta_deg);
    printf("  runmode_before=%u runmode_after_command=%u runmode_after_stop=%u error_flags=0x%04X\n",
           (unsigned)runmode_before, (unsigned)runmode_after_command, (unsigned)runmode_after_stop, s1->error_flags);

    rc = myact_read_status1(c, s1);
    if (rc < 0) { ret = rc; goto cleanup; }
    if (s1->error_flags != 0) {
        printf("[FAIL] speed-step-test: error_flags=0x%04X\n", s1->error_flags);
        ret = MYACT_ESTATE;
        goto cleanup;
    }

    if (max_abs_speed_dps < o->speed_pass_min_dps) {
        if (low_voltage_condition) {
            printf("[NO_MOTION_UNDER_LOW_VOLTAGE] speed-step-test: protocol/runmode observed, but low-voltage bench produced insufficient motion\n");
            printf("classification: no-motion-under-low-voltage\n");
        } else {
            printf("[PARAMETER_OR_CONFIGURATION_SUSPECT] speed-step-test: runmode=0x02 but measured speed stayed below threshold\n");
            printf("classification: parameter/configuration suspect\n");
        }
        if (max_abs_iq_a < o->torque_pass_min_iq_a) {
            printf("Hint: current remained very low (max_abs_iq_A=%.2f). Check max torque/current limit and speed PID/acceleration.\n", max_abs_iq_a);
        }
        ret = MYACT_ENOMOTION;
        goto cleanup;
    }

    if (!(pos_sign_ok && neg_sign_ok && pos_delta_deg > 0.0 && neg_delta_deg < 0.0)) {
        printf("[FAIL] speed-step-test: direction/sign checks failed (pos_sign=%d neg_sign=%d pos_delta=%.2f neg_delta=%.2f)\n",
               pos_sign_ok ? 1 : 0, neg_sign_ok ? 1 : 0, pos_delta_deg, neg_delta_deg);
        ret = MYACT_ENOMOTION;
        goto cleanup;
    }

    ret = MYACT_OK;

cleanup:
    if (must_stop) {
        int stop_rc = send_stop_and_sample(c, "speed-step-test");
        (void)myact_read_operating_mode(c, &runmode_after_stop);
        if (stop_rc < 0 && ret == MYACT_OK) ret = stop_rc;
    }
    return ret;
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
    myact_runmode_t runmode_before = MYACT_RUNMODE_UNKNOWN;
    myact_runmode_t runmode_after_command = MYACT_RUNMODE_UNKNOWN;
    myact_runmode_t runmode_after_stop = MYACT_RUNMODE_UNKNOWN;

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
        (void)myact_read_operating_mode(c, &runmode_before);
        (void)myact_wait_runmode(c, MYACT_RUNMODE_CURRENT, o->runmode_timeout_ms, o->runmode_poll_ms);
        (void)myact_read_operating_mode(c, &runmode_after_command);
        if (runmode_after_command != MYACT_RUNMODE_CURRENT) {
            printf("RUNMODE_DIAGNOSTIC_MISMATCH expected=%u observed=%u\n",
                   (unsigned)MYACT_RUNMODE_CURRENT, (unsigned)runmode_after_command);
        }

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
        (void)myact_wait_runmode(c, MYACT_RUNMODE_CURRENT, o->runmode_timeout_ms, o->runmode_poll_ms);
        (void)myact_read_operating_mode(c, &runmode_after_command);
        if (runmode_after_command != MYACT_RUNMODE_CURRENT) {
            printf("RUNMODE_DIAGNOSTIC_MISMATCH expected=%u observed=%u\n",
                   (unsigned)MYACT_RUNMODE_CURRENT, (unsigned)runmode_after_command);
        }

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

    (void)myact_read_operating_mode(c, &runmode_after_stop);
    angle_delta_deg = angle_after - angle_before;
    printf("torque-step-test summary:\n");
    printf("  requested_iq_a=%.2f\n", o->torque_step_a);
    printf("  max_abs_iq_A=%.2f\n", max_abs_iq_a);
    printf("  max_abs_speed_dps=%.1f\n", max_abs_speed_dps);
    printf("  angle_before=%.2f\n", angle_before);
    printf("  angle_after=%.2f\n", angle_after);
    printf("  angle_delta_deg=%.2f\n", angle_delta_deg);
    printf("  runmode_before=%u runmode_after_command=%u runmode_after_stop=%u\n",
           (unsigned)runmode_before, (unsigned)runmode_after_command, (unsigned)runmode_after_stop);

    if (max_abs_iq_a >= o->torque_pass_min_iq_a || fabs(angle_delta_deg) >= o->torque_pass_min_delta_deg) {
        return MYACT_OK;
    }

    if (acked) {
        printf("[FAIL] torque-step-test: A1 command acknowledged but current loop not responding in this runtime mode\n");
    }

    return MYACT_ENOMOTION;
}

static int run_native_a4_abs_test(myact_ctx_t* c, const test_opts_t* o, myact_status1_t* s1) {
    double orig;
    double target_abs;
    double final_abs = 0.0;
    double max_abs_iq_a = 0.0;
    double max_abs_speed_dps = 0.0;
    int32_t target_raw;
    int32_t pos_acc = 0, pos_dec = 0;
    float pos_kp = 0.0f, pos_ki = 0.0f, pos_kd = 0.0f;
    uint16_t abs_speed_raw;
    uint8_t a4_frame[8] = {0};
    int rc;
    myact_runmode_t rm = MYACT_RUNMODE_UNKNOWN;
    myact_motion_commissioning_snapshot_t snap;
    int snap_rc;
    uint64_t t0;

    rc = prepare_sync(c);
    if (rc < 0) return rc;

    rc = axis_enable_for_motion(c, o, s1, "native-a4-abs-test");
    if (rc < 0) return rc;

    rc = read_angle_deg(c, &orig);
    if (rc < 0) return rc;

    target_abs = orig + o->abs_delta_deg;
    target_raw = myact_deg_to_raw_0p01(target_abs);
    abs_speed_raw = (uint16_t)o->abs_speed_dps;
    a4_frame[0] = 0xA4;
    a4_frame[1] = 0x00;
    a4_frame[2] = (uint8_t)(abs_speed_raw & 0xFF);
    a4_frame[3] = (uint8_t)((abs_speed_raw >> 8) & 0xFF);
    a4_frame[4] = (uint8_t)(target_raw & 0xFF);
    a4_frame[5] = (uint8_t)((target_raw >> 8) & 0xFF);
    a4_frame[6] = (uint8_t)((target_raw >> 16) & 0xFF);
    a4_frame[7] = (uint8_t)((target_raw >> 24) & 0xFF);

    snap_rc = myact_read_motion_commissioning_snapshot(c, &snap);
    (void)myact_accel_read(c, MYACT_ACCEL_POSITION_ACC, &pos_acc);
    (void)myact_accel_read(c, MYACT_ACCEL_POSITION_DEC, &pos_dec);
    (void)myact_pid_read(c, MYACT_PID_POSITION_KP, &pos_kp);
    (void)myact_pid_read(c, MYACT_PID_POSITION_KI, &pos_ki);
    (void)myact_pid_read(c, MYACT_PID_POSITION_KD, &pos_kd);

    printf("native-a4-abs-test: target_deg=%.2f target_raw_0p01deg=%ld maxSpeed=%u frame=%02X %02X %02X %02X %02X %02X %02X %02X\n",
           target_abs, (long)target_raw, (unsigned)abs_speed_raw,
           a4_frame[0], a4_frame[1], a4_frame[2], a4_frame[3], a4_frame[4], a4_frame[5], a4_frame[6], a4_frame[7]);
    if (o->set_positive_limit_deg_valid || o->set_negative_limit_deg_valid) {
        bool above_pos = o->set_positive_limit_deg_valid && target_abs > o->set_positive_limit_deg;
        bool below_neg = o->set_negative_limit_deg_valid && target_abs < o->set_negative_limit_deg;
        printf("native-a4-abs-test: software limits are write-only in firmware; using application-configured values only.\n");
        printf("native-a4-abs-test: configured limits positive=%s negative=%s\n",
               o->set_positive_limit_deg_valid ? "known" : "unknown",
               o->set_negative_limit_deg_valid ? "known" : "unknown");
        if (o->set_positive_limit_deg_valid) {
            printf("native-a4-abs-test: positive_limit_deg=%.2f\n", o->set_positive_limit_deg);
        }
        if (o->set_negative_limit_deg_valid) {
            printf("native-a4-abs-test: negative_limit_deg=%.2f\n", o->set_negative_limit_deg);
        }
        printf("native-a4-abs-test: target_vs_limits=%s\n", (above_pos || below_neg) ? "OUTSIDE" : "INSIDE");
    } else {
        printf("native-a4-abs-test: software limits unknown to app; skipping target-vs-limit check (firmware provides no readback).\n");
    }
    print_motion_commissioning_snapshot("native-a4-abs-test", snap_rc, &snap);
    rc = myact_position_abs_send(c, abs_speed_raw, target_raw);
    if (rc < 0) goto out;
    {
        myact_runmode_t runmode_before = MYACT_RUNMODE_UNKNOWN;
        myact_runmode_t runmode_after_command = MYACT_RUNMODE_UNKNOWN;
        myact_runmode_t runmode_after_stop = MYACT_RUNMODE_UNKNOWN;
        (void)myact_read_operating_mode(c, &runmode_before);
        (void)myact_wait_runmode(c, MYACT_RUNMODE_POSITION, o->runmode_timeout_ms, o->runmode_poll_ms);
        (void)myact_read_operating_mode(c, &runmode_after_command);
        if (runmode_after_command != MYACT_RUNMODE_POSITION) {
            printf("RUNMODE_DIAGNOSTIC_MISMATCH expected=%u observed=%u\n",
                   (unsigned)MYACT_RUNMODE_POSITION, (unsigned)runmode_after_command);
        }
        printf("native-a4-abs-test: runmode_before=%u runmode_after_command=%u runmode_after_stop=%u\n",
               (unsigned)runmode_before, (unsigned)runmode_after_command, (unsigned)runmode_after_stop);
    }

    t0 = now_ms(NULL);
    for (;;) {
        myact_status2_t s2;
        double err;
        rc = read_angle_deg(c, &final_abs);
        if (rc < 0) goto out;
        err = target_abs - final_abs;
        printf("t_ms=%llu tag=abs target_deg=%.2f actual_deg=%.2f error_deg=%.2f\n",
               (unsigned long long)(now_ms(NULL) - t0), target_abs, final_abs, err);

        if (myact_read_status2(c, &s2) == MYACT_OK) {
            double iq = fabs(s2.iq_0p01a / 100.0);
            double spd = fabs((double)s2.speed_dps);
            if (iq > max_abs_iq_a) max_abs_iq_a = iq;
            if (spd > max_abs_speed_dps) max_abs_speed_dps = spd;
            printf("t_ms=%llu tag=abs-9C iq_A=%.2f speed_dps=%d angle_deg=%d\n",
                   (unsigned long long)(now_ms(NULL) - t0), s2.iq_0p01a / 100.0, s2.speed_dps, s2.angle_deg);
        }

        if (fabs(err) <= o->abs_tolerance_deg) break;
        if (now_ms(NULL) - t0 >= o->abs_timeout_ms) {
            myact_status1_t diag_s1;
            memset(&diag_s1, 0, sizeof(diag_s1));
            (void)myact_read_status1(c, &diag_s1);
            (void)myact_read_operating_mode(c, &rm);
            printf("native-a4-abs-test: A4 timeout\n");
            printf("native-a4-abs-test: original_abs_deg=%.2f target_abs_deg=%.2f final_abs_deg=%.2f error_deg=%.2f\n",
                   orig, target_abs, final_abs, target_abs - final_abs);
            printf("native-a4-abs-test: A4 frame bytes=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                   a4_frame[0], a4_frame[1], a4_frame[2], a4_frame[3], a4_frame[4], a4_frame[5], a4_frame[6], a4_frame[7]);
            printf("native-a4-abs-test: runmode=%u status1 error_flags=0x%04X brake=%u bus=%.1fV\n",
                   (unsigned)rm, diag_s1.error_flags, (unsigned)diag_s1.brake_release_command_state, diag_s1.voltage_0p1v / 10.0);
            if ((diag_s1.voltage_0p1v / 10.0) < 40.0) {
                printf("classification: no-motion-under-low-voltage\n");
            } else {
                printf("classification: parameter/configuration suspect\n");
            }
            printf("native-a4-abs-test: max_abs_iq_A=%.2f max_abs_speed_dps=%.1f\n", max_abs_iq_a, max_abs_speed_dps);
            printf("native-a4-abs-test: position PID kp=%.6f ki=%.6f kd=%.6f\n", (double)pos_kp, (double)pos_ki, (double)pos_kd);
            printf("native-a4-abs-test: position accel/decel dps2=%ld/%ld\n", (long)pos_acc, (long)pos_dec);
            printf("diag likely_issue: check runmode, PID tuning/zero, accel/decel limits, bus voltage, current limit, position limits, or motor error flags\n");
            rc = MYACT_ENOMOTION;
            goto out;
        }
        msleep(o->abs_poll_ms);
    }

    printf("[PASS] native-a4-abs-test: native A4 absolute move succeeded\n");
    rc = MYACT_OK;
out:
    (void)myact_stop_send(c);
    (void)send_stop_and_sample(c, "native-a4-abs-test");
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

    printf("enable-hold: status1 brake=%s bus=%.1fV error=0x%04X\n",
           s1->brake_release_command_state ? "released" : "locked",
           s1->voltage_0p1v / 10.0,
           s1->error_flags);
    if (s1->error_flags != 0u) {
        printf("enable-hold: axis enable failed: error_flags=0x%04X\n", s1->error_flags);
        return MYACT_ESTATE;
    }

    rc = myact_read_operating_mode(c, &rm);
    if (rc < 0) return rc;
    if (rm == MYACT_RUNMODE_IDLE_OR_OFF) {
        printf("enable-hold: runmode=0 idle/off, no motion command active\n");
        printf("INFO: runmode stayed 0 because brake release alone does not select a control mode.\n");
    } else {
        printf("enable-hold: runmode=%u\n", (unsigned)rm);
    }

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
    {
        myact_runmode_t runmode_before = MYACT_RUNMODE_UNKNOWN;
        myact_runmode_t runmode_after_command = MYACT_RUNMODE_UNKNOWN;
        myact_runmode_t runmode_after_stop = MYACT_RUNMODE_UNKNOWN;
        (void)myact_read_operating_mode(c, &runmode_before);
        (void)myact_wait_runmode(c, MYACT_RUNMODE_POSITION, 1500u, 50u);
        (void)myact_read_operating_mode(c, &runmode_after_command);
        if (runmode_after_command != MYACT_RUNMODE_POSITION) {
            printf("RUNMODE_DIAGNOSTIC_MISMATCH expected=%u observed=%u\n",
                   (unsigned)MYACT_RUNMODE_POSITION, (unsigned)runmode_after_command);
        }
        printf("%s runmode_before=%u runmode_after_command=%u runmode_after_stop=%u\n",
               tag, (unsigned)runmode_before, (unsigned)runmode_after_command, (unsigned)runmode_after_stop);
    }
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

static int run_commissioning_writes(myact_ctx_t* c, const test_opts_t* o) {
    myact_status2_t s2;
    int rc;

    if (!commissioning_set_requested(o)) return MYACT_OK;

    rc = prepare_sync(c);
    if (rc < 0) return rc;
    rc = myact_speed_send(c, 0);
    if (rc < 0) return rc;
    rc = myact_stop_send(c);
    if (rc < 0) return rc;
    msleep(100);
    rc = myact_read_status2(c, &s2);
    if (rc < 0) return rc;
    if (abs((int)s2.speed_dps) > 2) {
        printf("Refusing commissioning write: motor is moving (speed_dps=%d)\n", s2.speed_dps);
        return MYACT_ESTATE;
    }

    if (o->set_speed_accel_valid || o->set_speed_decel_valid || o->set_position_accel_valid || o->set_position_decel_valid) {
        printf("WARNING: 0x43 writes acceleration to RAM and ROM.\n");
    }

    if (o->set_speed_accel_valid) {
        int32_t old_v = 0, new_v = 0;
        rc = myact_accel_read(c, MYACT_ACCEL_SPEED_ACC, &old_v); if (rc < 0) return rc;
        rc = myact_accel_write_rom(c, MYACT_ACCEL_SPEED_ACC, (uint32_t)o->set_speed_accel_dps2); if (rc < 0) return rc;
        rc = myact_accel_read(c, MYACT_ACCEL_SPEED_ACC, &new_v); if (rc < 0) return rc;
        printf("speed_accel_dps2 old=%ld new=%ld\n", (long)old_v, (long)new_v);
    }
    if (o->set_speed_decel_valid) {
        int32_t old_v = 0, new_v = 0;
        rc = myact_accel_read(c, MYACT_ACCEL_SPEED_DEC, &old_v); if (rc < 0) return rc;
        rc = myact_accel_write_rom(c, MYACT_ACCEL_SPEED_DEC, (uint32_t)o->set_speed_decel_dps2); if (rc < 0) return rc;
        rc = myact_accel_read(c, MYACT_ACCEL_SPEED_DEC, &new_v); if (rc < 0) return rc;
        printf("speed_decel_dps2 old=%ld new=%ld\n", (long)old_v, (long)new_v);
    }
    if (o->set_position_accel_valid) {
        int32_t old_v = 0, new_v = 0;
        rc = myact_accel_read(c, MYACT_ACCEL_POSITION_ACC, &old_v); if (rc < 0) return rc;
        rc = myact_accel_write_rom(c, MYACT_ACCEL_POSITION_ACC, (uint32_t)o->set_position_accel_dps2); if (rc < 0) return rc;
        rc = myact_accel_read(c, MYACT_ACCEL_POSITION_ACC, &new_v); if (rc < 0) return rc;
        printf("position_accel_dps2 old=%ld new=%ld\n", (long)old_v, (long)new_v);
    }
    if (o->set_position_decel_valid) {
        int32_t old_v = 0, new_v = 0;
        rc = myact_accel_read(c, MYACT_ACCEL_POSITION_DEC, &old_v); if (rc < 0) return rc;
        rc = myact_accel_write_rom(c, MYACT_ACCEL_POSITION_DEC, (uint32_t)o->set_position_decel_dps2); if (rc < 0) return rc;
        rc = myact_accel_read(c, MYACT_ACCEL_POSITION_DEC, &new_v); if (rc < 0) return rc;
        printf("position_decel_dps2 old=%ld new=%ld\n", (long)old_v, (long)new_v);
    }

    {
        bool write_rom = o->write_pid_rom;
        if (!o->write_pid_ram && !o->write_pid_rom) {
            printf("PID write target defaulting to RAM (use --write-pid-rom for persistent PID writes).\n");
        }
        if (o->set_speed_kp_valid) { rc = commissioning_write_float(c, "speed_kp", MYACT_PID_SPEED_KP, o->set_speed_kp, write_rom); if (rc < 0) return rc; }
        if (o->set_speed_ki_valid) { rc = commissioning_write_float(c, "speed_ki", MYACT_PID_SPEED_KI, o->set_speed_ki, write_rom); if (rc < 0) return rc; }
        if (o->set_position_kp_valid) { rc = commissioning_write_float(c, "position_kp", MYACT_PID_POSITION_KP, o->set_position_kp, write_rom); if (rc < 0) return rc; }
        if (o->set_position_ki_valid) { rc = commissioning_write_float(c, "position_ki", MYACT_PID_POSITION_KI, o->set_position_ki, write_rom); if (rc < 0) return rc; }
        if (o->set_position_kd_valid) { rc = commissioning_write_float(c, "position_kd", MYACT_PID_POSITION_KD, o->set_position_kd, write_rom); if (rc < 0) return rc; }
    }
    if (o->set_positive_limit_deg_valid) {
        int32_t raw = myact_deg_to_raw_0p01(o->set_positive_limit_deg);
        rc = myact_set_positive_limit_deg(c, o->set_positive_limit_deg); if (rc < 0) return rc;
        printf("positive_limit_deg requested=%.2f raw_0p01deg=%ld (write-only; no readback)\n",
               o->set_positive_limit_deg, (long)raw);
    }
    if (o->set_negative_limit_deg_valid) {
        int32_t raw = myact_deg_to_raw_0p01(o->set_negative_limit_deg);
        rc = myact_set_negative_limit_deg(c, o->set_negative_limit_deg); if (rc < 0) return rc;
        printf("negative_limit_deg requested=%.2f raw_0p01deg=%ld (write-only; no readback)\n",
               o->set_negative_limit_deg, (long)raw);
    }

    return MYACT_OK;
}




typedef enum { RAW_SMOKE_A1, RAW_SMOKE_A2, RAW_SMOKE_A4, RAW_SMOKE_A8 } raw_smoke_kind_t;

static int run_one_raw_smoke(myact_ctx_t* c, const test_opts_t* o, raw_smoke_kind_t kind) {
    int rc; myact_status1_t s1_before, s1_after; myact_status2_t s2_before, s2_after;
    int32_t a_before_raw = 0, a_after_raw = 0; double a_before=0.0, a_after=0.0, delta=0.0;
    uint8_t frame[8] = {0}; const char* name = ""; bool pass = false;
    double target_abs_deg = 0.0, cmd_delta_deg = 0.0;

    rc = myact_active_reply_disable_all(c); if (rc < 0) return rc;
    rc = myact_drain_rx(c, 300); if (rc < 0) return rc;
    rc = myact_stop_send(c); if (rc < 0) return rc;
    rc = myact_shutdown_send(c); if (rc < 0) return rc;

    rc = myact_read_status1(c, &s1_before); if (rc < 0) return rc;
    rc = myact_read_status2(c, &s2_before); if (rc < 0) return rc;
    rc = myact_read_angle_multi(c, &a_before_raw); if (rc < 0) return rc;
    a_before = myact_raw_0p01_to_deg(a_before_raw);

    if (!s1_before.brake_release_command_state) {
        uint64_t t0 = now_ms(NULL);
        rc = myact_brake_release(c); if (rc < 0) return rc;
        while (now_ms(NULL) - t0 < 2500) {
            rc = myact_read_status1(c, &s1_before); if (rc < 0) return rc;
            if (s1_before.brake_release_command_state) break;
            msleep(o->brake_poll_ms ? o->brake_poll_ms : 50);
        }
        if (!s1_before.brake_release_command_state) return MYACT_ETIMEDOUT;
    }

    switch (kind) {
        case RAW_SMOKE_A1: {
            int16_t iq = myact_current_a_to_raw(0.20);
            name = "raw-a1-smoke"; frame[0]=0xA1; frame[4]=iq & 0xFF; frame[5]=(iq>>8)&0xFF;
            rc = myact_torque_send(c, iq); if (rc < 0) return rc;
            msleep(200);
            cmd_delta_deg = 0.0;
            break; }
        case RAW_SMOKE_A2: {
            int32_t sp = myact_speed_dps_to_raw(100.0);
            name = "raw-a2-smoke"; frame[0]=0xA2; frame[4]=sp&0xFF; frame[5]=(sp>>8)&0xFF; frame[6]=(sp>>16)&0xFF; frame[7]=(sp>>24)&0xFF;
            rc = myact_speed_send(c, sp); if (rc < 0) return rc;
            msleep(700);
            break; }
        case RAW_SMOKE_A4: {
            int32_t target_raw = myact_deg_to_raw_0p01(a_before + 5.0);
            uint16_t sp = 50;
            target_abs_deg = a_before + 5.0;
            name = "raw-a4-smoke"; frame[0]=0xA4; frame[2]=sp&0xFF; frame[3]=(sp>>8)&0xFF; frame[4]=target_raw&0xFF; frame[5]=(target_raw>>8)&0xFF; frame[6]=(target_raw>>16)&0xFF; frame[7]=(target_raw>>24)&0xFF;
            rc = myact_position_abs_send(c, sp, target_raw); if (rc < 0) return rc;
            msleep(700);
            break; }
        case RAW_SMOKE_A8: {
            int32_t d = myact_deg_to_raw_0p01(5.0); uint16_t sp = 50; cmd_delta_deg = 5.0;
            name = "raw-a8-smoke"; frame[0]=0xA8; frame[2]=sp&0xFF; frame[3]=(sp>>8)&0xFF; frame[4]=d&0xFF; frame[5]=(d>>8)&0xFF; frame[6]=(d>>16)&0xFF; frame[7]=(d>>24)&0xFF;
            rc = myact_position_inc_send(c, sp, d); if (rc < 0) return rc;
            msleep(700);
            break; }
    }

    rc = myact_stop_send(c); if (rc < 0) return rc;
    rc = myact_shutdown_send(c); if (rc < 0) return rc;
    rc = myact_read_status1(c, &s1_after); if (rc < 0) return rc;
    rc = myact_read_status2(c, &s2_after); if (rc < 0) return rc;
    rc = myact_read_angle_multi(c, &a_after_raw); if (rc < 0) return rc;
    a_after = myact_raw_0p01_to_deg(a_after_raw);
    delta = a_after - a_before;

    if (kind == RAW_SMOKE_A1) pass = fabs(delta) >= o->torque_pass_min_delta_deg || fabs(s2_after.iq_0p01a/100.0) >= o->torque_pass_min_iq_a;
    else if (kind == RAW_SMOKE_A2) pass = fabs(delta) >= o->speed_pass_min_delta_deg;
    else if (kind == RAW_SMOKE_A4) pass = fabs(a_after - target_abs_deg) <= o->abs_tolerance_deg;
    else pass = fabs(delta - cmd_delta_deg) <= fmax(1.0, 0.20 * fabs(cmd_delta_deg));

    printf("%s frame=%02X %02X %02X %02X %02X %02X %02X %02X angle_before=%.2f angle_after=%.2f delta_deg=%.2f brake_before=%u brake_after=%u voltage=%.1f error_flags=0x%04X %s\n",
        name, frame[0],frame[1],frame[2],frame[3],frame[4],frame[5],frame[6],frame[7],
        a_before, a_after, delta, (unsigned)s1_before.brake_release_command_state, (unsigned)s1_after.brake_release_command_state,
        s1_after.voltage_0p1v/10.0, s1_after.error_flags, pass ? "PASS" : "FAIL");

    return pass ? RET_OK : RET_COMMS_FAIL;
}

static int run_raw_smoke_suite(myact_ctx_t* c, const test_opts_t* o) {
    int rc = RET_OK;
    if (o->raw_a1_smoke || o->raw_smoke_all) if (run_one_raw_smoke(c,o,RAW_SMOKE_A1)!=RET_OK) rc = RET_COMMS_FAIL;
    if (o->raw_a2_smoke || o->raw_smoke_all) if (run_one_raw_smoke(c,o,RAW_SMOKE_A2)!=RET_OK) rc = RET_COMMS_FAIL;
    if (o->raw_a4_smoke || o->raw_smoke_all) if (run_one_raw_smoke(c,o,RAW_SMOKE_A4)!=RET_OK) rc = RET_COMMS_FAIL;
    if (o->raw_a8_smoke || o->raw_smoke_all) if (run_one_raw_smoke(c,o,RAW_SMOKE_A8)!=RET_OK) rc = RET_COMMS_FAIL;
    return rc;
}

static int selftest_send_cb(const myact_can_frame_t* fr, void* user) {
    myact_can_frame_t* cap = (myact_can_frame_t*)user;
    if (!fr || !cap) return MYACT_EINVAL;
    *cap = *fr;
    return MYACT_OK;
}

static int selftest_recv_cb(myact_can_frame_t* out, uint32_t timeout_ms, void* user) {
    (void)out; (void)timeout_ms; (void)user;
    return MYACT_ETIMEDOUT;
}

static void print_bytes(const uint8_t b[8]) {
    printf("%02X %02X %02X %02X %02X %02X %02X %02X",
           b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
}

static int run_selftest_pack(void) {
    myact_io_t io;
    myact_ctx_t ctx;
    myact_can_frame_t cap;
    int rc;
    int fails = 0;

    struct {
        const char* name;
        uint8_t expected[8];
        int (*fn)(myact_ctx_t*);
    } cases[] = {
        {"A2 +100 dps", {0xA2,0x00,0x00,0x00,0x10,0x27,0x00,0x00}, NULL},
        {"A2 -100 dps", {0xA2,0x00,0x00,0x00,0xF0,0xD8,0xFF,0xFF}, NULL},
        {"A4 +360 deg @500 dps", {0xA4,0x00,0xF4,0x01,0xA0,0x8C,0x00,0x00}, NULL},
        {"A4 -360 deg @500 dps", {0xA4,0x00,0xF4,0x01,0x60,0x73,0xFF,0xFF}, NULL},
        {"A8 +360 deg @500 dps", {0xA8,0x00,0xF4,0x01,0xA0,0x8C,0x00,0x00}, NULL},
        {"A8 -360 deg @500 dps", {0xA8,0x00,0xF4,0x01,0x60,0x73,0xFF,0xFF}, NULL},
    };

    memset(&io, 0, sizeof(io));
    memset(&cap, 0, sizeof(cap));
    io.send = selftest_send_cb;
    io.recv = selftest_recv_cb;
    io.time_ms = now_ms;
    io.user = &cap;

    rc = myact_init(&ctx, 1, &io);
    if (rc < 0) {
        fprintf(stderr, "selftest-pack init failed rc=%d (%s)\n", rc, myact_strerror(rc));
        return RET_COMMS_FAIL;
    }

    #define CHECK_CASE(idx, call_expr) do { \
        memset(&cap, 0, sizeof(cap)); \
        rc = (call_expr); \
        if (rc < 0) { \
            fprintf(stderr, "[FAIL] %s send rc=%d (%s)\n", cases[idx].name, rc, myact_strerror(rc)); \
            ++fails; \
        } else if (memcmp(cap.data, cases[idx].expected, 8) != 0) { \
            printf("[FAIL] %s expected=", cases[idx].name); print_bytes(cases[idx].expected); \
            printf(" actual="); print_bytes(cap.data); printf("\n"); \
            ++fails; \
        } else { \
            printf("[PASS] %s\n", cases[idx].name); \
        } \
    } while (0)

    CHECK_CASE(0, myact_speed_dps_send(&ctx, 100.0));
    CHECK_CASE(1, myact_speed_dps_send(&ctx, -100.0));
    CHECK_CASE(2, myact_position_abs_deg_send(&ctx, 360.0, 500));
    CHECK_CASE(3, myact_position_abs_deg_send(&ctx, -360.0, 500));
    CHECK_CASE(4, myact_position_inc_send(&ctx, 500, 36000));
    CHECK_CASE(5, myact_position_inc_send(&ctx, 500, -36000));

    if (fails) {
        printf("selftest-pack: %d mismatch(es)\n", fails);
        return RET_COMMS_FAIL;
    }
    printf("selftest-pack: all packing checks passed\n");
    return RET_OK;
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

    if (argc < 2) {
        print_usage(argv[0]);
        return RET_BAD_CLI;
    }

    if (argc == 2 && !strcmp(argv[1], "--selftest-pack")) {
        return run_selftest_pack();
    }

    if (argc < 3) {
        print_usage(argv[0]);
        return RET_BAD_CLI;
    }

    rc = parse_cli(argc, argv, &o);
    if (rc != RET_OK) {
        print_usage(argv[0]);
        return RET_BAD_CLI;
    }

    if (o.selftest_pack) return run_selftest_pack();

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
    g_trace.enabled = o.trace_frames;
    if (g_trace.enabled &&
        (o.speed_step_test || o.native_a4_abs_test || o.rt_inc_burst || o.raw_motion_smoke || o.raw_motion_smoke_sendonly)) {
        printf("candump filter: candump -tz -ae can0,141:7FF,241:7FF,401:7FF,501:7FF\n");
    }

    if (o.reset_first) {
        (void)myact_system_reset(&ctx);
        msleep(2000);
    }

    rc = prepare_sync(&ctx);
    if (rc < 0) goto done;

    rc = read_full_diagnostics(&ctx, &s1, &a);
    if (rc < 0) goto done;
    rc = run_commissioning_writes(&ctx, &o);
    if (rc < 0) goto done;

    {
        bool any_test_requested = o.raw_motion_smoke || o.raw_motion_smoke_sendonly || o.raw_a1_smoke || o.raw_a2_smoke || o.raw_a4_smoke || o.raw_a8_smoke || o.raw_smoke_all || o.enable_hold ||
                                  o.brake_latency_test || o.brake_persistence_test || o.rt_inc_burst ||
                                  o.speed_step_test || o.speed_stream_test || o.torque_step_test || o.torque_sweep_test || o.native_a4_abs_test ||
                                  o.mit_mode_test || o.active_telemetry_rt_test || o.production_qual || o.experimental_all;
        bool any_dispatched = false;

        if (!any_test_requested) {
        printf("DIAG-ONLY mode: no motion requested.\n");
        rc = RET_OK;
        goto done;
        }

        if (o.production_qual) {
            int b;
            bool required_pass = true;

            any_dispatched = true;

            printf("[PASS] diagnostics\n");

            b = bench_result("identity-status-read", RET_OK);
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("brake-release-confirmation", run_enable_hold(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("stop-shutdown", run_brake_latency_test(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("active-telemetry", run_active_telemetry_rt_test(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("incremental-step-correctness", run_raw_motion_smoke(&ctx, &o, &s1, false));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("profile-position-correctness", run_native_a4_abs_test(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("jog-profile-velocity-correctness", run_speed_stream_test(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("command-stream-timeout", run_rt_inc_burst(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("startup-bringup-repeatability", run_rt_inc_burst(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("stop-latency", run_brake_latency_test(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("fault-stop-behavior", run_brake_persistence_test(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            b = bench_result("telemetry-freshness", run_active_telemetry_rt_test(&ctx, &o, &s1));
            if (b != RET_OK) {
                ret = RET_COMMS_FAIL;
                required_pass = false;
            }

            printf("production-qual summary: %s\n", required_pass ? "PASS" : "FAIL");
        } else if (o.experimental_all) {
            int b;
            any_dispatched = true;
            printf("[PASS] diagnostics\n");
            b = bench_result("speed-step-test", run_speed_step_test(&ctx, &o, &s1));
            if (b != RET_OK) ret = RET_COMMS_FAIL;
            b = bench_result("native-a4-abs-test", run_native_a4_abs_test(&ctx, &o, &s1));
            if (b != RET_OK) ret = RET_COMMS_FAIL;
            b = bench_result("torque-step-test", run_torque_step_test(&ctx, &o, &s1));
            if (b != RET_OK) ret = RET_COMMS_FAIL;
            if (o.run_mit_in_experimental_all) {
                b = bench_result("mit-mode-test", run_mit_mode_test(&ctx, &o, &s1));
                if (b != RET_OK) ret = RET_COMMS_FAIL;
            } else {
                printf("[SKIP] mit-mode-test: not requested (use --bench-all-mit)\n");
            }
        } else {
            if (o.brake_latency_test) { any_dispatched = true; if (bench_result("brake-latency-test", run_brake_latency_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.enable_hold) { any_dispatched = true; if (bench_result("enable-hold", run_enable_hold(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.raw_motion_smoke) { any_dispatched = true; if (bench_result("raw-motion-smoke", run_raw_motion_smoke(&ctx, &o, &s1, false)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.raw_motion_smoke_sendonly) { any_dispatched = true; if (bench_result("raw-motion-smoke-sendonly", run_raw_motion_smoke(&ctx, &o, &s1, true)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.raw_a1_smoke || o.raw_a2_smoke || o.raw_a4_smoke || o.raw_a8_smoke || o.raw_smoke_all) { any_dispatched = true; if (bench_result("raw-smoke-suite", run_raw_smoke_suite(&ctx, &o)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.brake_persistence_test) { any_dispatched = true; if (bench_result("brake-persistence-test", run_brake_persistence_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.rt_inc_burst) { any_dispatched = true; if (bench_result("rt-inc-burst", run_rt_inc_burst(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.speed_step_test) { any_dispatched = true; if (bench_result("speed-step-test", run_speed_step_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.speed_stream_test) { any_dispatched = true; if (bench_result("speed-stream-test", run_speed_stream_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
            if (o.native_a4_abs_test) { any_dispatched = true; if (bench_result("native-a4-abs-test", run_native_a4_abs_test(&ctx, &o, &s1)) != RET_OK) ret = RET_COMMS_FAIL; }
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
