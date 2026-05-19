/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   On-hardware smoke/diagnostic exerciser for Nanotec PD4-C CANopen/DS402 paths.
 *
 * Design notes:
 *   Validates mixed SDO setup traffic and runtime PDO command/telemetry behavior.
 *   Uses an additional passive listener socket to observe TPDO/HB/EMCY traffic
 *   independently from library telemetry caching for easier bus-level triage.
 */

/*
  pd4c_test.c — PD4-C CANopen test app (SDO + runtime PDO)

  Build (from ~/dev/src):
    gcc -O2 -Wall -Wextra -std=c11 -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L \
      -I../include \
      pd4c_test.c pd4c.c -lm -o pd4c_test

  Run:
    ./pd4c_test can0 1

  Notes:
  - This test avoids accessing any internal pd4c_ctx_t fields.
  - It uses your library for SDO + RPDO runtime commands.
  - It opens a separate CAN RAW socket to PASSIVELY decode TPDO/Heartbeat/EMCY
    so logs stay insightful even if the library keeps caches private.

  Your fixed runtime mapping (kept as-is):
    RPDO1 (0x200+node): 6040 Controlword + 6060 Mode
    RPDO2 (0x300+node): 607A Target Position (32) + 6081 Profile Velocity (16)
    RPDO3 (0x400+node): 6042 Target Velocity (16)

    TPDO1 (0x180+node): 6041 Statusword + 6061 Mode display
    TPDO2 (0x280+node): 6064 Position actual
    TPDO3 (0x380+node): 6044 Velocity actual
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "pd4c.h"

/* ----------------------------- time utils ----------------------------- */

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static uint64_t now_ms(void) { return now_us() / 1000ULL; }

static void msleep(int ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

/* ----------------------------- logging ----------------------------- */

static const char* rc_str(int rc) {
    if (rc == 0) return "OK";

    /* Keep this mapping robust across header changes */
#ifdef PD4C_ESDO_ABORT
    if (rc == PD4C_ESDO_ABORT) return "ESDO_ABORT";
#endif
#ifdef PD4C_EINVAL
    if (rc == PD4C_EINVAL) return "EINVAL";
#endif
#ifdef PD4C_ETIMEDOUT
    if (rc == PD4C_ETIMEDOUT) return "ETIMEDOUT";
#endif
#ifdef PD4C_EIO
    if (rc == PD4C_EIO) return "EIO";
#endif

    return "ERR";
}

static void log_cb(int level, const char* msg, void* user) {
    (void)user;
    const char* tag = "LOG";
#ifdef PD4C_LOG_ERROR
    if (level == PD4C_LOG_ERROR) tag = "ERR";
#endif
#ifdef PD4C_LOG_WARN
    if (level == PD4C_LOG_WARN)  tag = "WRN";
#endif
#ifdef PD4C_LOG_INFO
    if (level == PD4C_LOG_INFO)  tag = "INF";
#endif
#ifdef PD4C_LOG_DEBUG
    if (level == PD4C_LOG_DEBUG) tag = "DBG";
#endif
    fprintf(stderr, "[pd4c][%s] %s\n", tag, msg);
}

/* ---------------------- DS402 statusword decode ---------------------- */
/* Minimal decode only for readability in test. */
static const char* ds402_state_from_sw(uint16_t sw) {
    /* DS402 pattern on low bits: see CiA402 standard */
    uint16_t m = sw & 0x006F;
    if (m == 0x0000) return "NOT READY TO SWITCH ON";
    if (m == 0x0040) return "SWITCH ON DISABLED";
    if (m == 0x0021) return "READY TO SWITCH ON";
    if (m == 0x0023) return "SWITCHED ON";
    if (m == 0x0027) return "OPERATION ENABLED";
    if (m == 0x0007) return "QUICK STOP ACTIVE";
    if (m == 0x000F) return "FAULT REACTION ACTIVE";
    if (m == 0x0008) return "FAULT";
    return "UNKNOWN";
}

/* ------------------------ passive TPDO listener ----------------------- */

typedef struct {
    /* last values */
    uint16_t statusword;
    int8_t   mode_display;
    int32_t  pos_actual;
    int16_t  vel_actual;

    uint8_t  hb_state;
    uint16_t emcy_code;
    uint8_t  emcy_reg;
    uint8_t  emcy_mfg[5];

    /* timestamps */
    uint64_t ts_tpdo1_us;
    uint64_t ts_tpdo2_us;
    uint64_t ts_tpdo3_us;
    uint64_t ts_hb_us;
    uint64_t ts_emcy_us;

    /* counters */
    uint32_t cnt_tpdo1;
    uint32_t cnt_tpdo2;
    uint32_t cnt_tpdo3;
    uint32_t cnt_hb;
    uint32_t cnt_emcy;
} telem_t;

/* Passive listener is intentionally separate from pd4c_ctx socket:
 * it confirms what is really present on the bus even if the library socket
 * is filtered or application-level polling cadence delays cache visibility.
 */
static int can_open_listener(const char* ifname, int node_id, int* out_sock) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return -errno;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        int e = -errno;
        close(s);
        return e;
    }

    /* Filters for TPDO1/2/3, heartbeat, EMCY */
    struct can_filter filt[5];
    int n = 0;

    uint32_t tpdo1 = 0x180u + (uint32_t)node_id;
    uint32_t tpdo2 = 0x280u + (uint32_t)node_id;
    uint32_t tpdo3 = 0x380u + (uint32_t)node_id;
    uint32_t hb    = 0x700u + (uint32_t)node_id;
    uint32_t emcy  = 0x080u + (uint32_t)node_id;

    filt[n++] = (struct can_filter){ .can_id = tpdo1, .can_mask = CAN_SFF_MASK };
    filt[n++] = (struct can_filter){ .can_id = tpdo2, .can_mask = CAN_SFF_MASK };
    filt[n++] = (struct can_filter){ .can_id = tpdo3, .can_mask = CAN_SFF_MASK };
    filt[n++] = (struct can_filter){ .can_id = hb,    .can_mask = CAN_SFF_MASK };
    filt[n++] = (struct can_filter){ .can_id = emcy,  .can_mask = CAN_SFF_MASK };

    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, (socklen_t)(sizeof(filt[0]) * (size_t)n)) < 0) {
        int e = -errno;
        close(s);
        return e;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        int e = -errno;
        close(s);
        return e;
    }

    *out_sock = s;
    return 0;
}

static inline uint16_t le16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline int16_t  le16s(const uint8_t* p) { return (int16_t)le16(p); }
static inline int32_t  le32s(const uint8_t* p) {
    return (int32_t)((uint32_t)p[0] |
                    ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16) |
                    ((uint32_t)p[3] << 24));
}

static void telem_update_from_frame(telem_t* t, int node_id, const struct can_frame* fr) {
    uint32_t id = fr->can_id & CAN_SFF_MASK;
    uint64_t ts = now_us();

    const uint32_t tpdo1 = 0x180u + (uint32_t)node_id;
    const uint32_t tpdo2 = 0x280u + (uint32_t)node_id;
    const uint32_t tpdo3 = 0x380u + (uint32_t)node_id;
    const uint32_t hb    = 0x700u + (uint32_t)node_id;
    const uint32_t emcy  = 0x080u + (uint32_t)node_id;

    if (id == tpdo1 && fr->can_dlc >= 3) {
        t->statusword   = le16(fr->data);
        t->mode_display = (int8_t)fr->data[2];
        t->ts_tpdo1_us  = ts;
        t->cnt_tpdo1++;
    } else if (id == tpdo2 && fr->can_dlc >= 4) {
        t->pos_actual  = le32s(fr->data);
        t->ts_tpdo2_us = ts;
        t->cnt_tpdo2++;
    } else if (id == tpdo3 && fr->can_dlc >= 2) {
        t->vel_actual  = le16s(fr->data);
        t->ts_tpdo3_us = ts;
        t->cnt_tpdo3++;
    } else if (id == hb && fr->can_dlc >= 1) {
        t->hb_state  = fr->data[0];
        t->ts_hb_us  = ts;
        t->cnt_hb++;
    } else if (id == emcy && fr->can_dlc >= 3) {
        t->emcy_code = le16(fr->data);
        t->emcy_reg  = fr->data[2];
        if (fr->can_dlc >= 8) {
            memcpy(t->emcy_mfg, &fr->data[3], 5);
        } else {
            memset(t->emcy_mfg, 0, 5);
        }
        t->ts_emcy_us = ts;
        t->cnt_emcy++;
    }
}

static void telem_print(const telem_t* t) {
    printf("  TPDO1: SW=0x%04X  state=%s  mode_disp=%d  age=%" PRIu64 "ms\n",
           (unsigned)t->statusword,
           ds402_state_from_sw(t->statusword),
           (int)t->mode_display,
           t->ts_tpdo1_us ? ((now_us() - t->ts_tpdo1_us) / 1000ULL) : 0ULL);

    printf("  TPDO2: pos_actual=%" PRId32 "  age=%" PRIu64 "ms\n",
           t->pos_actual,
           t->ts_tpdo2_us ? ((now_us() - t->ts_tpdo2_us) / 1000ULL) : 0ULL);

    printf("  TPDO3: vel_actual=%" PRId16 "  age=%" PRIu64 "ms\n",
           t->vel_actual,
           t->ts_tpdo3_us ? ((now_us() - t->ts_tpdo3_us) / 1000ULL) : 0ULL);

    printf("  HB:    state=0x%02X  age=%" PRIu64 "ms\n",
           (unsigned)t->hb_state,
           t->ts_hb_us ? ((now_us() - t->ts_hb_us) / 1000ULL) : 0ULL);

    if (t->cnt_emcy) {
        printf("  EMCY:  code=0x%04X reg=0x%02X mfg=%02X %02X %02X %02X %02X age=%" PRIu64 "ms\n",
               (unsigned)t->emcy_code, (unsigned)t->emcy_reg,
               (unsigned)t->emcy_mfg[0], (unsigned)t->emcy_mfg[1], (unsigned)t->emcy_mfg[2],
               (unsigned)t->emcy_mfg[3], (unsigned)t->emcy_mfg[4],
               t->ts_emcy_us ? ((now_us() - t->ts_emcy_us) / 1000ULL) : 0ULL);
    } else {
        printf("  EMCY:  none\n");
    }
}

/* -------------------------- graceful stop -------------------------- */

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int s) { (void)s; g_stop = 1; }

/* -------------------------- SDO diagnostics -------------------------- */

static void sdo_diag(pd4c_ctx_t* ctx) {
    uint16_t sw = 0, ec = 0;
    uint8_t  md = 0, er = 0;
    int32_t  pos = 0;
    uint16_t vel_u16 = 0;

    (void)pd4c_sdo_read_u16(ctx, 0x6041, 0x00, &sw);
    (void)pd4c_sdo_read_u8 (ctx, 0x6061, 0x00, &md);
    (void)pd4c_sdo_read_i32(ctx, 0x6064, 0x00, &pos);
    (void)pd4c_sdo_read_u16(ctx, 0x6044, 0x00, &vel_u16);
    (void)pd4c_sdo_read_u16(ctx, 0x603F, 0x00, &ec);
    (void)pd4c_sdo_read_u8 (ctx, 0x1001, 0x00, &er);

    printf("  SDO: SW=0x%04X (%s)\n", (unsigned)sw, ds402_state_from_sw(sw));
    printf("  SDO: mode_display(6061)=%u\n", (unsigned)md);
    printf("  SDO: pos_actual(6064)=%" PRId32 "\n", pos);
    printf("  SDO: vel_actual(6044)=%" PRId16 "\n", (int16_t)vel_u16);
    printf("  SDO: err_code(603F)=0x%04X  err_reg(1001)=0x%02X\n", (unsigned)ec, (unsigned)er);
}

/* -------------------------- runtime PDO helpers -------------------------- */

static int send_rpdo1(pd4c_ctx_t* ctx, uint16_t cw, int8_t mode) {
    /* Preferred API name (your new lib) */
    return pd4c_rpdo1_send_control_mode(ctx, cw, mode);
}

static int send_rpdo2_pp(pd4c_ctx_t* ctx, int32_t target_pos, uint16_t profile_vel) {
    return pd4c_rpdo2_send_pp(ctx, target_pos, profile_vel);
}

static int send_rpdo3_vel(pd4c_ctx_t* ctx, int16_t vel) {
    return pd4c_rpdo3_send_target_velocity(ctx, vel);
}

static bool sw_is_fault(uint16_t sw) { return (sw & 0x0008u) != 0; }

static int assert_mode_display(pd4c_ctx_t* ctx, uint8_t expected_mode, int timeout_ms) {
    /* PD4-C uses 0x6042/0x6044 in mode 2 (Velocity mode), not mode 3. */
    uint64_t t0 = now_ms();
    uint8_t mode_display = 0;

    while ((int)(now_ms() - t0) < timeout_ms) {
        int rc = pd4c_sdo_read_u8(ctx, 0x6061, 0x00, &mode_display);
        if (rc == 0 && mode_display == expected_mode) return 0;
        msleep(20);
    }

    fprintf(stderr,
            "mode_display mismatch: expected=%u got=%u before RPDO3 velocity command\n",
            (unsigned)expected_mode,
            (unsigned)mode_display);
    return -1;
}

/* enable sequence using RPDO1 (controlword + mode), verify with SDO reads */
static int ds402_to_enabled(pd4c_ctx_t* ctx, int8_t mode, int timeout_ms) {
    uint64_t t0 = now_ms();
    uint16_t sw = 0;

    /* fault reset if needed */
    (void)pd4c_sdo_read_u16(ctx, 0x6041, 0x00, &sw);
    if (sw_is_fault(sw)) {
        int rc = send_rpdo1(ctx, 0x0080, mode); /* fault reset */
        if (rc) return rc;
        msleep(50);
    }

    /* Shutdown -> Switched on -> Operation enabled */
    while ((int)(now_ms() - t0) < timeout_ms) {
        (void)pd4c_sdo_read_u16(ctx, 0x6041, 0x00, &sw);
        uint16_t m = sw & 0x006F;

        if (m == 0x0027) return 0; /* op enabled */

        if (m == 0x0040) { /* switch on disabled -> shutdown */
            int rc = send_rpdo1(ctx, 0x0006, mode);
            if (rc) return rc;
        } else if (m == 0x0021) { /* ready to switch on -> switch on */
            int rc = send_rpdo1(ctx, 0x0007, mode);
            if (rc) return rc;
        } else if (m == 0x0023) { /* switched on -> enable op */
            int rc = send_rpdo1(ctx, 0x000F, mode);
            if (rc) return rc;
        } else {
            /* keep trying a safe path */
            int rc = send_rpdo1(ctx, 0x0006, mode);
            if (rc) return rc;
        }
        msleep(30);
    }
    return -1; /* timeout */
}

/* -------------------------- event mask pretty print -------------------------- */

static void print_ev_mask(int ev) {
    printf("  pd4c_poll events: 0x%X", ev);
#ifdef PD4C_EVT_TPDO1
    if (ev & PD4C_EVT_TPDO1) printf(" TPDO1");
#endif
#ifdef PD4C_EVT_TPDO2
    if (ev & PD4C_EVT_TPDO2) printf(" TPDO2");
#endif
#ifdef PD4C_EVT_TPDO3
    if (ev & PD4C_EVT_TPDO3) printf(" TPDO3");
#endif
#ifdef PD4C_EVT_HEARTBEAT
    if (ev & PD4C_EVT_HEARTBEAT) printf(" HB");
#endif
#ifdef PD4C_EVT_EMCY
    if (ev & PD4C_EVT_EMCY) printf(" EMCY");
#endif
    printf("\n");
}

/* ------------------------------- main tests ------------------------------- */

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <can_iface> <node_id>\n", argv[0]);
        return 2;
    }

    const char* ifname = argv[1];
    int node = atoi(argv[2]);
    if (node <= 0 || node > 127) {
        fprintf(stderr, "Invalid node_id: %d\n", node);
        return 2;
    }

    signal(SIGINT, on_sigint);

    printf("=== PD4C TEST (SDO + runtime PDO) ===\n");
    printf("  iface=%s node_id=%d\n", ifname, node);
    printf("  reducer: 100 motor rev = 1 turret rev (100:1)\n\n");

    pd4c_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));

    /* logger (if supported) */
    (void)pd4c_set_logger(&ctx, log_cb, NULL);

    int rc = pd4c_open(&ctx, ifname, (uint8_t)node);
    if (rc != 0) {
        fprintf(stderr, "pd4c_open failed: rc=%d (%s)\n", rc, rc_str(rc));
        return 1;
    }

    int sock_listen = -1;
    int lrc = can_open_listener(ifname, node, &sock_listen);
    if (lrc != 0) {
        fprintf(stderr, "WARN: passive listener socket failed: %d\n", lrc);
        /* continue anyway */
    }

    telem_t telem;
    memset(&telem, 0, sizeof(telem));

    printf("[1] Basic SDO diagnostics\n");
    sdo_diag(&ctx);
    printf("\n");

    printf("[2] NMT: set Operational\n");
    rc = pd4c_nmt_start(&ctx);
    printf("  pd4c_nmt_start: %s (%d)\n\n", rc_str(rc), rc);
    if (rc != 0) goto cleanup;

    /* Drain some TPDO/heartbeat for 1 second */
    if (sock_listen >= 0) {
        printf("[3] Passive telemetry warm-up (1s)\n");
        uint64_t t_end = now_ms() + 1000;
        struct pollfd pfd = { .fd = sock_listen, .events = POLLIN, .revents = 0 };
        while (!g_stop && now_ms() < t_end) {
            int pr = poll(&pfd, 1, 50);
            if (pr > 0 && (pfd.revents & POLLIN)) {
                struct can_frame fr;
                int n = (int)read(sock_listen, &fr, sizeof(fr));
                if (n == (int)sizeof(fr)) telem_update_from_frame(&telem, node, &fr);
            }
        }
        telem_print(&telem);
        printf("\n");
    }

    /* ---------------- velocity mode via RPDO3 (6042) ---------------- */
    printf("[4] Velocity mode (6060=2) test via PDO\n");
    printf("  target channel: RPDO3 (0x%03X) -> 0x6042 (int16)\n", 0x400 + node);
    printf("  note: PD4-C object dictionary ties 0x6042 target velocity to DS402 mode 2.\n");

    rc = ds402_to_enabled(&ctx, /*mode=*/2, /*timeout_ms=*/2000);
    printf("  ds402_to_enabled(mode=2): %s (%d)\n", (rc==0)?"OK":"FAIL", rc);
    if (rc != 0) goto cleanup;

    rc = assert_mode_display(&ctx, /*expected_mode=*/2, /*timeout_ms=*/500);
    printf("  assert mode_display(6061)==2 before RPDO3: %s (%d)\n", (rc==0)?"OK":"FAIL", rc);
    if (rc != 0) goto cleanup;

    /* command +1000 then stop */
    printf("  command vel=+1000 for 1000 ms, then 0\n");
    rc = send_rpdo3_vel(&ctx, (int16_t)1000);
    printf("    rpdo3_send_vel(+1000): %s (%d)\n", rc_str(rc), rc);
    msleep(1000);
    (void)send_rpdo3_vel(&ctx, (int16_t)0);

    /* command -1000 then stop */
    printf("  command vel=-1000 for 1000 ms, then 0\n");
    rc = send_rpdo3_vel(&ctx, (int16_t)-1000);
    printf("    rpdo3_send_vel(-1000): %s (%d)\n", rc_str(rc), rc);
    msleep(1000);
    (void)send_rpdo3_vel(&ctx, (int16_t)0);

    if (sock_listen >= 0) {
        printf("  telemetry after velocity test:\n");
        /* collect a bit more frames */
        uint64_t t_end = now_ms() + 300;
        struct pollfd pfd = { .fd = sock_listen, .events = POLLIN, .revents = 0 };
        while (!g_stop && now_ms() < t_end) {
            int pr = poll(&pfd, 1, 50);
            if (pr > 0 && (pfd.revents & POLLIN)) {
                struct can_frame fr;
                int n = (int)read(sock_listen, &fr, sizeof(fr));
                if (n == (int)sizeof(fr)) telem_update_from_frame(&telem, node, &fr);
            }
        }
        telem_print(&telem);
    }
    printf("\n");

    /* ---------------- profile position via RPDO2 ---------------- */
    printf("[5] Profile Position test via PDO\n");
    printf("  target channel: RPDO2 (0x%03X) -> 0x607A(int32)+0x6081(uint16)\n", 0x300 + node);

    rc = ds402_to_enabled(&ctx, /*mode=*/1, /*timeout_ms=*/2000);
    printf("  ds402_to_enabled(mode=1): %s (%d)\n", (rc==0)?"OK":"FAIL", rc);
    if (rc != 0) goto cleanup;

    /* choose a small safe move in internal units */
    int32_t start_pos = 0;
    (void)pd4c_sdo_read_i32(&ctx, 0x6064, 0x00, &start_pos);
    int32_t target = start_pos + 2000;
    uint16_t prof_vel = 1000;

    printf("  start_pos=%" PRId32 "  target=%" PRId32 "  profile_vel=%u\n", start_pos, target, (unsigned)prof_vel);

    rc = send_rpdo2_pp(&ctx, target, prof_vel);
    printf("    rpdo2_send_pp(target, prof_vel): %s (%d)\n", rc_str(rc), rc);

    /* trigger setpoint edge via RPDO1 controlword bit4: 0x001F then back to 0x000F */
    rc = send_rpdo1(&ctx, (uint16_t)0x001F, (int8_t)1);
    printf("    rpdo1 setpoint=1 (0x001F): %s (%d)\n", rc_str(rc), rc);
    msleep(20);
    rc = send_rpdo1(&ctx, (uint16_t)0x000F, (int8_t)1);
    printf("    rpdo1 setpoint=0 (0x000F): %s (%d)\n", rc_str(rc), rc);

    /* wait a bit and show new pos */
    msleep(500);

    int32_t pos_after = 0;
    (void)pd4c_sdo_read_i32(&ctx, 0x6064, 0x00, &pos_after);
    printf("  pos_after(SDO 6064)=%" PRId32 "  delta=%" PRId32 "\n",
           pos_after, (pos_after - start_pos));

    if (sock_listen >= 0) {
        printf("  telemetry after PP test:\n");
        uint64_t t_end = now_ms() + 300;
        struct pollfd pfd = { .fd = sock_listen, .events = POLLIN, .revents = 0 };
        while (!g_stop && now_ms() < t_end) {
            int pr = poll(&pfd, 1, 50);
            if (pr > 0 && (pfd.revents & POLLIN)) {
                struct can_frame fr;
                int n = (int)read(sock_listen, &fr, sizeof(fr));
                if (n == (int)sizeof(fr)) telem_update_from_frame(&telem, node, &fr);
            }
        }
        telem_print(&telem);
    }
    printf("\n");

    /* ---------------- library polling verification (no ctx internals) ---------------- */
    printf("[6] Library poll() verification (counts events for 2s)\n");
    printf("  (This checks your library sees TPDO/HB/EMCY events without exposing ctx internals.)\n");

    uint64_t t_end = now_ms() + 2000;
    uint32_t ev_cnt = 0;
    while (!g_stop && now_ms() < t_end) {
        int ev = pd4c_poll(&ctx, 50);
        if (ev > 0) {
            ev_cnt++;
            /* Print occasionally */
            if ((ev_cnt % 10u) == 1u) print_ev_mask(ev);
        }
    }
    printf("  pd4c_poll event bursts observed: %u\n\n", ev_cnt);

    printf("[7] Final SDO diagnostics\n");
    sdo_diag(&ctx);
    printf("\n");

cleanup:
    /* Safe stop best-effort */
    (void)send_rpdo3_vel(&ctx, 0);
    (void)send_rpdo1(&ctx, 0x0006, 0); /* shutdown */

    if (sock_listen >= 0) close(sock_listen);
    pd4c_close(&ctx);

    printf("Done.\n");
    return (rc == 0) ? 0 : 1;
}
