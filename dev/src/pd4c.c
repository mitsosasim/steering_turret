/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Implementation/declarations for the STC development codebase.
 *
 * Design notes:
 *   Documentation/comment ownership header only; no runtime behavior changes.
 */

#define _GNU_SOURCE
#include "pd4c.h"
#include "stc_log.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <stdarg.h>
#include <poll.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

/* ---------------- time + logging ---------------- */

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static void default_logger(pd4c_log_level_t lvl, const char* msg, void* user) {
  (void)user;
    if (lvl == PD4C_LOG_ERROR) STC_LOG_ERROR("[pd4c] %s", msg);
    else if (lvl == PD4C_LOG_WARN) STC_LOG_WARN("[pd4c] %s", msg);
    else if (lvl == PD4C_LOG_DEBUG) STC_LOG_DEBUG("[pd4c] %s", msg);
    else STC_LOG_INFO("[pd4c] %s", msg);
}

static void logf_ctx(pd4c_ctx_t* ctx, pd4c_log_level_t lvl, const char* fmt, ...) {
    if (!ctx) return;
    if (!ctx->log_cb) ctx->log_cb = default_logger;

    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    ctx->log_cb(lvl, buf, ctx->log_user);
}

const char* pd4c_strerror(int rc) {
    switch (rc) {
        case PD4C_OK: return "OK";
        case PD4C_EINVAL: return "EINVAL";
        case PD4C_EIO: return "EIO";
        case PD4C_ETIMEDOUT: return "ETIMEDOUT";
        case PD4C_ESDO_ABORT: return "ESDO_ABORT";
        case PD4C_ESTATE: return "ESTATE";
        default: return "UNKNOWN";
    }
}

void pd4c_set_logger(pd4c_ctx_t* ctx, pd4c_log_fn fn, void* user) {
    if (!ctx) return;
    ctx->log_cb = fn ? fn : default_logger;
    ctx->log_user = user;
}

void pd4c_set_sdo_timeout(pd4c_ctx_t* ctx, int timeout_ms) {
    if (!ctx) return;
    if (timeout_ms < 10) timeout_ms = 10;
    ctx->sdo_timeout_ms = timeout_ms;
}

const pd4c_telemetry_t* pd4c_telem(const pd4c_ctx_t* ctx) {
    return ctx ? &ctx->telem : NULL;
}

/* ---------------- CAN send/recv ---------------- */

static int can_send_frame(pd4c_ctx_t* ctx, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    if (!ctx || ctx->sock < 0) return PD4C_EINVAL;
    if (dlc > 8) return PD4C_EINVAL;

    struct can_frame fr;
    memset(&fr, 0, sizeof(fr));
    fr.can_id = can_id;
    fr.can_dlc = dlc;
    if (dlc && data) memcpy(fr.data, data, dlc);

    int n = (int)write(ctx->sock, &fr, sizeof(fr));
    if (n != (int)sizeof(fr)) {
        logf_ctx(ctx, PD4C_LOG_ERROR, "CAN write failed id=0x%X: %s", can_id, strerror(errno));
        return PD4C_EIO;
    }
    return PD4C_OK;
}

static int can_recv_frame(pd4c_ctx_t* ctx, int timeout_ms, struct can_frame* out) {
    if (!ctx || ctx->sock < 0 || !out) return PD4C_EINVAL;

    struct pollfd pfd;
    pfd.fd = ctx->sock;
    pfd.events = POLLIN;

    int prc = poll(&pfd, 1, timeout_ms);
    if (prc == 0) return PD4C_ETIMEDOUT;
    if (prc < 0) {
        if (errno == EINTR) return PD4C_ETIMEDOUT;
        logf_ctx(ctx, PD4C_LOG_ERROR, "poll() failed: %s", strerror(errno));
        return PD4C_EIO;
    }

    int n = (int)read(ctx->sock, out, sizeof(*out));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return PD4C_ETIMEDOUT;
        logf_ctx(ctx, PD4C_LOG_ERROR, "CAN read failed: %s", strerror(errno));
        return PD4C_EIO;
    }
    if (n != (int)sizeof(*out)) return PD4C_EIO;
    return PD4C_OK;
}

/* ---------------- parsing helpers ---------------- */

static inline uint16_t le16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t le32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void wr_le16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline void wr_le32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static int handle_frame(pd4c_ctx_t* ctx, const struct can_frame* fr) {
    if (!ctx || !fr) return PD4C_EINVAL;

    uint32_t id = fr->can_id & CAN_SFF_MASK;
    uint64_t t = now_ms();
    int ev = PD4C_EVT_NONE;

    if (id == ctx->tpdo1_id && fr->can_dlc >= 3) {
        ctx->telem.statusword   = le16(fr->data + 0);
        ctx->telem.mode_display = (int8_t)fr->data[2];
        ctx->telem.have_tpdo1   = true;
        ctx->telem.ts_tpdo1_ms  = t;
        ev |= PD4C_EVT_TPDO1;
        return ev;
    }
    if (id == ctx->tpdo2_id && fr->can_dlc >= 4) {
        ctx->telem.pos_actual  = (int32_t)le32(fr->data);
        ctx->telem.have_tpdo2  = true;
        ctx->telem.ts_tpdo2_ms = t;
        ev |= PD4C_EVT_TPDO2;
        return ev;
    }
    if (id == ctx->tpdo3_id && (fr->can_dlc == 2 || fr->can_dlc == 4)) {
        if (fr->can_dlc == 2) {
            int16_t v = (int16_t)le16(fr->data);
            ctx->telem.vel_actual = (int32_t)v;
        } else {
            ctx->telem.vel_actual = (int32_t)le32(fr->data);
        }
        ctx->telem.have_tpdo3  = true;
        ctx->telem.ts_tpdo3_ms = t;
        ev |= PD4C_EVT_TPDO3;
        return ev;
    }
    if (id == ctx->hb_id && fr->can_dlc >= 1) {
        ctx->telem.hb_state  = (pd4c_nmt_state_t)fr->data[0];
        ctx->telem.have_hb   = true;
        ctx->telem.ts_hb_ms  = t;
        ev |= PD4C_EVT_HEARTBEAT;
        return ev;
    }
    if (id == ctx->emcy_id && fr->can_dlc == 8) {
        ctx->telem.emcy_error_code = le16(fr->data + 0);
        ctx->telem.emcy_error_reg  = fr->data[2];
        memcpy(ctx->telem.emcy_mfg, fr->data + 3, 5);
        ctx->telem.have_emcy = true;
        ctx->telem.ts_emcy_ms = t;
        ev |= PD4C_EVT_EMCY;

        logf_ctx(ctx, PD4C_LOG_ERROR,
                 "EMCY received: code=0x%04X reg=0x%02X mfg=%02X %02X %02X %02X %02X",
                 ctx->telem.emcy_error_code, ctx->telem.emcy_error_reg,
                 ctx->telem.emcy_mfg[0], ctx->telem.emcy_mfg[1], ctx->telem.emcy_mfg[2],
                 ctx->telem.emcy_mfg[3], ctx->telem.emcy_mfg[4]);
        return ev;
    }

    /* ignore other frames (but you can enable DEBUG in your logger to inspect) */
    return PD4C_EVT_NONE;
}

int pd4c_poll(pd4c_ctx_t* ctx, int timeout_ms) {
    if (!ctx) return PD4C_EINVAL;

    struct can_frame fr;
    int rc = can_recv_frame(ctx, timeout_ms, &fr);
    if (rc != PD4C_OK) return rc;

    return handle_frame(ctx, &fr);
}

/* ---------------- open/close ---------------- */

static void init_ids(pd4c_ctx_t* ctx) {
    const uint8_t n = ctx->node_id;
    ctx->sdo_tx_id = 0x600u + n;
    ctx->sdo_rx_id = 0x580u + n;

    ctx->rpdo1_id = 0x200u + n;
    ctx->rpdo2_id = 0x300u + n;
    ctx->rpdo3_id = 0x400u + n;

    ctx->tpdo1_id = 0x180u + n;
    ctx->tpdo2_id = 0x280u + n;
    ctx->tpdo3_id = 0x380u + n;

    ctx->hb_id   = 0x700u + n;
    ctx->emcy_id = 0x080u + n;
}

int pd4c_open(pd4c_ctx_t* ctx, const char* ifname, uint8_t node_id) {
    if (!ctx || !ifname || node_id == 0 || node_id > 127) return PD4C_EINVAL;

    memset(ctx, 0, sizeof(*ctx));
    ctx->sock = -1;
    ctx->node_id = node_id;
    snprintf(ctx->ifname, sizeof(ctx->ifname), "%s", ifname);
    ctx->sdo_timeout_ms = 500;
    ctx->log_cb = default_logger;
    ctx->log_user = NULL;
    init_ids(ctx);

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        logf_ctx(ctx, PD4C_LOG_ERROR, "socket(PF_CAN) failed: %s", strerror(errno));
        return PD4C_EIO;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        logf_ctx(ctx, PD4C_LOG_ERROR, "ioctl(SIOCGIFINDEX) failed: %s", strerror(errno));
        close(s);
        return PD4C_EIO;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        logf_ctx(ctx, PD4C_LOG_ERROR, "bind(%s) failed: %s", ifname, strerror(errno));
        close(s);
        return PD4C_EIO;
    }

    /* Install filters: accept SDO responses + TPDOs + heartbeat + EMCY.
       If you want full bus logging, remove filters at app layer by using candump.
     */
    struct can_filter flt[6];
    int k = 0;
    flt[k++] = (struct can_filter){ .can_id = ctx->sdo_rx_id, .can_mask = CAN_SFF_MASK };
    flt[k++] = (struct can_filter){ .can_id = ctx->tpdo1_id, .can_mask = CAN_SFF_MASK };
    flt[k++] = (struct can_filter){ .can_id = ctx->tpdo2_id, .can_mask = CAN_SFF_MASK };
    flt[k++] = (struct can_filter){ .can_id = ctx->tpdo3_id, .can_mask = CAN_SFF_MASK };
    flt[k++] = (struct can_filter){ .can_id = ctx->hb_id,    .can_mask = CAN_SFF_MASK };
    flt[k++] = (struct can_filter){ .can_id = ctx->emcy_id,  .can_mask = CAN_SFF_MASK };

    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &flt, (socklen_t)(sizeof(flt))) < 0) {
        logf_ctx(ctx, PD4C_LOG_WARN, "setsockopt(CAN_RAW_FILTER) failed: %s (continuing)", strerror(errno));
    }

    ctx->sock = s;
    logf_ctx(ctx, PD4C_LOG_INFO, "Opened %s node=%u (SDO 0x%03X/0x%03X)", ifname, node_id, ctx->sdo_tx_id, ctx->sdo_rx_id);
    return PD4C_OK;
}

void pd4c_close(pd4c_ctx_t* ctx) {
    if (!ctx) return;
    if (ctx->sock >= 0) close(ctx->sock);
    ctx->sock = -1;
}

/* ---------------- NMT ---------------- */

static int nmt_send(pd4c_ctx_t* ctx, uint8_t cs, uint8_t node) {
    uint8_t d[2] = { cs, node };
    return can_send_frame(ctx, 0x000, d, 2);
}

int pd4c_nmt_start(pd4c_ctx_t* ctx) {
    if (!ctx) return PD4C_EINVAL;
    return nmt_send(ctx, 0x01, ctx->node_id);
}
int pd4c_nmt_stop(pd4c_ctx_t* ctx) {
    if (!ctx) return PD4C_EINVAL;
    return nmt_send(ctx, 0x02, ctx->node_id);
}
int pd4c_nmt_preop(pd4c_ctx_t* ctx) {
    if (!ctx) return PD4C_EINVAL;
    return nmt_send(ctx, 0x80, ctx->node_id);
}
int pd4c_nmt_reset_node(pd4c_ctx_t* ctx) {
    if (!ctx) return PD4C_EINVAL;
    return nmt_send(ctx, 0x81, ctx->node_id);
}

/* ---------------- SDO expedited ---------------- */

static int sdo_wait_resp(pd4c_ctx_t* ctx, int timeout_ms, struct can_frame* out) {
    uint64_t start = now_ms();
    for (;;) {
        int remain = timeout_ms;
        if (timeout_ms >= 0) {
            uint64_t elapsed = now_ms() - start;
            if ((int)elapsed >= timeout_ms) return PD4C_ETIMEDOUT;
            remain = timeout_ms - (int)elapsed;
        }

        struct can_frame fr;
        int rc = can_recv_frame(ctx, remain, &fr);
        if (rc != PD4C_OK) return rc;

        uint32_t id = fr.can_id & CAN_SFF_MASK;
        if (id == ctx->sdo_rx_id) {
            *out = fr;
            return PD4C_OK;
        }

        /* Update telemetry with other frames while waiting for SDO. */
        (void)handle_frame(ctx, &fr);
    }
}

static int sdo_abort(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint32_t code) {
    logf_ctx(ctx, PD4C_LOG_ERROR, "SDO abort idx=0x%04X sub=0x%02X code=0x%08X", idx, sub, code);
    return PD4C_ESDO_ABORT;
}

static int sdo_read_exp(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint8_t* payload, uint8_t* payload_len) {
    if (!ctx || !payload || !payload_len) return PD4C_EINVAL;

    uint8_t req[8] = {0};
    req[0] = 0x40; /* upload request */
    wr_le16(req + 1, idx);
    req[3] = sub;

    int rc = can_send_frame(ctx, ctx->sdo_tx_id, req, 8);
    if (rc != PD4C_OK) return rc;

    struct can_frame resp;
    rc = sdo_wait_resp(ctx, ctx->sdo_timeout_ms, &resp);
    if (rc != PD4C_OK) return rc;
    if (resp.can_dlc != 8) return PD4C_EIO;

    uint8_t cs = resp.data[0];
    uint16_t ridx = le16(resp.data + 1);
    uint8_t rsub = resp.data[3];

    if (cs == 0x80) {
        uint32_t ac = le32(resp.data + 4);
        return sdo_abort(ctx, ridx, rsub, ac);
    }
    if (ridx != idx || rsub != sub) {
        logf_ctx(ctx, PD4C_LOG_WARN, "SDO read echo mismatch (req %04X:%02X got %04X:%02X)", idx, sub, ridx, rsub);
    }

    /* expedited upload response: bits indicate size */
    bool expedited = (cs & 0x02) != 0;
    bool size_ind  = (cs & 0x01) != 0;
    if (!expedited || !size_ind) return PD4C_EIO;

    uint8_t n_unused = (cs >> 2) & 0x03;
    uint8_t n = (uint8_t)(4 - n_unused);
    memcpy(payload, resp.data + 4, n);
    *payload_len = n;
    return PD4C_OK;
}

static int sdo_write_exp(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, const uint8_t* payload, uint8_t payload_len) {
    if (!ctx || (!payload && payload_len)) return PD4C_EINVAL;
    if (!(payload_len == 1 || payload_len == 2 || payload_len == 4)) return PD4C_EINVAL;

    uint8_t req[8] = {0};
    /* expedited download with size indicated */
    uint8_t n_unused = (uint8_t)(4 - payload_len);
    req[0] = (uint8_t)(0x20 | 0x02 | 0x01 | (n_unused << 2));
    wr_le16(req + 1, idx);
    req[3] = sub;
    memcpy(req + 4, payload, payload_len);

    int rc = can_send_frame(ctx, ctx->sdo_tx_id, req, 8);
    if (rc != PD4C_OK) return rc;

    struct can_frame resp;
    rc = sdo_wait_resp(ctx, ctx->sdo_timeout_ms, &resp);
    if (rc != PD4C_OK) return rc;
    if (resp.can_dlc != 8) return PD4C_EIO;

    uint8_t cs = resp.data[0];
    uint16_t ridx = le16(resp.data + 1);
    uint8_t rsub = resp.data[3];

    if (cs == 0x80) {
        uint32_t ac = le32(resp.data + 4);
        return sdo_abort(ctx, ridx, rsub, ac);
    }

    /* successful download response is 0x60 */
    if (cs != 0x60) {
        logf_ctx(ctx, PD4C_LOG_WARN, "SDO write unexpected cs=0x%02X (idx=%04X:%02X)", cs, ridx, rsub);
    }
    if (ridx != idx || rsub != sub) {
        logf_ctx(ctx, PD4C_LOG_WARN, "SDO write echo mismatch (req %04X:%02X got %04X:%02X)", idx, sub, ridx, rsub);
    }
    return PD4C_OK;
}

/* typed wrappers */
int pd4c_sdo_read_u8(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint8_t* out) {
    if (!out) return PD4C_EINVAL;
    uint8_t p[4]; uint8_t n = 0;
    int rc = sdo_read_exp(ctx, idx, sub, p, &n);
    if (rc != PD4C_OK) return rc;
    if (n != 1) return PD4C_EIO;
    *out = p[0];
    return PD4C_OK;
}
int pd4c_sdo_read_u16(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint16_t* out) {
    if (!out) return PD4C_EINVAL;
    uint8_t p[4]; uint8_t n = 0;
    int rc = sdo_read_exp(ctx, idx, sub, p, &n);
    if (rc != PD4C_OK) return rc;
    if (n != 2) return PD4C_EIO;
    *out = le16(p);
    return PD4C_OK;
}
int pd4c_sdo_read_u32(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint32_t* out) {
    if (!out) return PD4C_EINVAL;
    uint8_t p[4]; uint8_t n = 0;
    int rc = sdo_read_exp(ctx, idx, sub, p, &n);
    if (rc != PD4C_OK) return rc;
    if (n != 4) return PD4C_EIO;
    *out = le32(p);
    return PD4C_OK;
}
int pd4c_sdo_read_i32(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, int32_t* out) {
    uint32_t u = 0;
    int rc = pd4c_sdo_read_u32(ctx, idx, sub, &u);
    if (rc != PD4C_OK) return rc;
    *out = (int32_t)u;
    return PD4C_OK;
}

int pd4c_sdo_write_u8(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint8_t v) {
    return sdo_write_exp(ctx, idx, sub, &v, 1);
}
int pd4c_sdo_write_u16(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint16_t v) {
    uint8_t p[2]; wr_le16(p, v);
    return sdo_write_exp(ctx, idx, sub, p, 2);
}
int pd4c_sdo_write_u32(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, uint32_t v) {
    uint8_t p[4]; wr_le32(p, v);
    return sdo_write_exp(ctx, idx, sub, p, 4);
}
int pd4c_sdo_write_i32(pd4c_ctx_t* ctx, uint16_t idx, uint8_t sub, int32_t v) {
    return pd4c_sdo_write_u32(ctx, idx, sub, (uint32_t)v);
}

/* ---------------- PDO send ---------------- */

int pd4c_rpdo1_send_control_mode(pd4c_ctx_t* ctx, uint16_t controlword, int8_t mode) {
    if (!ctx) return PD4C_EINVAL;
    uint8_t d[3];
    wr_le16(d + 0, controlword);
    d[2] = (uint8_t)mode;
    return can_send_frame(ctx, ctx->rpdo1_id, d, 3);
}

int pd4c_rpdo2_send_pp(pd4c_ctx_t* ctx, int32_t target_pos, uint16_t profile_vel) {
    if (!ctx) return PD4C_EINVAL;
    uint8_t d[6];
    wr_le32(d + 0, (uint32_t)target_pos);
    wr_le16(d + 4, profile_vel);
    return can_send_frame(ctx, ctx->rpdo2_id, d, 6);
}

int pd4c_rpdo3_send_target_velocity(pd4c_ctx_t* ctx, int16_t target_vel) {
    if (!ctx) return PD4C_EINVAL;
    uint8_t d[2];
    wr_le16(d, (uint16_t)target_vel);
    return can_send_frame(ctx, ctx->rpdo3_id, d, 2);
}

/* ---------------- DS402 convenience ---------------- */

// DS402 statusword bits relevant to Profile Position handshake
#define PD4C_SW_SETPOINT_ACK   0x1000u  // bit12
#define PD4C_SW_TARGET_REACHED 0x0400u  // bit10 (optional)

static int ds402_state(uint16_t sw) {
    /* Derived from CiA402 state machine patterns (mask 0x006F). */
    uint16_t m = sw & 0x006F;
    if (m == 0x0000) return 0; /* NOT READY */
    if (m == 0x0040) return 1; /* SWITCH ON DISABLED */
    if (m == 0x0021) return 2; /* READY TO SWITCH ON */
    if (m == 0x0023) return 3; /* SWITCHED ON */
    if (m == 0x0027) return 4; /* OPERATION ENABLED */
    if (m == 0x0007) return 5; /* QUICK STOP ACTIVE (common pattern) */
    if (m == 0x000F) return 6; /* FAULT REACTION ACTIVE (pattern varies) */
    if (m == 0x0008) return 7; /* FAULT */
    return 99;
}

const char* pd4c_ds402_state_str(uint16_t sw) {
    switch (ds402_state(sw)) {
        case 0: return "NOT READY TO SWITCH ON";
        case 1: return "SWITCH ON DISABLED";
        case 2: return "READY TO SWITCH ON";
        case 3: return "SWITCHED ON";
        case 4: return "OPERATION ENABLED";
        case 5: return "QUICK STOP ACTIVE";
        case 6: return "FAULT REACTION ACTIVE";
        case 7: return "FAULT";
        default: return "UNKNOWN";
    }
}

static int wait_status(pd4c_ctx_t* ctx, int desired_state, int timeout_ms) {
    uint64_t start = now_ms();
    while ((int)(now_ms() - start) < timeout_ms) {
        /* ensure we process incoming TPDO1 */
        int rc = pd4c_poll(ctx, 50);
        if (rc >= 0) {
            if ((rc & PD4C_EVT_TPDO1) && ctx->telem.have_tpdo1) {
                int st = ds402_state(ctx->telem.statusword);
                if (st == desired_state) return PD4C_OK;
                if (st == 7) return PD4C_ESTATE; /* FAULT */
            }
        } else if (rc != PD4C_ETIMEDOUT) {
            return rc;
        }
    }
    return PD4C_ETIMEDOUT;
}

static int wait_statusword_mask_eq(pd4c_ctx_t* ctx,
                                   uint16_t mask,
                                   uint16_t want,
                                   int timeout_ms) {
    if (!ctx) return PD4C_EINVAL;
    if (timeout_ms < 50) timeout_ms = 50;

    uint64_t start = now_ms();
    while ((int)(now_ms() - start) < timeout_ms) {
        // progress by polling (small bounded slice)
        int prc = pd4c_poll(ctx, 10);
        if (prc < 0 && prc != PD4C_ETIMEDOUT) return prc;

        if (ctx->telem.have_tpdo1) {
            // if drive enters FAULT, stop early
            if (ds402_state(ctx->telem.statusword) == 7) return PD4C_ESTATE;

            if ((ctx->telem.statusword & mask) == want) return PD4C_OK;
        }
    }
    return PD4C_ETIMEDOUT;
}


int pd4c_ds402_enable_operation(pd4c_ctx_t* ctx, int8_t mode, int timeout_ms) {
    if (!ctx) return PD4C_EINVAL;
    if (timeout_ms < 200) timeout_ms = 200;

    /* shutdown -> switch on -> enable op */
    int rc;

    rc = pd4c_rpdo1_send_control_mode(ctx, 0x0006, mode);
    if (rc != PD4C_OK) return rc;
    rc = wait_status(ctx, 2, timeout_ms); /* READY TO SWITCH ON */
    if (rc != PD4C_OK) logf_ctx(ctx, PD4C_LOG_WARN, "enable_op: did not reach READY (rc=%d)", rc);

    rc = pd4c_rpdo1_send_control_mode(ctx, 0x0007, mode);
    if (rc != PD4C_OK) return rc;
    rc = wait_status(ctx, 3, timeout_ms); /* SWITCHED ON */
    if (rc != PD4C_OK) logf_ctx(ctx, PD4C_LOG_WARN, "enable_op: did not reach SWITCHED ON (rc=%d)", rc);

    rc = pd4c_rpdo1_send_control_mode(ctx, 0x000F, mode);
    if (rc != PD4C_OK) return rc;
    rc = wait_status(ctx, 4, timeout_ms); /* OPERATION ENABLED */
    return rc;
}

int pd4c_ds402_fault_reset(pd4c_ctx_t* ctx, int8_t mode, int timeout_ms) {
    if (!ctx) return PD4C_EINVAL;
    if (timeout_ms < 200) timeout_ms = 200;

    /* pulse bit7 */
    int rc = pd4c_rpdo1_send_control_mode(ctx, 0x0080, mode);
    if (rc != PD4C_OK) return rc;
    usleep(20000);
    rc = pd4c_rpdo1_send_control_mode(ctx, 0x0000, mode);
    if (rc != PD4C_OK) return rc;

    /* wait until not FAULT */
    uint64_t start = now_ms();
    while ((int)(now_ms() - start) < timeout_ms) {
        int prc = pd4c_poll(ctx, 50);
        if (prc >= 0 && (prc & PD4C_EVT_TPDO1) && ctx->telem.have_tpdo1) {
            int st = ds402_state(ctx->telem.statusword);
            if (st != 7) return PD4C_OK;
        }
    }
    return PD4C_ETIMEDOUT;
}

int pd4c_ds402_quick_stop(pd4c_ctx_t* ctx, int8_t mode) {
    if (!ctx) return PD4C_EINVAL;
    /* Clear bit2 (quick stop) while keeping switch on + enable voltage + enable op bits as needed.
       Common "quick stop command" uses 0x000B; here we send 0x000B with mode.
     */
    return pd4c_rpdo1_send_control_mode(ctx, 0x000B, mode);
}

int pd4c_pp_move(pd4c_ctx_t* ctx,
                 int32_t target_pos,
                 uint16_t profile_vel,
                 int is_relative,
                 int timeout_ms) {
    if (!ctx) return PD4C_EINVAL;
    if (timeout_ms < 50) timeout_ms = 50;

    // Ensure we're operational enabled in PP mode
    int rc = pd4c_ds402_enable_operation(ctx, PD4C_MODE_PROFILE_POSITION, timeout_ms);
    if (rc != PD4C_OK) return rc;

    // Base controlword for PP in "operation enabled".
    // We set bit5 ("change set immediately") to avoid buffered semantics.
    // For relative moves, set bit6 (CiA402 PP abs/rel selection).
    uint16_t cw_base = (uint16_t)(0x000F | 0x0020);
    if (is_relative) cw_base |= 0x0040;  // bit6

    uint16_t cw_sp0 = (uint16_t)(cw_base & ~0x0010); // bit4 = 0
    uint16_t cw_sp1 = (uint16_t)(cw_base |  0x0010); // bit4 = 1

    // (1) Arm: bit4 low, wait set-point-ack low (bit12=0)
    // Some drives keep TPDO1 sparse while idle; in that case we continue with
    // a best-effort trigger sequence instead of aborting before sending RPDO2.
    rc = pd4c_rpdo1_send_control_mode(ctx, cw_sp0, PD4C_MODE_PROFILE_POSITION);
    if (rc != PD4C_OK) return rc;

    rc = wait_statusword_mask_eq(ctx, PD4C_SW_SETPOINT_ACK, 0, timeout_ms);
    if (rc != PD4C_OK) {
        if (rc == PD4C_ETIMEDOUT) {
            logf_ctx(ctx, PD4C_LOG_WARN,
                     "pp_move: setpoint-ack low not observed before trigger; continuing with best-effort pulse");
        } else {
            return rc;
        }
    }

    // (2) Send setpoint + profile vel via RPDO2
    rc = pd4c_rpdo2_send_pp(ctx, target_pos, profile_vel);
    if (rc != PD4C_OK) return rc;

    // (3) Rising edge: bit4 high, wait ack set (bit12=1)
    rc = pd4c_rpdo1_send_control_mode(ctx, cw_sp1, PD4C_MODE_PROFILE_POSITION);
    if (rc != PD4C_OK) return rc;

    rc = wait_statusword_mask_eq(ctx, PD4C_SW_SETPOINT_ACK, PD4C_SW_SETPOINT_ACK, timeout_ms);
    if (rc != PD4C_OK) {
        if (rc == PD4C_ETIMEDOUT) {
            logf_ctx(ctx, PD4C_LOG_WARN,
                     "pp_move: setpoint-ack high not observed after trigger; command was sent");
        } else {
            return rc;
        }
    }

    // (4) Clear bit4, wait ack clear (bit12=0) so next command works reliably
    rc = pd4c_rpdo1_send_control_mode(ctx, cw_sp0, PD4C_MODE_PROFILE_POSITION);
    if (rc != PD4C_OK) return rc;

    rc = wait_statusword_mask_eq(ctx, PD4C_SW_SETPOINT_ACK, 0, timeout_ms);
    if (rc != PD4C_OK) {
        if (rc == PD4C_ETIMEDOUT) {
            logf_ctx(ctx, PD4C_LOG_WARN,
                     "pp_move: setpoint-ack low not observed after clear; keeping command success for compatibility");
        } else {
            return rc;
        }
    }

    return PD4C_OK;
}


/* ---------------- misc helpers ---------------- */

double pd4c_deg_clamp(double deg, double min_deg, double max_deg) {
    if (deg < min_deg) return min_deg;
    if (deg > max_deg) return max_deg;
    return deg;
}
