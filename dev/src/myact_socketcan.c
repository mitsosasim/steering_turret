/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Linux SocketCAN transport binding for the MYACT X8 protocol API.
 *
 * Design notes:
 *   Owns one raw CAN socket with node-scoped receive filters.
 *   Binds one node's service and motion reply IDs to one interface.
 *   Transport layer only moves frames; x8_25.c owns protocol semantics.
 */

#define _POSIX_C_SOURCE 200809L

#include "myact_socketcan.h"
#include "stc_log.h"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define MYACT_CMD_ID_BASE 0x140u
#define MYACT_REPLY_ID_BASE 0x240u
#define MYACT_MOTION_CMD_ID_BASE 0x400u
#define MYACT_MOTION_REPLY_ID_BASE 0x500u

static uint64_t sockcan_time_ms(void* user) {
    (void)user;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}

/* Transport-only TX path: frame meaning is defined by the upper protocol layer. */
static int sockcan_send(const myact_can_frame_t* fr, void* user) {
    myact_socketcan_t* io = (myact_socketcan_t*)user;
    struct can_frame c = {0};
    if (!fr || !io || io->fd < 0 || fr->dlc > 8u) return MYACT_EINVAL;
    c.can_id = fr->id & CAN_SFF_MASK;
    c.can_dlc = fr->dlc;
    memcpy(c.data, fr->data, 8);
    STC_LOG_TRACE("MYACT TX id=0x%03X dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
                  (unsigned)c.can_id, (unsigned)c.can_dlc,
                  c.data[0], c.data[1], c.data[2], c.data[3],
                  c.data[4], c.data[5], c.data[6], c.data[7]);
    return (write(io->fd, &c, sizeof(c)) == (ssize_t)sizeof(c)) ? MYACT_OK : MYACT_EIO;
}

/* Transport-only RX path with bounded wait; parser decisions stay in x8_25.c. */
static int sockcan_recv(myact_can_frame_t* out, uint32_t to_ms, void* user) {
    myact_socketcan_t* io = (myact_socketcan_t*)user;
    struct pollfd p;
    struct can_frame c;
    if (!out || !io || io->fd < 0) return MYACT_EINVAL;
    p.fd = io->fd; p.events = POLLIN; p.revents = 0;
    int r = poll(&p, 1, (int)to_ms);
    if (r == 0) return MYACT_ETIMEDOUT;
    if (r < 0) return MYACT_EIO;
    if (read(io->fd, &c, sizeof(c)) != (ssize_t)sizeof(c)) return MYACT_EIO;
    out->id = c.can_id & CAN_SFF_MASK;
    out->dlc = c.can_dlc;
    memcpy(out->data, c.data, 8);
    /* Higher protocol layers decide whether RX frames are awaited replies or async telemetry. */
    STC_LOG_TRACE("MYACT RX id=0x%03X dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
                  (unsigned)out->id, (unsigned)out->dlc,
                  out->data[0], out->data[1], out->data[2], out->data[3],
                  out->data[4], out->data[5], out->data[6], out->data[7]);
    return MYACT_OK;
}

/* Open and bind one PF_CAN raw socket configured for a single MYACT node ID. */
int myact_socketcan_open(myact_socketcan_t* io, const char* ifname, uint8_t node_id) {
    if (!io || !ifname || !ifname[0] || node_id < MYACT_NODE_ID_MIN || node_id > MYACT_NODE_ID_MAX) return MYACT_EINVAL;
    memset(io, 0, sizeof(*io));
    io->fd = -1;

    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return MYACT_EIO;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { close(fd); return MYACT_EIO; }

    io->tx_single_id = MYACT_CMD_ID_BASE + node_id;
    io->rx_single_id = MYACT_REPLY_ID_BASE + node_id;
    io->tx_motion_id = MYACT_MOTION_CMD_ID_BASE + node_id;
    io->rx_motion_id = MYACT_MOTION_REPLY_ID_BASE + node_id;

    struct can_filter filters[2];
    filters[0].can_id = io->rx_single_id;
    filters[0].can_mask = CAN_SFF_MASK;
    filters[1].can_id = io->rx_motion_id;
    filters[1].can_mask = CAN_SFF_MASK;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, (socklen_t)sizeof(filters)) < 0) {
        close(fd);
        return MYACT_EIO;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) { close(fd); return MYACT_EIO; }

    io->fd = fd;
    io->node_id = node_id;
    io->io.send = sockcan_send;
    io->io.recv = sockcan_recv;
    io->io.time_ms = sockcan_time_ms;
    io->io.user = io;
    STC_LOG_INFO("MYACT socketcan axis0 node=%u tx_single=0x%03X rx_single=0x%03X tx_motion=0x%03X rx_motion=0x%03X",
                 (unsigned)node_id,
                 (unsigned)io->tx_single_id, (unsigned)io->rx_single_id,
                 (unsigned)io->tx_motion_id, (unsigned)io->rx_motion_id);
    return MYACT_OK;
}

void myact_socketcan_close(myact_socketcan_t* io) {
    if (!io) return;
    if (io->fd >= 0) close(io->fd);
    io->fd = -1;
}

int myact_socketcan_fd(const myact_socketcan_t* io) { return io ? io->fd : -1; }

int myact_socketcan_drain(myact_socketcan_t* io, myact_ctx_t* ctx, int max_frames) {
    if (!io || !ctx || max_frames < 0) return MYACT_EINVAL;
    int drained = 0;
    while (drained < max_frames) {
        int rc = myact_poll(ctx, 0);
        if (rc == MYACT_ETIMEDOUT) break;
        if (rc < 0) return rc;
        drained++;
    }
    return drained;
}

int myact_socketcan_drain_for(myact_socketcan_t* io, myact_ctx_t* ctx, int drain_ms) {
    if (!io || !ctx || drain_ms < 0) return MYACT_EINVAL;
    const uint64_t start = sockcan_time_ms(NULL);
    const uint64_t deadline = start + (uint64_t)drain_ms;
    int drained = 0;
    while (sockcan_time_ms(NULL) <= deadline) {
        int rc = myact_poll(ctx, 0);
        if (rc == MYACT_ETIMEDOUT) break;
        if (rc < 0) return rc;
        drained++;
    }
    return drained;
}
