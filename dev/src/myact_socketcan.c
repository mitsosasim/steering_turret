#define _POSIX_C_SOURCE 200809L

#include "myact_socketcan.h"

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

static uint64_t sockcan_time_ms(void* user) {
    (void)user;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}

static int sockcan_send(const myact_can_frame_t* fr, void* user) {
    myact_socketcan_t* io = (myact_socketcan_t*)user;
    struct can_frame c = {0};
    if (!fr || !io || io->fd < 0 || fr->dlc > 8u) return MYACT_EINVAL;
    c.can_id = fr->id & CAN_SFF_MASK;
    c.can_dlc = fr->dlc;
    memcpy(c.data, fr->data, 8);
    return (write(io->fd, &c, sizeof(c)) == (ssize_t)sizeof(c)) ? MYACT_OK : MYACT_EIO;
}

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
    return MYACT_OK;
}

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

    struct can_filter filters[2];
    filters[0].can_id = MYACT_REPLY_ID_BASE + node_id;
    filters[0].can_mask = CAN_SFF_MASK;
    filters[1].can_id = MYACT_MOTION_CMD_ID_BASE + node_id;
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
