/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   SocketCAN transport adapter that binds x8_25 protocol callbacks to Linux CAN.
 *
 * Design notes:
 *   Exposes transport-facing abstraction owned by the x8_25 socketcan module.
 *   Not real-time policy itself; scheduling is owned by caller thread context.
 *   Hardware-facing via SocketCAN file descriptor, but protocol-neutral above x8_25.
 *   Caller owns object lifetime and any cross-thread synchronization.
 */

#ifndef MYACT_SOCKETCAN_H
#define MYACT_SOCKETCAN_H

#include "x8_25.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int fd;
    uint8_t node_id;
    uint32_t tx_single_id;
    uint32_t rx_single_id;
    uint32_t tx_motion_id;
    uint32_t rx_motion_id;
    myact_io_t io;
} myact_socketcan_t;

int myact_socketcan_open(myact_socketcan_t* io, const char* ifname, uint8_t node_id);
void myact_socketcan_close(myact_socketcan_t* io);
int myact_socketcan_fd(const myact_socketcan_t* io);
int myact_socketcan_drain(myact_socketcan_t* io, myact_ctx_t* ctx, int max_frames);

#ifdef __cplusplus
}
#endif

#endif
