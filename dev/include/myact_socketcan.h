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
