#include "x8_25.h"
#include <assert.h>
#include <stdint.h>

static int send_stub(const myact_can_frame_t* frame, void* user) {(void)frame; (void)user; return MYACT_OK;}
static int recv_stub(myact_can_frame_t* frame, uint32_t timeout_ms, void* user) {(void)frame; (void)timeout_ms; (void)user; return MYACT_ETIMEDOUT;}
static uint64_t time_stub(void* user) {(void)user; return 0;}

static void check_ids(uint8_t node, uint32_t req, uint32_t rep) {
  myact_ctx_t ctx;
  myact_io_t io = {.send = send_stub, .recv = recv_stub, .time_ms = time_stub, .user = 0};
  assert(myact_init(&ctx, node, &io) == MYACT_OK);
  assert(ctx.cmd_id == req);
  assert(ctx.reply_id == rep);
}

int main(void) {
  check_ids(1u, 0x141u, 0x241u);
  check_ids(8u, 0x148u, 0x248u);
  return 0;
}
