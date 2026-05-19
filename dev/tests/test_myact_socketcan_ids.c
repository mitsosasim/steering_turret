#include "x8_25.h"
#include <assert.h>
#include <stdint.h>

static void check_ids(uint8_t node, uint32_t req, uint32_t rep) {
  assert((uint32_t)(0x140u + (uint32_t)node) == req);
  assert((uint32_t)(0x240u + (uint32_t)node) == rep);
}

int main(void) {
  check_ids(1u, 0x141u, 0x241u);
  check_ids(8u, 0x148u, 0x248u);
  check_ids(32u, 0x160u, 0x260u);
  return 0;
}
