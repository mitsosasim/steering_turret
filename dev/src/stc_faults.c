#include "stc_faults.h"

int stc_detect_ds402_fault(uint16_t status) {
  const uint16_t masked = (uint16_t)(status & 0x004Fu);
  return (masked == 0x0008u || masked == 0x000Fu) ? 1 : 0;
}

int stc_detect_can_timeout(uint64_t now_ms, uint64_t last_rx_ms, uint32_t timeout_ms) {
  if (now_ms < last_rx_ms) {
    return 0;
  }
  return (now_ms - last_rx_ms) >= (uint64_t)timeout_ms;
}

int stc_is_within_startup_grace(uint64_t now_ms, uint64_t server_start_ms, uint32_t startup_grace_ms) {
  if (startup_grace_ms == 0u) {
    return 0;
  }
  if (now_ms < server_start_ms) {
    return 1;
  }
  return (now_ms - server_start_ms) < (uint64_t)startup_grace_ms;
}
