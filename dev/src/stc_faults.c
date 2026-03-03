#include "stc_faults.h"

int stc_detect_ds402_fault(uint16_t status) {
  return (status & 0x0008u) != 0;
}

int stc_detect_can_timeout(uint64_t now_ms, uint64_t last_rx_ms, uint32_t timeout_ms) {
  return (now_ms - last_rx_ms) >= (uint64_t)timeout_ms;
}
