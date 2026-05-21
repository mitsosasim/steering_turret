/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Internal module implementation for STC runtime support services.
 *
 * Design notes:
 *   Comment-only documentation refactor; runtime behavior is unchanged.
 */

#include "stc_faults.h"

/* DS402 fault detection is bit-based: bit 3 indicates fault regardless of
 * other state bits in the statusword.
 */
int stc_detect_ds402_fault(uint16_t status) {
  return (status & 0x0008u) != 0u;
}

/* Stale telemetry detection is time-delta based so callers can apply uniform
 * timeout policy across transports and backend implementations.
 */
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
