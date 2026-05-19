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

/* DS402 fault detection uses statusword state-mask bits (0x004F) and matches
 * standardized fault/fault-reaction patterns, instead of vendor-specific text.
 */
int stc_detect_ds402_fault(uint16_t status) {
  const uint16_t masked = (uint16_t)(status & 0x004Fu);
  return (masked == 0x0008u || masked == 0x000Fu) ? 1 : 0;
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
