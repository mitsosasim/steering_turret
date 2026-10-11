/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Small fault-detection helpers shared by runtime health checks.
 *
 * Design notes:
 *   Includes DS402 statusword fault-bit checks and stale CAN telemetry timeout
 *   detection with startup grace handling to avoid false positives at boot.
 */

#ifndef STC_FAULTS_H
#define STC_FAULTS_H

#include <stdint.h>

/** Return non-zero when DS402 statusword indicates a drive fault condition. */
int stc_detect_ds402_fault(uint16_t status);
/** Return non-zero when age(last_rx_ms) exceeds timeout_ms (stale CAN telemetry). */
int stc_detect_can_timeout(uint64_t now_ms, uint64_t last_rx_ms, uint32_t timeout_ms);
/** Return non-zero while runtime is inside startup grace window for stale suppression. */
int stc_is_within_startup_grace(uint64_t now_ms, uint64_t server_start_ms, uint32_t startup_grace_ms);

#endif
