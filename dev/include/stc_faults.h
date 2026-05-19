/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Implementation/declarations for the STC development codebase.
 *
 * Design notes:
 *   Documentation/comment ownership header only; no runtime behavior changes.
 */

#ifndef STC_FAULTS_H
#define STC_FAULTS_H

#include <stdint.h>

int stc_detect_ds402_fault(uint16_t status);
int stc_detect_can_timeout(uint64_t now_ms, uint64_t last_rx_ms, uint32_t timeout_ms);
int stc_is_within_startup_grace(uint64_t now_ms, uint64_t server_start_ms, uint32_t startup_grace_ms);

#endif
