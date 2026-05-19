#ifndef STC_FAULTS_H
#define STC_FAULTS_H

#include <stdint.h>

int stc_detect_ds402_fault(uint16_t status);
int stc_detect_can_timeout(uint64_t now_ms, uint64_t last_rx_ms, uint32_t timeout_ms);
int stc_is_within_startup_grace(uint64_t now_ms, uint64_t server_start_ms, uint32_t startup_grace_ms);

#endif
