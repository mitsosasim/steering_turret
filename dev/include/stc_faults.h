#ifndef STC_FAULTS_H
#define STC_FAULTS_H

#include <stdint.h>

int stc_detect_ds402_fault(uint16_t status);
int stc_detect_can_timeout(uint64_t now_ms, uint64_t last_rx_ms, uint32_t timeout_ms);

#endif
