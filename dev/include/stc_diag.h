#pragma once

#include <stdbool.h>
#include <stdint.h>

struct shared_data;
typedef struct shared_data shared_data_t;

typedef enum {
  STC_DIAG_HEALTH_OK = 0,
  STC_DIAG_HEALTH_CRITICAL_FAULT
} stc_diag_health_t;

uint32_t stc_diag_encode(uint8_t category,
                         uint8_t subcategory,
                         uint16_t error_code);

bool run_pbit_startup(void);
stc_diag_health_t run_cbit_cycle(shared_data_t* shared);
bool run_ibit_sequence(shared_data_t* shared);

void stc_diag_set_pbit_result(bool pass);
void stc_diag_set_ibit_result(bool pass);
void stc_diag_set_cbit_critical(shared_data_t* shared, bool critical);
