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

typedef enum {
  STC_DIAG_MYACT_SEV_NONE = 0,
  STC_DIAG_MYACT_SEV_WARNING = 1,
  STC_DIAG_MYACT_SEV_CRITICAL = 2,
} stc_diag_myact_severity_t;

typedef enum {
  STC_DIAG_ERR_AXIS_FAULT = 0x0001,
  STC_DIAG_ERR_AXIS_STALE = 0x0002,
  STC_DIAG_ERR_MYACT_STALL = 0x0011,
  STC_DIAG_ERR_MYACT_UNDERVOLT = 0x0012,
  STC_DIAG_ERR_MYACT_OVERVOLT = 0x0013,
  STC_DIAG_ERR_MYACT_OVERCURRENT = 0x0014,
  STC_DIAG_ERR_MYACT_OVERSPEED = 0x0015,
  STC_DIAG_ERR_MYACT_ENCODER_CAL = 0x0016,
  STC_DIAG_ERR_MYACT_OVERTEMP = 0x0017,
  STC_DIAG_WARN_AXIS_SAT = 0x0100,
  STC_DIAG_WARN_MYACT_POWER_OVERRUN = 0x0111,
  STC_DIAG_WARN_MYACT_CAL_WRITE = 0x0112,
} stc_diag_error_code_t;

stc_diag_myact_severity_t stc_diag_myact_error_severity(uint16_t error_flags);
uint16_t stc_diag_myact_primary_error_code(uint16_t error_flags);
