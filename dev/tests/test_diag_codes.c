#include "stc_diag.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static uint8_t motor_status_flags(int axis_enabled, int axis_fault, int axis_stale, int saturated) {
  uint8_t flags = 0;
  if (axis_enabled) flags |= (1u << 0);
  if (axis_fault) flags |= (1u << 1);
  if (axis_stale) flags |= (1u << 2);
  if (saturated) flags |= (1u << 3);
  return flags;
}

int main(void) {
  assert(stc_diag_encode(0x12, 0x34, 0xABCDu) == 0x1234ABCDu);
  assert(stc_diag_encode(0xAA, 0x55, 0x0000u) == 0xAA550000u);
  assert(stc_diag_encode(0xAA, 0x55, 0xFFFFu) == 0xAA55FFFFu);

  uint8_t axis_cat = 3;
  uint8_t axis_subcat = 5;
  assert(stc_diag_encode(axis_cat, axis_subcat, 0x0001u) == 0x03050001u);

  assert(motor_status_flags(1, 0, 0, 0) == 0x01u);
  assert(motor_status_flags(1, 1, 0, 0) == 0x03u);
  assert(motor_status_flags(0, 0, 1, 0) == 0x04u);
  assert(motor_status_flags(0, 0, 0, 1) == 0x08u);
  assert(motor_status_flags(1, 1, 1, 1) == 0x0Fu);

  assert(stc_diag_myact_error_severity(0x0000u) == STC_DIAG_MYACT_SEV_NONE);
  assert(stc_diag_myact_error_severity(0x0010u) == STC_DIAG_MYACT_SEV_CRITICAL);
  assert(stc_diag_myact_error_severity(0x1000u) == STC_DIAG_MYACT_SEV_CRITICAL);
  assert(stc_diag_myact_primary_error_code(0x0000u) == 0u);
  assert(stc_diag_myact_primary_error_code(0x0010u) == STC_DIAG_ERR_MYACT_OVERCURRENT);
  assert(stc_diag_myact_primary_error_code(0x1000u) == STC_DIAG_ERR_MYACT_OVERTEMP);

  puts("test_diag_codes: PASS");
  return 0;
}
