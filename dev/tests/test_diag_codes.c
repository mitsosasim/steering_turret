/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Diagnostic code encoding and motor-status flag mapping regression tests.
 *
 * Design notes:
 *   Ensures BIT outputs remain wire-compatible and that motor fault/state flags are interpreted consistently.
 */

#include "stc_diag.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* Locks wire-level diagnostic word layout (cat/subcat/code) and MYACT fault mapping consumed by external tooling. */
int main(void) {
  assert(stc_diag_encode(0x12, 0x34, 0xABCDu) == 0x1234ABCDu);
  assert(stc_diag_encode(0xAA, 0x55, 0x0000u) == 0xAA550000u);
  assert(stc_diag_encode(0xAA, 0x55, 0xFFFFu) == 0xAA55FFFFu);

  assert(stc_diag_encode(3u, 5u, 0x0001u) == 0x03050001u);


  assert(stc_diag_myact_error_severity(0x0000u) == STC_DIAG_MYACT_SEV_NONE);
  assert(stc_diag_myact_error_severity(0x0010u) == STC_DIAG_MYACT_SEV_CRITICAL);
  assert(stc_diag_myact_error_severity(0x1000u) == STC_DIAG_MYACT_SEV_CRITICAL);
  assert(stc_diag_myact_primary_error_code(0x0000u) == 0u);
  assert(stc_diag_myact_primary_error_code(0x0010u) == STC_DIAG_ERR_MYACT_OVERCURRENT);
  assert(stc_diag_myact_primary_error_code(0x1000u) == STC_DIAG_ERR_MYACT_OVERTEMP);

  puts("test_diag_codes: PASS");
  return 0;
}
