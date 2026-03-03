#include "stc_diag.h"

#include <assert.h>
#include <stdio.h>

int main(void) {
  assert(stc_diag_encode(0x12, 0x34, 0xABCDu) == 0x1234ABCDu);
  assert(stc_diag_encode(0xAA, 0x55, 0x0000u) == 0xAA550000u);
  assert(stc_diag_encode(0xAA, 0x55, 0xFFFFu) == 0xAA55FFFFu);

  puts("stc_diag_test: PASS");
  return 0;
}
