#include "stc_diag.h"

uint32_t stc_diag_encode(uint8_t category,
                         uint8_t subcategory,
                         uint16_t error_code) {
  return ((uint32_t)category << 24) |
         ((uint32_t)subcategory << 16) |
         ((uint32_t)error_code & 0xFFFFu);
}
