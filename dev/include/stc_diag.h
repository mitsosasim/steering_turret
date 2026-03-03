#pragma once

#include <stdint.h>

uint32_t stc_diag_encode(uint8_t category,
                         uint8_t subcategory,
                         uint16_t error_code);
