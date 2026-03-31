#include "stc_diag.h"

#include <stddef.h>

uint32_t stc_diag_encode(uint8_t category,
                         uint8_t subcategory,
                         uint16_t error_code) {
  return ((uint32_t)category << 24) |
         ((uint32_t)subcategory << 16) |
         ((uint32_t)error_code & 0xFFFFu);
}

static bool g_pbit_pass = true;
static bool g_ibit_pass = true;

typedef struct {
  shared_data_t* shared;
  bool critical;
} cbit_slot_t;

static cbit_slot_t g_cbit_slots[8];

static cbit_slot_t* cbit_find_slot(shared_data_t* shared, bool create) {
  if (!shared) return NULL;

  cbit_slot_t* free_slot = NULL;
  for (size_t i = 0; i < sizeof(g_cbit_slots) / sizeof(g_cbit_slots[0]); i++) {
    if (g_cbit_slots[i].shared == shared) return &g_cbit_slots[i];
    if (!g_cbit_slots[i].shared && !free_slot) free_slot = &g_cbit_slots[i];
  }
  if (!create) return NULL;
  if (!free_slot) return NULL;
  free_slot->shared = shared;
  free_slot->critical = false;
  return free_slot;
}

bool run_pbit_startup(void) {
  return g_pbit_pass;
}

stc_diag_health_t run_cbit_cycle(shared_data_t* shared) {
  cbit_slot_t* slot = cbit_find_slot(shared, false);
  if (!slot) return STC_DIAG_HEALTH_OK;
  return slot->critical ? STC_DIAG_HEALTH_CRITICAL_FAULT : STC_DIAG_HEALTH_OK;
}

bool run_ibit_sequence(shared_data_t* shared) {
  (void)shared;
  return g_ibit_pass;
}

void stc_diag_set_pbit_result(bool pass) {
  g_pbit_pass = pass;
}

void stc_diag_set_ibit_result(bool pass) {
  g_ibit_pass = pass;
}

void stc_diag_set_cbit_critical(shared_data_t* shared, bool critical) {
  cbit_slot_t* slot = cbit_find_slot(shared, true);
  if (!slot) return;
  slot->critical = critical;
}
