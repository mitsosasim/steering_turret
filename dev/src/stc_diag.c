/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Implementation/declarations for the STC development codebase.
 *
 * Design notes:
 *   Documentation/comment ownership header only; no runtime behavior changes.
 */

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

stc_diag_myact_severity_t stc_diag_myact_error_severity(uint16_t error_flags) {
  if (error_flags == 0u) return STC_DIAG_MYACT_SEV_NONE;
  if (error_flags & (uint16_t)(0x0001u | 0x0002u | 0x0004u | 0x0010u | 0x0020u | 0x1000u | 0x2000u)) {
    return STC_DIAG_MYACT_SEV_CRITICAL;
  }
  if (error_flags & (uint16_t)(0x0100u | 0x0800u)) {
    return STC_DIAG_MYACT_SEV_WARNING;
  }
  return STC_DIAG_MYACT_SEV_WARNING;
}

uint16_t stc_diag_myact_primary_error_code(uint16_t error_flags) {
  if (error_flags & 0x0001u) return STC_DIAG_ERR_MYACT_STALL;
  if (error_flags & 0x0002u) return STC_DIAG_ERR_MYACT_UNDERVOLT;
  if (error_flags & 0x0004u) return STC_DIAG_ERR_MYACT_OVERVOLT;
  if (error_flags & 0x0010u) return STC_DIAG_ERR_MYACT_OVERCURRENT;
  if (error_flags & 0x0020u) return STC_DIAG_ERR_MYACT_OVERSPEED;
  if (error_flags & 0x2000u) return STC_DIAG_ERR_MYACT_ENCODER_CAL;
  if (error_flags & 0x1000u) return STC_DIAG_ERR_MYACT_OVERTEMP;
  if (error_flags & 0x0100u) return STC_DIAG_WARN_MYACT_POWER_OVERRUN;
  if (error_flags & 0x0800u) return STC_DIAG_WARN_MYACT_CAL_WRITE;
  return 0u;
}
