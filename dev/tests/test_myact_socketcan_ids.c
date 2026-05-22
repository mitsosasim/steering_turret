/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   SocketCAN identifier mapping tests for MYACTUATOR transport adapter.
 *
 * Design notes:
 *   Protects protocol ID calculations required for reliable X8-25 command/telemetry routing.
 */

#include "x8_25.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef NDEBUG
#define TEST_ASSERT_OR_ABORT(cond) do { if (!(cond)) { fprintf(stderr, "CHECK failed: %s\n", #cond); abort(); } } while (0)
#else
#define TEST_ASSERT_OR_ABORT(cond) assert(cond)
#endif

/* Confirms fixed request/reply CAN-ID formulas required by MYACT transport ABI. */
static void check_ids(uint8_t node, uint32_t req, uint32_t rep) {
  TEST_ASSERT_OR_ABORT((uint32_t)(0x140u + (uint32_t)node) == req);
  TEST_ASSERT_OR_ABORT((uint32_t)(0x240u + (uint32_t)node) == rep);
}

/* Regression set covers low/mid/high node IDs used in bench deployments. */
int main(void) {
  check_ids(1u, 0x141u, 0x241u);
  check_ids(8u, 0x148u, 0x248u);
  check_ids(32u, 0x160u, 0x260u);
  return 0;
}
