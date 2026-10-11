/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Central version constants exported for protocol responses and build metadata.
 *
 * Design notes:
 *   Single source of truth for software version string and wire revision tags.
 */

#pragma once

/* Single source of truth for STC software version string. */
#define STC_SW_VERSION "1.0"

/* Wire-format revision for TLML/TLMH binary layout compatibility tracking. */
#define STC_PROTOCOL_WIRE_REVISION 2u
