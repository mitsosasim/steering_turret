/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Maps parsed INI axis configuration into normalized runtime axis_cfg_t entries.
 *
 * Design notes:
 *   Implementation ownership: stc_axis_cfg_map module used during startup only.
 *   Not real-time sensitive; called while building runtime configuration snapshot.
 *   Protocol/hardware-facing through driver/node mapping fields in axis_cfg_t.
 *   Caller owns input/output storage; function performs deterministic value copy/map.
 */

#ifndef STC_AXIS_CFG_MAP_H
#define STC_AXIS_CFG_MAP_H

#include "stc_config.h"
#include "stc_model.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void stc_axis_cfg_from_config(const stc_config_t* cfg, int axis, axis_cfg_t* out_cfg, uint8_t* out_axis_expected);

#ifdef __cplusplus
}
#endif

#endif
