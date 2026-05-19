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
