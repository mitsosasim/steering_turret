/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 */

#include "motion_backend.h"
#include "stc_log.h"

#include <string.h>

typedef enum {
  BACKEND_FAMILY_UNKNOWN = 0,
  BACKEND_FAMILY_SIMULATOR,
  BACKEND_FAMILY_HARDWARE_CAN,
  BACKEND_FAMILY_MYACT,
  BACKEND_FAMILY_MOCK,
} backend_family_t;

#ifdef SIMULATION
int sim_motion_backend_start(const motion_backend_cfg_t*, motion_backend_t**);
void sim_motion_backend_stop(motion_backend_t*);
int sim_motion_backend_discover(motion_backend_t*);
int sim_motion_backend_apply_pp_move(motion_backend_t*, uint32_t, const double[STC_AXIS_COUNT]);
int sim_motion_backend_apply_speed(motion_backend_t*, uint32_t, const double[STC_AXIS_COUNT]);
int sim_motion_backend_quick_stop(motion_backend_t*, uint32_t);
int sim_motion_backend_clear_faults(motion_backend_t*, uint32_t);
int sim_motion_backend_set_laser(motion_backend_t*, uint8_t, uint8_t);
int sim_motion_backend_get_snapshot(motion_backend_t*, motion_snapshot_t*);
#else
static int sim_motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out){ (void)cfg; (void)out; return -1; }
static void sim_motion_backend_stop(motion_backend_t* b){ (void)b; }
static int sim_motion_backend_discover(motion_backend_t* b){ (void)b; return -1; }
static int sim_motion_backend_apply_pp_move(motion_backend_t* b, uint32_t m, const double t[STC_AXIS_COUNT]){ (void)b; (void)m; (void)t; return -1; }
static int sim_motion_backend_apply_speed(motion_backend_t* b, uint32_t m, const double s[STC_AXIS_COUNT]){ (void)b; (void)m; (void)s; return -1; }
static int sim_motion_backend_quick_stop(motion_backend_t* b, uint32_t m){ (void)b; (void)m; return -1; }
static int sim_motion_backend_clear_faults(motion_backend_t* b, uint32_t m){ (void)b; (void)m; return -1; }
static int sim_motion_backend_set_laser(motion_backend_t* b, uint8_t i, uint8_t o){ (void)b; (void)i; (void)o; return -1; }
static int sim_motion_backend_get_snapshot(motion_backend_t* b, motion_snapshot_t* s){ (void)b; (void)s; return -1; }
#endif

static int mock_motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out){ (void)cfg; if (!out) return -1; *out = (motion_backend_t*)cfg; return 0; }
static void mock_motion_backend_stop(motion_backend_t* b){ (void)b; }
static int mock_motion_backend_discover(motion_backend_t* b){ (void)b; return 0; }
static int mock_motion_backend_apply_pp_move(motion_backend_t* b, uint32_t m, const double t[STC_AXIS_COUNT]){ (void)b; (void)m; (void)t; return 0; }
static int mock_motion_backend_apply_speed(motion_backend_t* b, uint32_t m, const double s[STC_AXIS_COUNT]){ (void)b; (void)m; (void)s; return 0; }
static int mock_motion_backend_quick_stop(motion_backend_t* b, uint32_t m){ (void)b; (void)m; return 0; }
static int mock_motion_backend_clear_faults(motion_backend_t* b, uint32_t m){ (void)b; (void)m; return 0; }
static int mock_motion_backend_set_laser(motion_backend_t* b, uint8_t i, uint8_t o){ (void)b; (void)i; (void)o; return 0; }
static int mock_motion_backend_get_snapshot(motion_backend_t* b, motion_snapshot_t* s){ (void)b; if (!s) return -1; memset(s, 0, sizeof(*s)); return 0; }

__attribute__((weak)) int can_motion_backend_start(const motion_backend_cfg_t*, motion_backend_t**);
__attribute__((weak)) void can_motion_backend_stop(motion_backend_t*);
__attribute__((weak)) int can_motion_backend_discover(motion_backend_t*);
__attribute__((weak)) int can_motion_backend_apply_pp_move(motion_backend_t*, uint32_t, const double[STC_AXIS_COUNT]);
__attribute__((weak)) int can_motion_backend_apply_motion(motion_backend_t*, uint32_t, const stc_axis_motion_kind_t[STC_AXIS_COUNT], const double[STC_AXIS_COUNT], const double[STC_AXIS_COUNT]);
__attribute__((weak)) int can_motion_backend_apply_speed(motion_backend_t*, uint32_t, const double[STC_AXIS_COUNT]);
__attribute__((weak)) int can_motion_backend_quick_stop(motion_backend_t*, uint32_t);
__attribute__((weak)) int can_motion_backend_clear_faults(motion_backend_t*, uint32_t);
__attribute__((weak)) int can_motion_backend_set_laser(motion_backend_t*, uint8_t, uint8_t);
__attribute__((weak)) int can_motion_backend_get_snapshot(motion_backend_t*, motion_snapshot_t*);
__attribute__((weak)) int can_motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend){ (void)cfg; (void)out_backend; return -1; }
__attribute__((weak)) void can_motion_backend_stop(motion_backend_t* backend){ (void)backend; }
__attribute__((weak)) int can_motion_backend_discover(motion_backend_t* backend){ (void)backend; return -1; }
__attribute__((weak)) int can_motion_backend_apply_pp_move(motion_backend_t* backend, uint32_t axes_mask, const double targets[STC_AXIS_COUNT]){ (void)backend; (void)axes_mask; (void)targets; return -1; }
__attribute__((weak)) int can_motion_backend_apply_speed(motion_backend_t* backend, uint32_t axes_mask, const double speeds[STC_AXIS_COUNT]){ (void)backend; (void)axes_mask; (void)speeds; return -1; }
__attribute__((weak)) int can_motion_backend_quick_stop(motion_backend_t* backend, uint32_t axes_mask){ (void)backend; (void)axes_mask; return -1; }
__attribute__((weak)) int can_motion_backend_clear_faults(motion_backend_t* backend, uint32_t axes_mask){ (void)backend; (void)axes_mask; return -1; }
__attribute__((weak)) int can_motion_backend_set_laser(motion_backend_t* backend, uint8_t laser_id, uint8_t on_off){ (void)backend; (void)laser_id; (void)on_off; return -1; }
__attribute__((weak)) int can_motion_backend_get_snapshot(motion_backend_t* backend, motion_snapshot_t* snapshot){ (void)backend; (void)snapshot; return -1; }

typedef struct {
  const char* type;
  backend_family_t family;
  motion_backend_t* impl;
} routed_backend_t;

static routed_backend_t g_backend;

int motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend){
  if(!cfg||!out_backend) return -1;
  memset(&g_backend,0,sizeof(g_backend));
  g_backend.type = (cfg->backend[0] ? cfg->backend : "simulator");

  int rc = -1;
  if (strcmp(g_backend.type, "simulator") == 0) {
    g_backend.family = BACKEND_FAMILY_SIMULATOR;
    rc = sim_motion_backend_start(cfg, &g_backend.impl);
  } else if (strcmp(g_backend.type, "hardware_can_backend") == 0) {
    g_backend.family = BACKEND_FAMILY_HARDWARE_CAN;
    rc = can_motion_backend_start(cfg, &g_backend.impl);
  } else if (strcmp(g_backend.type, "myact_x8_25") == 0) {
    g_backend.family = BACKEND_FAMILY_MYACT;
    rc = myact_motion_backend_start(cfg, &g_backend.impl);
  } else if (strcmp(g_backend.type, "mock") == 0 || strcmp(g_backend.type, "noop") == 0) {
    g_backend.family = BACKEND_FAMILY_MOCK;
    rc = mock_motion_backend_start(cfg, &g_backend.impl);
  } else {
    STC_LOG_ERROR("unsupported backend requested: %s", g_backend.type ? g_backend.type : "(null)");
    return -1;
  }

  if (g_backend.family == BACKEND_FAMILY_HARDWARE_CAN) {
    STC_LOG_INFO("backend_transport=%s can_if=%s watchdog_ms=%d telem_interval_ms=%d native_speed_mode=%d",
      g_backend.type,
      cfg->can_if ? cfg->can_if : "",
      cfg->speed_watchdog_ms,
      cfg->telemetry_interval_ms,
      cfg->experimental_native_speed_mode);
  } else {
    STC_LOG_INFO("backend_transport=%s can_if=%s node_id=%u tx=0x%03x rx=0x%03x watchdog_ms=%d telem_interval_ms=%d native_speed_mode=%d",
      g_backend.type,
      cfg->can_if ? cfg->can_if : "",
      cfg->primary_node_id,
      cfg->primary_tx_id,
      cfg->primary_rx_id,
      cfg->speed_watchdog_ms,
      cfg->telemetry_interval_ms,
      cfg->experimental_native_speed_mode);
  }

  if (rc == 0) *out_backend = g_backend.impl;
  return rc;
}

void motion_backend_stop(motion_backend_t* b){
  (void)b;
  if(!g_backend.impl) return;
  if (g_backend.family == BACKEND_FAMILY_SIMULATOR) sim_motion_backend_stop(g_backend.impl);
  else if (g_backend.family == BACKEND_FAMILY_HARDWARE_CAN) can_motion_backend_stop(g_backend.impl);
  else if (g_backend.family == BACKEND_FAMILY_MYACT) myact_motion_backend_stop(g_backend.impl);
  else if (g_backend.family == BACKEND_FAMILY_MOCK) mock_motion_backend_stop(g_backend.impl);
  else STC_LOG_ERROR("motion_backend_stop: unsupported backend family backend=%s", g_backend.type ? g_backend.type : "(null)");
  g_backend.impl=NULL;
}

int motion_backend_discover(motion_backend_t* b){ (void)b; if (g_backend.family == BACKEND_FAMILY_SIMULATOR) return sim_motion_backend_discover(g_backend.impl); if (g_backend.family == BACKEND_FAMILY_HARDWARE_CAN) return can_motion_backend_discover(g_backend.impl); if (g_backend.family == BACKEND_FAMILY_MYACT) return myact_motion_backend_discover(g_backend.impl); if (g_backend.family == BACKEND_FAMILY_MOCK) return mock_motion_backend_discover(g_backend.impl); STC_LOG_ERROR("motion_backend_discover: unsupported backend family backend=%s", g_backend.type ? g_backend.type : "(null)"); return -1; }
int motion_backend_apply_pp_move(motion_backend_t* b,uint32_t m,const double t[STC_AXIS_COUNT]){ (void)b; if (g_backend.family == BACKEND_FAMILY_SIMULATOR) return sim_motion_backend_apply_pp_move(g_backend.impl,m,t); if (g_backend.family == BACKEND_FAMILY_HARDWARE_CAN) return can_motion_backend_apply_pp_move(g_backend.impl,m,t); if (g_backend.family == BACKEND_FAMILY_MYACT) return myact_motion_backend_apply_pp_move(g_backend.impl,m,t); if (g_backend.family == BACKEND_FAMILY_MOCK) return mock_motion_backend_apply_pp_move(g_backend.impl,m,t); STC_LOG_ERROR("motion_backend_apply_pp_move: unsupported backend family backend=%s", g_backend.type ? g_backend.type : "(null)"); return -1; }
int motion_backend_apply_motion(motion_backend_t* b,uint32_t m,const stc_axis_motion_kind_t k[STC_AXIS_COUNT],const double v[STC_AXIS_COUNT],const double s[STC_AXIS_COUNT]){
  (void)b;
  if (g_backend.family == BACKEND_FAMILY_HARDWARE_CAN && can_motion_backend_apply_motion) {
    return can_motion_backend_apply_motion(g_backend.impl, m, k, v, s);
  }
  if (g_backend.family == BACKEND_FAMILY_MYACT) return myact_motion_backend_apply_motion(g_backend.impl, m, k, v, s);
  motion_snapshot_t snap;
  if(motion_backend_get_snapshot(g_backend.impl, &snap) != 0) return -1;

  uint32_t speed_mask = 0;
  uint32_t pos_mask = 0;
  double abs_t[STC_AXIS_COUNT]={0};
  double sp[STC_AXIS_COUNT]={0};
  for(size_t i=0;i<STC_AXIS_COUNT;i++){
    if(!(m&(1u<<i))) continue;
    switch(k[i]){
      case STC_AXIS_MOTION_SPEED:
        sp[i]=v[i];
        speed_mask |= (1u<<i);
        break;
      case STC_AXIS_MOTION_REL:
        abs_t[i]=(double)snap.pos_raw[i] + v[i];
        pos_mask |= (1u<<i);
        break;
      case STC_AXIS_MOTION_ABS:
      default:
        abs_t[i]=v[i];
        pos_mask |= (1u<<i);
        break;
    }
  }
  if(speed_mask && motion_backend_apply_speed(g_backend.impl,speed_mask,sp)!=0) return -1;
  if(pos_mask && motion_backend_apply_pp_move(g_backend.impl,pos_mask,abs_t)!=0) return -1;
  return 0;
}
int motion_backend_apply_speed(motion_backend_t* b,uint32_t m,const double s[STC_AXIS_COUNT]){(void)b; if (g_backend.family == BACKEND_FAMILY_SIMULATOR) return sim_motion_backend_apply_speed(g_backend.impl,m,s); if (g_backend.family == BACKEND_FAMILY_HARDWARE_CAN) return can_motion_backend_apply_speed(g_backend.impl,m,s); if (g_backend.family == BACKEND_FAMILY_MYACT) return myact_motion_backend_apply_speed(g_backend.impl,m,s); if (g_backend.family == BACKEND_FAMILY_MOCK) return mock_motion_backend_apply_speed(g_backend.impl,m,s); STC_LOG_ERROR("motion_backend_apply_speed: unsupported backend family backend=%s", g_backend.type ? g_backend.type : "(null)"); return -1; }
int motion_backend_quick_stop(motion_backend_t* b,uint32_t m){(void)b; if (g_backend.family == BACKEND_FAMILY_SIMULATOR) return sim_motion_backend_quick_stop(g_backend.impl,m); if (g_backend.family == BACKEND_FAMILY_HARDWARE_CAN) return can_motion_backend_quick_stop(g_backend.impl,m); if (g_backend.family == BACKEND_FAMILY_MYACT) return myact_motion_backend_quick_stop(g_backend.impl,m); if (g_backend.family == BACKEND_FAMILY_MOCK) return mock_motion_backend_quick_stop(g_backend.impl,m); STC_LOG_ERROR("motion_backend_quick_stop: unsupported backend family backend=%s", g_backend.type ? g_backend.type : "(null)"); return -1; }
int motion_backend_clear_faults(motion_backend_t* b,uint32_t m){(void)b; if (g_backend.family == BACKEND_FAMILY_SIMULATOR) return sim_motion_backend_clear_faults(g_backend.impl,m); if (g_backend.family == BACKEND_FAMILY_HARDWARE_CAN) return can_motion_backend_clear_faults(g_backend.impl,m); if (g_backend.family == BACKEND_FAMILY_MYACT) return myact_motion_backend_clear_faults(g_backend.impl,m); if (g_backend.family == BACKEND_FAMILY_MOCK) return mock_motion_backend_clear_faults(g_backend.impl,m); STC_LOG_ERROR("motion_backend_clear_faults: unsupported backend family backend=%s", g_backend.type ? g_backend.type : "(null)"); return -1; }
int motion_backend_set_laser(motion_backend_t* b,uint8_t i,uint8_t o){(void)b; if (g_backend.family == BACKEND_FAMILY_SIMULATOR) return sim_motion_backend_set_laser(g_backend.impl,i,o); if (g_backend.family == BACKEND_FAMILY_HARDWARE_CAN) return can_motion_backend_set_laser(g_backend.impl,i,o); if (g_backend.family == BACKEND_FAMILY_MYACT) return myact_motion_backend_set_laser(g_backend.impl,i,o); if (g_backend.family == BACKEND_FAMILY_MOCK) return mock_motion_backend_set_laser(g_backend.impl,i,o); STC_LOG_ERROR("motion_backend_set_laser: unsupported backend family backend=%s", g_backend.type ? g_backend.type : "(null)"); return -1; }
int motion_backend_get_snapshot(motion_backend_t* b,motion_snapshot_t* s){(void)b; if (g_backend.family == BACKEND_FAMILY_SIMULATOR) return sim_motion_backend_get_snapshot(g_backend.impl,s); if (g_backend.family == BACKEND_FAMILY_HARDWARE_CAN) return can_motion_backend_get_snapshot(g_backend.impl,s); if (g_backend.family == BACKEND_FAMILY_MYACT) return myact_motion_backend_get_snapshot(g_backend.impl,s); if (g_backend.family == BACKEND_FAMILY_MOCK) return mock_motion_backend_get_snapshot(g_backend.impl,s); STC_LOG_ERROR("motion_backend_get_snapshot: unsupported backend family backend=%s", g_backend.type ? g_backend.type : "(null)"); return -1; }
