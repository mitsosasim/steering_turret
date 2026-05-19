#include "motion_backend.h"
#include "stc_log.h"
#include <string.h>

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

int myact_motion_backend_start(const motion_backend_cfg_t*, motion_backend_t**);
void myact_motion_backend_stop(motion_backend_t*);
int myact_motion_backend_discover(motion_backend_t*);
int myact_motion_backend_apply_pp_move(motion_backend_t*, uint32_t, const double[STC_AXIS_COUNT]);
int myact_motion_backend_apply_motion(motion_backend_t*, uint32_t, const stc_axis_motion_kind_t[STC_AXIS_COUNT], const double[STC_AXIS_COUNT], const double[STC_AXIS_COUNT]);
int myact_motion_backend_apply_speed(motion_backend_t*, uint32_t, const double[STC_AXIS_COUNT]);
int myact_motion_backend_quick_stop(motion_backend_t*, uint32_t);
int myact_motion_backend_clear_faults(motion_backend_t*, uint32_t);
int myact_motion_backend_set_laser(motion_backend_t*, uint8_t, uint8_t);
int myact_motion_backend_get_snapshot(motion_backend_t*, motion_snapshot_t*);

typedef struct {
  const char* type;
  motion_backend_t* impl;
} routed_backend_t;

static routed_backend_t g_backend;
static int use_sim(void){
#ifdef SIMULATION
  return strcmp(g_backend.type, "simulator")==0;
#else
  return 0;
#endif
}

int motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend){
  if(!cfg||!out_backend) return -1;
  memset(&g_backend,0,sizeof(g_backend));
  g_backend.type = (cfg->backend[0] ? cfg->backend : "simulator");
  STC_LOG_INFO("backend_selected=%s can_if=%s node_id=%u tx=0x%03x rx=0x%03x watchdog_ms=%d telem_interval_ms=%d native_speed_mode=%d",
    g_backend.type,cfg->can_if?cfg->can_if:"",cfg->primary_node_id,cfg->primary_tx_id,cfg->primary_rx_id,cfg->speed_watchdog_ms,cfg->telemetry_interval_ms,cfg->experimental_native_speed_mode);
  int rc;
  if (strcmp(g_backend.type,"myactuator")==0 || strcmp(g_backend.type,"pd4c")==0) {
    rc = myact_motion_backend_start(cfg,&g_backend.impl);
  } else if (use_sim()) {
    rc = sim_motion_backend_start(cfg,&g_backend.impl);
  } else {
    STC_LOG_ERROR("unsupported backend requested: %s", g_backend.type ? g_backend.type : "(null)");
    return -1;
  }
  if(rc==0) *out_backend=g_backend.impl;
  return rc;
}
void motion_backend_stop(motion_backend_t* b){ (void)b; if(!g_backend.impl) return; if(use_sim()) sim_motion_backend_stop(g_backend.impl); else myact_motion_backend_stop(g_backend.impl); g_backend.impl=NULL; }
int motion_backend_discover(motion_backend_t* b){ (void)b; return use_sim()?sim_motion_backend_discover(g_backend.impl):myact_motion_backend_discover(g_backend.impl);} 
int motion_backend_apply_pp_move(motion_backend_t* b,uint32_t m,const double t[STC_AXIS_COUNT]){(void)b;return use_sim()?sim_motion_backend_apply_pp_move(g_backend.impl,m,t):myact_motion_backend_apply_pp_move(g_backend.impl,m,t);} 
int motion_backend_apply_motion(motion_backend_t* b,uint32_t m,const stc_axis_motion_kind_t k[STC_AXIS_COUNT],const double v[STC_AXIS_COUNT],const double s[STC_AXIS_COUNT]){(void)b;(void)s; if(use_sim()){double abs_t[STC_AXIS_COUNT]={0}; double sp[STC_AXIS_COUNT]={0}; for(size_t i=0;i<STC_AXIS_COUNT;i++){if(!(m&(1u<<i))) continue; if(k[i]==STC_AXIS_MOTION_SPEED) sp[i]=v[i]; else abs_t[i]=v[i];} if(sim_motion_backend_apply_speed(g_backend.impl,m,sp)==0) return sim_motion_backend_apply_pp_move(g_backend.impl,m,abs_t); return -1;} return myact_motion_backend_apply_motion(g_backend.impl,m,k,v,s);}
int motion_backend_apply_speed(motion_backend_t* b,uint32_t m,const double s[STC_AXIS_COUNT]){(void)b;return use_sim()?sim_motion_backend_apply_speed(g_backend.impl,m,s):myact_motion_backend_apply_speed(g_backend.impl,m,s);} 
int motion_backend_quick_stop(motion_backend_t* b,uint32_t m){(void)b;return use_sim()?sim_motion_backend_quick_stop(g_backend.impl,m):myact_motion_backend_quick_stop(g_backend.impl,m);} 
int motion_backend_clear_faults(motion_backend_t* b,uint32_t m){(void)b;return use_sim()?sim_motion_backend_clear_faults(g_backend.impl,m):myact_motion_backend_clear_faults(g_backend.impl,m);} 
int motion_backend_set_laser(motion_backend_t* b,uint8_t i,uint8_t o){(void)b;return use_sim()?sim_motion_backend_set_laser(g_backend.impl,i,o):myact_motion_backend_set_laser(g_backend.impl,i,o);} 
int motion_backend_get_snapshot(motion_backend_t* b,motion_snapshot_t* s){(void)b;return use_sim()?sim_motion_backend_get_snapshot(g_backend.impl,s):myact_motion_backend_get_snapshot(g_backend.impl,s);} 
