#include "motion_backend.h"

int myact_motion_backend_start(const motion_backend_cfg_t*, motion_backend_t**);
void myact_motion_backend_stop(motion_backend_t*);
int myact_motion_backend_discover(motion_backend_t*);
int myact_motion_backend_apply_pp_move(motion_backend_t*, uint32_t, const double[STC_AXIS_COUNT]);
int myact_motion_backend_apply_speed(motion_backend_t*, uint32_t, const double[STC_AXIS_COUNT]);
int myact_motion_backend_quick_stop(motion_backend_t*, uint32_t);
int myact_motion_backend_clear_faults(motion_backend_t*, uint32_t);
int myact_motion_backend_set_laser(motion_backend_t*, uint8_t, uint8_t);
int myact_motion_backend_get_snapshot(motion_backend_t*, motion_snapshot_t*);

int pd4c_motion_backend_start(const motion_backend_cfg_t* cfg, motion_backend_t** out_backend) { return myact_motion_backend_start(cfg, out_backend); }
void pd4c_motion_backend_stop(motion_backend_t* backend) { myact_motion_backend_stop(backend); }
int pd4c_motion_backend_discover(motion_backend_t* backend) { return myact_motion_backend_discover(backend); }
int pd4c_motion_backend_apply_pp_move(motion_backend_t* backend, uint32_t axes_mask, const double targets_deg[STC_AXIS_COUNT]) { return myact_motion_backend_apply_pp_move(backend, axes_mask, targets_deg); }
int pd4c_motion_backend_apply_speed(motion_backend_t* backend, uint32_t axes_mask, const double speeds_deg_s[STC_AXIS_COUNT]) { return myact_motion_backend_apply_speed(backend, axes_mask, speeds_deg_s); }
int pd4c_motion_backend_quick_stop(motion_backend_t* backend, uint32_t axes_mask) { return myact_motion_backend_quick_stop(backend, axes_mask); }
int pd4c_motion_backend_clear_faults(motion_backend_t* backend, uint32_t axes_mask) { return myact_motion_backend_clear_faults(backend, axes_mask); }
int pd4c_motion_backend_set_laser(motion_backend_t* backend, uint8_t laser_id, uint8_t on_off) { return myact_motion_backend_set_laser(backend, laser_id, on_off); }
int pd4c_motion_backend_get_snapshot(motion_backend_t* backend, motion_snapshot_t* snapshot) { return myact_motion_backend_get_snapshot(backend, snapshot); }
