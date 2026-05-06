#include "x8_25.h"

#include <errno.h>
#include <inttypes.h>
#include <net/if.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

typedef struct { int s; char ifname[IFNAMSIZ]; } sockcan_bus_t;
#define MYACT_DEFAULT_MIN_MOTION_VOLTAGE_V 24.0

typedef struct {
    bool diag_only;
    bool motion;
    bool active_test;
    bool reset_first;
    bool force_motion_low_voltage;
    double min_motion_voltage;
} test_opts_t;

static uint64_t now_ms(void* u){struct timespec ts;(void)u;clock_gettime(CLOCK_MONOTONIC,&ts);return (uint64_t)ts.tv_sec*1000ULL+(uint64_t)(ts.tv_nsec/1000000ULL);} 
static void msleep(unsigned ms){usleep(ms*1000u);} 
static int sockcan_send_cb(const myact_can_frame_t* fr, void* user){sockcan_bus_t* b=(sockcan_bus_t*)user;struct can_frame c={0};if(!fr||!b||fr->dlc>8)return MYACT_EINVAL;c.can_id=fr->id & CAN_SFF_MASK;c.can_dlc=fr->dlc;memcpy(c.data,fr->data,8);return write(b->s,&c,sizeof(c))==(ssize_t)sizeof(c)?MYACT_OK:MYACT_EIO;}
static int sockcan_recv_cb(myact_can_frame_t* out,uint32_t to,void* user){sockcan_bus_t* b=(sockcan_bus_t*)user;struct pollfd p={.fd=b->s,.events=POLLIN};if(!out||!b)return MYACT_EINVAL;int r=poll(&p,1,(int)to);if(r==0)return MYACT_ETIMEDOUT;if(r<0)return MYACT_EIO;struct can_frame c;ssize_t n=read(b->s,&c,sizeof(c));if(n!=(ssize_t)sizeof(c))return MYACT_EIO;out->id=c.can_id & CAN_SFF_MASK;out->dlc=(uint8_t)c.can_dlc;memcpy(out->data,c.data,8);return MYACT_OK;}
static int sockcan_open(sockcan_bus_t* bus,const char* ifname){struct ifreq ifr;struct sockaddr_can addr;int s=socket(PF_CAN,SOCK_RAW,CAN_RAW);if(s<0)return -errno;memset(&ifr,0,sizeof(ifr));strncpy(ifr.ifr_name,ifname,IFNAMSIZ-1);if(ioctl(s,SIOCGIFINDEX,&ifr)<0){int e=-errno;close(s);return e;}memset(&addr,0,sizeof(addr));addr.can_family=AF_CAN;addr.can_ifindex=ifr.ifr_ifindex;if(bind(s,(struct sockaddr*)&addr,sizeof(addr))<0){int e=-errno;close(s);return e;}bus->s=s;return 0;}

static int prepare_sync(myact_ctx_t* ctx){int rc=myact_disable_all_active_replies(ctx);if(rc<0)return rc;rc=myact_drain_rx(ctx,300);if(rc<0)return rc;rc=myact_wait_quiet(ctx,300,1500);if(rc<0){fprintf(stderr,"active replies still present; synchronous test unsafe\n");}return rc;}
static void cleanup_send_only(myact_ctx_t* ctx){(void)myact_disable_all_active_replies(ctx);(void)myact_drain_rx(ctx,500);(void)myact_stop_send(ctx);usleep(100000);(void)myact_shutdown_send(ctx);if(myact_wait_quiet(ctx,300,1500)<0)fprintf(stderr,"warning: active replies may still be enabled; send 0x76 reset or power-cycle motor\n");}

static int read_identity(myact_ctx_t* ctx,myact_status1_t* s1){char model[8];uint32_t ver=0,rt=0;myact_runmode_t mode=MYACT_RUNMODE_UNKNOWN;int32_t mt=0;uint16_t st=0;int rc;
 if((rc=prepare_sync(ctx))<0) return rc;
 if((rc=myact_read_model(ctx,model))<0) return rc;
 if((rc=myact_read_version_date(ctx,&ver))<0) return rc;
 if((rc=myact_read_runtime(ctx,&rt))<0) return rc;
 if((rc=myact_read_operating_mode(ctx,&mode))<0) return rc;
 if((rc=myact_read_angle_multi(ctx,&mt))<0) return rc;
 if((rc=myact_read_angle_single(ctx,&st))<0) return rc;
 if((rc=myact_read_status1(ctx,s1))<0) return rc;
 printf("model:%s runmode:%u multi-turn:%.2f single-turn:%.2f bus:%.1fV\n",model,(unsigned)mode,myact_raw_0p01_to_deg(mt),st/100.0,s1->voltage_0p1v/10.0);
 return MYACT_OK;}

static int test_active_reply(myact_ctx_t* ctx){int rc=myact_active_reply_config(ctx,MYACT_CMD_STATUS2,true,10);if(rc<0)return rc;int got=0;while(got<3){int ev=myact_poll(ctx,300);if(ev==MYACT_ETIMEDOUT)continue;if(ev<0)return ev;if(ev&MYACT_EVENT_STATUS2)got++;}return myact_disable_all_active_replies(ctx);} 

int main(int argc,char** argv){ if(argc<3){fprintf(stderr,"usage: %s <can_if> <node_id> [--diag-only] [--motion] [--active-test] [--reset-first] [--min-motion-voltage v] [--force-motion-low-voltage]\n",argv[0]);return 2;} test_opts_t o={.diag_only=true,.min_motion_voltage=MYACT_DEFAULT_MIN_MOTION_VOLTAGE_V};
 for(int i=3;i<argc;++i){ if(!strcmp(argv[i],"--diag-only")){o.diag_only=true;o.motion=false;} else if(!strcmp(argv[i],"--motion")){o.motion=true;o.diag_only=false;} else if(!strcmp(argv[i],"--active-test"))o.active_test=true; else if(!strcmp(argv[i],"--reset-first"))o.reset_first=true; else if(!strcmp(argv[i],"--force-motion-low-voltage"))o.force_motion_low_voltage=true; else if(!strcmp(argv[i],"--min-motion-voltage")&&i+1<argc)o.min_motion_voltage=atof(argv[++i]); else {fprintf(stderr,"unknown option %s\n",argv[i]);return 2;}}
 sockcan_bus_t bus; int rc=sockcan_open(&bus,argv[1]); if(rc<0){fprintf(stderr,"open fail\n");return 1;} myact_io_t io={.send=sockcan_send_cb,.recv=sockcan_recv_cb,.time_ms=now_ms,.user=&bus}; myact_ctx_t ctx; rc=myact_init(&ctx,(uint8_t)atoi(argv[2]),&io); if(rc<0)return 1; myact_set_timeout_ms(&ctx,300);
 if(o.reset_first){(void)myact_system_reset(&ctx); msleep(2000);} 
 myact_status1_t s1={0}; rc=prepare_sync(&ctx); if(rc<0) goto done; rc=read_identity(&ctx,&s1); if(rc<0) goto done;
 if(o.motion){ if((s1.voltage_0p1v/10.0)<o.min_motion_voltage && !o.force_motion_low_voltage){ printf("MOTION TESTS SKIPPED: bus voltage too low for safe motion\n"); } else { rc=prepare_sync(&ctx); if(rc<0) goto done; rc=myact_brake_release(&ctx); if(rc==MYACT_ETIMEDOUT) fprintf(stderr,"brake release unsupported or no-brake variant\n"); else if(rc<0) goto done; myact_basic_feedback_t fb; rc=myact_torque(&ctx,myact_current_a_to_raw(0.20),&fb); if(rc<0) goto done; rc=myact_speed(&ctx,myact_speed_dps_to_raw(10.0),&fb); if(rc<0) goto done; rc=myact_position_abs(&ctx,30,myact_deg_to_raw_0p01(5.0),&fb); if(rc<0) goto done; }}
 if(o.active_test){ rc=test_active_reply(&ctx); if(rc<0) goto done; }
 done: cleanup_send_only(&ctx); close(bus.s); return rc<0?1:0; }
