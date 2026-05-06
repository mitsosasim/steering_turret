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

enum {
    MOTION_TORQUE = 1u << 0,
    MOTION_SPEED = 1u << 1,
    MOTION_INC = 1u << 2
};

typedef struct {
    bool motion;
    bool motion_torque;
    bool motion_speed;
    bool motion_inc;
    bool motion_all;
    bool no_motion_countdown;
    bool force_motion_brake;
    bool force_motion_low_voltage;
    bool active_test;
    bool reset_first;
    bool raw_motion_smoke;
    double min_motion_voltage;
} test_opts_t;

static uint64_t now_ms(void* u){struct timespec ts;(void)u;clock_gettime(CLOCK_MONOTONIC,&ts);return (uint64_t)ts.tv_sec*1000ULL+(uint64_t)(ts.tv_nsec/1000000ULL);} 
static void msleep(unsigned ms){usleep(ms*1000u);} 
static int sockcan_send_cb(const myact_can_frame_t* fr, void* user){sockcan_bus_t* b=(sockcan_bus_t*)user;struct can_frame c={0};if(!fr||!b||fr->dlc>8)return MYACT_EINVAL;c.can_id=fr->id & CAN_SFF_MASK;c.can_dlc=fr->dlc;memcpy(c.data,fr->data,8);return write(b->s,&c,sizeof(c))==(ssize_t)sizeof(c)?MYACT_OK:MYACT_EIO;}
static int sockcan_recv_cb(myact_can_frame_t* out,uint32_t to,void* user){sockcan_bus_t* b=(sockcan_bus_t*)user;struct pollfd p={.fd=b->s,.events=POLLIN};if(!out||!b)return MYACT_EINVAL;int r=poll(&p,1,(int)to);if(r==0)return MYACT_ETIMEDOUT;if(r<0)return MYACT_EIO;struct can_frame c;ssize_t n=read(b->s,&c,sizeof(c));if(n!=(ssize_t)sizeof(c))return MYACT_EIO;out->id=c.can_id & CAN_SFF_MASK;out->dlc=(uint8_t)c.can_dlc;memcpy(out->data,c.data,8);return MYACT_OK;}
static int sockcan_open(sockcan_bus_t* bus,const char* ifname){struct ifreq ifr;struct sockaddr_can addr;int s=socket(PF_CAN,SOCK_RAW,CAN_RAW);if(s<0)return -errno;memset(&ifr,0,sizeof(ifr));strncpy(ifr.ifr_name,ifname,IFNAMSIZ-1);if(ioctl(s,SIOCGIFINDEX,&ifr)<0){int e=-errno;close(s);return e;}memset(&addr,0,sizeof(addr));addr.can_family=AF_CAN;addr.can_ifindex=ifr.ifr_ifindex;if(bind(s,(struct sockaddr*)&addr,sizeof(addr))<0){int e=-errno;close(s);return e;}bus->s=s;return 0;}

static int prepare_sync(myact_ctx_t* ctx){int rc=myact_disable_all_active_replies(ctx);if(rc<0)return rc;rc=myact_drain_rx(ctx,300);if(rc<0)return rc;rc=myact_wait_quiet(ctx,300,1500);if(rc<0)fprintf(stderr,"active replies still present; synchronous test unsafe\n");return rc;}
static void cleanup_send_only(myact_ctx_t* ctx){(void)myact_disable_all_active_replies(ctx);(void)myact_drain_rx(ctx,500);(void)myact_stop_send(ctx);usleep(100000);(void)myact_shutdown_send(ctx);} 

static int read_diagnostics(myact_ctx_t* ctx,myact_status1_t* s1,int32_t* mt){char model[8]={0};uint32_t ver=0,rt=0;myact_runmode_t mode=MYACT_RUNMODE_UNKNOWN;uint16_t st=0;int32_t e60=0,e61=0,e62=0;myact_status2_t s2={0};myact_status3_t s3={0};int rc;
 if((rc=myact_read_model(ctx,model))<0) return rc;
 if((rc=myact_read_version_date(ctx,&ver))<0) return rc;
 if((rc=myact_read_runtime(ctx,&rt))<0) return rc;
 if((rc=myact_read_operating_mode(ctx,&mode))<0) return rc;
 if((rc=myact_read_angle_multi(ctx,mt))<0) return rc;
 if((rc=myact_read_angle_single(ctx,&st))<0) return rc;
 if((rc=myact_read_encoder_multi(ctx,&e60))<0) return rc;
 if((rc=myact_read_encoder_raw(ctx,&e61))<0) return rc;
 if((rc=myact_read_encoder_offset(ctx,&e62))<0) return rc;
 if((rc=myact_read_status1(ctx,s1))<0) return rc;
 if((rc=myact_read_status2(ctx,&s2))<0) return rc;
 if((rc=myact_read_status3(ctx,&s3))<0) return rc;
 printf("diag model:%s ver:%u runtime_ms:%u runmode:%u\\n",model,ver,rt,(unsigned)mode);
 printf("diag angle_multi:%.2f angle_single:%.2f enc60:%d enc61:%d enc62:%d\\n",myact_raw_0p01_to_deg(*mt),st/100.0,e60,e61,e62);
 printf("diag status1 brake:%s bus:%.1fV err:0x%04X\\n",s1->brake_release_command_state?"released":"locked",s1->voltage_0p1v/10.0,s1->error_flags);
 return MYACT_OK;}

static int motion_preflight(myact_ctx_t* ctx,test_opts_t* o,myact_status1_t* s1){int rc;if((rc=prepare_sync(ctx))<0)return rc;if((rc=myact_read_status1(ctx,s1))<0)return rc;if(!s1->brake_release_command_state){(void)myact_brake_release(ctx);msleep(300);if((rc=myact_read_status1(ctx,s1))<0)return rc;}if(!s1->brake_release_command_state&&!o->force_motion_brake){printf("MOTION SKIPPED: brake release command state is still locked/off\n");return MYACT_ESTATE;}if((s1->voltage_0p1v/10.0)<o->min_motion_voltage&&!o->force_motion_low_voltage){printf("MOTION SKIPPED: bus voltage %.1f V below configured minimum %.1f V\n",s1->voltage_0p1v/10.0,o->min_motion_voltage);return MYACT_ESTATE;}return MYACT_OK;}

static int run_motion_tests(myact_ctx_t* ctx,unsigned mask){myact_basic_feedback_t fb;int32_t mt2=0;int rc=MYACT_OK;if(mask&MOTION_TORQUE){rc=myact_torque(ctx,myact_current_a_to_raw(0.20),&fb);if(rc<0)return rc;msleep(300);(void)myact_stop_send(ctx);}if(mask&MOTION_SPEED){rc=myact_speed(ctx,myact_speed_dps_to_raw(10.0),&fb);if(rc<0)return rc;msleep(700);(void)myact_stop_send(ctx);}if(mask&MOTION_INC){rc=myact_position_inc(ctx,20,myact_deg_to_raw_0p01(2.0),&fb);if(rc<0)return rc;msleep(1200);rc=myact_read_angle_multi(ctx,&mt2);if(rc<0)return rc;printf("post-inc angle_multi: %.2f deg\n",myact_raw_0p01_to_deg(mt2));}return MYACT_OK;}

int main(int argc,char** argv){ if(argc<3){fprintf(stderr,"usage: %s <can_if> <node_id> [--motion] [--motion-torque|--motion-speed|--motion-inc|--motion-all] [--raw-motion-smoke]\n",argv[0]);return 2;} test_opts_t o={.min_motion_voltage=MYACT_DEFAULT_MIN_MOTION_VOLTAGE_V};
 for(int i=3;i<argc;++i){ if(!strcmp(argv[i],"--motion"))o.motion=true; else if(!strcmp(argv[i],"--motion-torque"))o.motion_torque=true; else if(!strcmp(argv[i],"--motion-speed"))o.motion_speed=true; else if(!strcmp(argv[i],"--motion-inc"))o.motion_inc=true; else if(!strcmp(argv[i],"--motion-all"))o.motion_all=true; else if(!strcmp(argv[i],"--no-motion-countdown"))o.no_motion_countdown=true; else if(!strcmp(argv[i],"--force-motion-brake"))o.force_motion_brake=true; else if(!strcmp(argv[i],"--force-motion-low-voltage"))o.force_motion_low_voltage=true; else if(!strcmp(argv[i],"--active-test"))o.active_test=true; else if(!strcmp(argv[i],"--reset-first"))o.reset_first=true; else if(!strcmp(argv[i],"--raw-motion-smoke"))o.raw_motion_smoke=true; else if(!strcmp(argv[i],"--min-motion-voltage")&&i+1<argc)o.min_motion_voltage=atof(argv[++i]); else {fprintf(stderr,"unknown option %s\n",argv[i]);return 2;}}
 sockcan_bus_t bus; int rc=sockcan_open(&bus,argv[1]); if(rc<0){fprintf(stderr,"open fail\n");return 1;} myact_io_t io={.send=sockcan_send_cb,.recv=sockcan_recv_cb,.time_ms=now_ms,.user=&bus}; myact_ctx_t ctx; rc=myact_init(&ctx,(uint8_t)atoi(argv[2]),&io); if(rc<0)return 1; myact_set_timeout_ms(&ctx,300);
 if(o.reset_first){(void)myact_system_reset(&ctx); msleep(2000);} 
 myact_status1_t s1={0};int32_t mt=0; rc=prepare_sync(&ctx); if(rc<0) goto done; rc=read_diagnostics(&ctx,&s1,&mt); if(rc<0) goto done;
 if(!o.motion&&!o.raw_motion_smoke){printf("DIAG-ONLY mode: no motion requested. Use --motion to run motion tests.\n");rc=0;goto done;}
 if(o.raw_motion_smoke){int32_t before=mt,after=0;rc=motion_preflight(&ctx,&o,&s1);if(rc<0){rc=0;goto done;}rc=myact_position_inc(&ctx,10,myact_deg_to_raw_0p01(1.0),NULL);if(rc<0)goto done;msleep(1000);rc=myact_read_angle_multi(&ctx,&after);if(rc<0)goto done;printf("raw-motion-smoke delta: %.2f deg\n",myact_raw_0p01_to_deg(after-before));rc=0;goto done;}
 unsigned mask=0;if(o.motion_all)mask=MOTION_TORQUE|MOTION_SPEED|MOTION_INC;else{if(o.motion_torque)mask|=MOTION_TORQUE;if(o.motion_speed)mask|=MOTION_SPEED;if(o.motion_inc)mask|=MOTION_INC;if(mask==0)mask=MOTION_INC;}
 printf("motion plan: angle_multi=%.2f deg bus=%.1fV brake=%s test=0x%X target:inc +2.00deg@20dps torque:+0.20A speed:+10dps\n",myact_raw_0p01_to_deg(mt),s1.voltage_0p1v/10.0,s1.brake_release_command_state?"released":"locked",mask);
 if(!o.no_motion_countdown)msleep(2000);
 rc=motion_preflight(&ctx,&o,&s1);if(rc<0){rc=0;goto done;}
 rc=run_motion_tests(&ctx,mask); if(rc<0) goto done;
 done: cleanup_send_only(&ctx); close(bus.s); return rc<0?1:0; }
