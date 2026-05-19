/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Protocol-level regression tests for MYACTUATOR X8-25 frame encode/decode behavior.
 *
 * Design notes:
 *   Catches wire-format or checksum regressions that would break actuator interoperability.
 */

#include "x8_25.h"
#include <assert.h>
#include <string.h>

typedef struct { myact_can_frame_t tx; myact_can_frame_t rx[8]; int rx_n; int rx_i; uint64_t now; } bus_t;
static int send_cb(const myact_can_frame_t* f, void* u){bus_t*b=u; b->tx=*f; return MYACT_OK;}
static int recv_cb(myact_can_frame_t* f,uint32_t t,void* u){(void)t; bus_t*b=u; if(b->rx_i>=b->rx_n)return MYACT_ETIMEDOUT; *f=b->rx[b->rx_i++]; return MYACT_OK;}
static uint64_t time_cb(void* u){bus_t*b=u; return ++b->now;}

int main(void){
 bus_t b={0}; myact_io_t io={send_cb,recv_cb,time_cb,&b}; myact_ctx_t ctx; int32_t v; myact_basic_feedback_t fb;
 assert(myact_init(&ctx,1,&io)==MYACT_OK);
 assert(myact_speed_send(&ctx,1234)==MYACT_OK);
 assert(b.tx.id==0x141 && b.tx.data[0]==MYACT_CMD_SPEED);
 assert(b.tx.data[4]==(uint8_t)(1234 & 0xff));

 b.rx_n=1; b.rx[0]=(myact_can_frame_t){.id=0x241,.dlc=8,.data={MYACT_CMD_READ_ANGLE_MULTI,0,0,0,0x78,0x56,0x34,0x12}};
 assert(myact_read_angle_multi(&ctx,&v)==MYACT_OK && v==0x12345678);

 b.rx_i=0; b.rx_n=2; b.rx[0]=(myact_can_frame_t){.id=0x241,.dlc=8,.data={MYACT_CMD_STATUS1,0,0,1,0,0,0,0}};
 b.rx[1]=(myact_can_frame_t){.id=0x241,.dlc=8,.data={MYACT_CMD_SPEED,30,0x10,0,0x20,0,0x30,0}};
 assert(myact_speed(&ctx,50,&fb)==MYACT_OK);
 assert(fb.valid && fb.source_cmd==MYACT_CMD_SPEED && fb.angle_deg==0x30);

 b.rx_i=0; b.rx_n=1; b.rx[0]=(myact_can_frame_t){.id=0x242,.dlc=8,.data={MYACT_CMD_SPEED}};
 assert(myact_speed(&ctx,50,&fb)<0);

 assert(myact_active_reply_config(&ctx,0x55,true,10)<0);
 return 0;
}
