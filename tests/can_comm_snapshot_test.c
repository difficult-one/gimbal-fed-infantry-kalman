#include "can_comm.h"
#include "crc8.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint32_t now;
uint64_t DWT_GetTimeline_us(void) { return now; }
CANInstance *CANRegister(CAN_Init_Config_s *c)
{
    CANInstance *p=calloc(1,sizeof(*p));
    p->id=c->id; p->can_module_callback=c->can_module_callback;
    return p;
}
void CANSetDLC(CANInstance *p,uint8_t length) { p->tx_len=length; }
uint8_t CANTransmit(CANInstance *p,uint8_t timeout) { (void)p; (void)timeout; return 1; }
static void feed(CANCommInstance *p,const uint8_t *raw,unsigned length)
{
    for (unsigned offset=0;offset<length;) {
        unsigned n=length-offset>8 ? 8 : length-offset;
        memcpy(p->can_ins->rx_buff,raw+offset,n); p->can_ins->rx_len=n;
        p->can_ins->can_module_callback(p->can_ins); offset+=n;
    }
}
int main(void)
{
    CANComm_Init_Config_s c={0}; c.send_data_len=c.recv_data_len=20; c.daemon_count=10;
    CANCommInstance *p=CANCommInit(&c); assert(p);
    uint8_t input[20],output[20]={0},raw[24];
    uint32_t seq=0,time=123;
    assert(!CANCommGetSnapshot(p,output,sizeof(output),&seq,&time)); assert(time==123);
    for (unsigned i=0;i<20;++i) input[i]=3*i;
    raw[0]='s'; raw[1]=20; memcpy(raw+2,input,20); raw[22]=crc_8(input,20); raw[23]='e';
    now=UINT32_MAX-1000;
    feed(p,raw,8); /* Incomplete messages never refresh the valid snapshot. */
    assert(!CANCommGetSnapshot(p,output,sizeof(output),&seq,&time));
    feed(p,raw+8,16);
    assert(CANCommIsOnline(p));
    assert(!CANCommGetSnapshot(p,output,19,&seq,&time));
    assert(CANCommGetSnapshot(p,output,sizeof(output),&seq,&time));
    assert(seq==1 && time==now && !memcmp(input,output,20));
    uint32_t oldtime=time; now=5000;
    assert(!CANCommGetSnapshot(p,output,sizeof(output),&seq,&time)); assert(time==oldtime);
    raw[22]^=1; feed(p,raw,sizeof(raw));
    assert(!CANCommGetSnapshot(p,output,sizeof(output),&seq,&time)); assert(time==oldtime);
    for (unsigned i=0;i<10;++i) DaemonTask();
    assert(!CANCommIsOnline(p));
    raw[22]^=1; now=7000; feed(p,raw,sizeof(raw));
    assert(CANCommGetSnapshot(p,output,sizeof(output),&seq,&time)); assert(seq==2 && time==7000);
    assert(CANCommIsOnline(p));
    /* Legacy pointer reads do not consume the sequence-based interface. */
    now=9000; feed(p,raw,sizeof(raw)); (void)CANCommGet(p);
    assert(CANCommGetSnapshot(p,output,sizeof(output),&seq,&time)); assert(time==9000);
    p->recv_seq=UINT32_MAX; seq=UINT32_MAX;
    now=11000; feed(p,raw,sizeof(raw));
    assert(CANCommGetSnapshot(p,output,sizeof(output),&seq,&time)); assert(seq==0 && time==11000);
    c.recv_data_len=61; assert(!CANCommInit(&c));
    assert(!CANCommGetSnapshot(NULL,output,sizeof(output),&seq,&time));
    puts("PASS: fragmented CAN CRC snapshot, partial/corrupt rejection, frozen old timestamp, timeout/reconnect and sequence/time wrap");
}
