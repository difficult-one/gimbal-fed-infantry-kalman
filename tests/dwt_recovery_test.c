#include "bsp_dwt.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

HostDWT host_dwt;
HostCoreDebug host_core;
uint32_t host_primask;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
static void advance(uint32_t us)
{
    tick+=us/1000u;
    host_dwt.CYCCNT+=us*180u;
}
int main(void)
{
    DWT_ClockDebug debug;
    DWT_Init(180);
    uint32_t last=0;
    /* More than one hardware counter wrap, exact elapsed time and no faults. */
    for (unsigned i=1;i<=600;++i) {
        advance(100000);
        assert(DWT_GetTimeline_us()==i*100000ull);
        assert(fabs(DWT_GetDeltaT(&last)-.1)<1e-7);
    }
    DWT_GetClockDebug(&debug); assert(!debug.generation);
    /* Repeated 1ms deficits must accumulate across the monitor's 5ms checks. */
    DWT_Init(180);
    for (unsigned i=0;i<20;++i) { tick+=5; host_dwt.CYCCNT+=4000u*180u; (void)DWT_GetTimeline_us(); }
    DWT_GetClockDebug(&debug); assert(debug.generation>0);
    DWT_Init(180); last=0;
    advance(100000); (void)DWT_GetDeltaT(&last);
    uint64_t before=DWT_GetTimeline_us();
    host_core.DEMCR=0; host_dwt.CTRL=0; tick+=20;
    assert(DWT_GetTimeline_us()==before+20000);
    DWT_GetClockDebug(&debug);
    assert(debug.generation==1 && (debug.last_flags&3u)==3u);
    assert(debug.demcr==0 && debug.ctrl==0);
    assert(host_core.DEMCR&CoreDebug_DEMCR_TRCENA_Msk);
    assert(host_dwt.CTRL&DWT_CTRL_CYCCNTENA_Msk);
    assert(fabs(DWT_GetDeltaT(&last)-.02)<1e-7);
    advance(1000); assert(DWT_GetTimeline_us()==before+21000);
    /* Enabled bits alone cannot prove that CYCCNT advances. */
    before=DWT_GetTimeline_us(); tick+=10;
    assert(DWT_GetTimeline_us()>=before+9000);
    DWT_GetClockDebug(&debug);
    assert(debug.generation==2 && debug.last_flags==DWT_CLOCK_TICK_MISMATCH);
    /* An external raw counter reset must not appear as a ~24 s wrap. */
    before=DWT_GetTimeline_us(); host_dwt.CYCCNT=0; tick++;
    uint64_t after=DWT_GetTimeline_us();
    assert(after>=before && after-before<3000);
    DWT_GetClockDebug(&debug); assert(debug.generation==3);
    host_primask=1; (void)DWT_GetTimeline_us(); assert(host_primask==1); host_primask=0;
    /* HAL's independent 32-bit millisecond counter may wrap too. */
    tick=UINT32_MAX-3; DWT_Init(180); advance(10000);
    assert(DWT_GetTimeline_us()==10000);
    DWT_GetClockDebug(&debug); assert(!debug.generation);
    DWT_Init(180); host_dwt.CTRL=0; tick+=51600;
    assert(DWT_GetTimeline_us()==51600000ull);
    DWT_GetClockDebug(&debug); assert(debug.generation==1);
    puts("PASS: monotonic DWT timeline, counter/tick wraps, disabled/frozen/reset recovery, IRQ preservation");
    return 0;
}
