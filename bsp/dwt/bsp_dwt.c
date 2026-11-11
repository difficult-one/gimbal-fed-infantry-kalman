#include "bsp_dwt.h"
#include <string.h>

static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t last_raw, reference_tick;
static uint64_t cycles, reference_cycles;
static DWT_ClockDebug clock_debug;

/* All readers (ISR and tasks) share one accumulated cycle timeline. TIM14 is
 * checked every >=5 ms, with 2 ms allowance for tick quantization/ISR latency.
 * Repairs do not reset CYCCNT, rewind time or hide the discontinuity. */
static uint64_t ReadCycles(void)
{
    uint32_t key=__get_PRIMASK();
    __disable_irq();
    uint32_t raw=DWT->CYCCNT, tick=HAL_GetTick(), flags=0;
    uint32_t demcr=CoreDebug->DEMCR, ctrl=DWT->CTRL;
    if (!CPU_FREQ_Hz) { __set_PRIMASK(key); return 0; }
    if (!(demcr & CoreDebug_DEMCR_TRCENA_Msk)) flags|=DWT_CLOCK_TRACE_OFF;
    if (!(ctrl & DWT_CTRL_CYCCNTENA_Msk)) flags|=DWT_CLOCK_COUNTER_OFF;
    uint32_t elapsed_tick=(uint32_t)(tick-reference_tick);
    uint64_t next=cycles+(uint32_t)(raw-last_raw);
    uint64_t coarse=reference_cycles+(uint64_t)elapsed_tick*CPU_FREQ_Hz_ms;
    uint64_t tolerance=(uint64_t)CPU_FREQ_Hz_ms*2u;
    if ((elapsed_tick>=5u || (uint32_t)(raw-last_raw)>CPU_FREQ_Hz/10u) &&
        (next>coarse+tolerance || coarse>next+tolerance))
        flags|=DWT_CLOCK_TICK_MISMATCH;
    if (flags) {
        /* If HAL tick lagged too, preserve monotonicity without inventing time. */
        next=coarse>cycles ? coarse : cycles;
        CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
        clock_debug.generation++;
        clock_debug.last_flags=flags;
        clock_debug.last_event_us=(uint32_t)(next/CPU_FREQ_Hz_us);
    }
    cycles=next; last_raw=raw;
    /* Keep a longer reference: rebasing every check would hide repeated short
     * stalls that each fall inside the 2 ms quantization allowance. */
    if (elapsed_tick>=1000u || flags) {
        reference_cycles=cycles;
        reference_tick=tick;
    }
    /* Register values at the most recent fault are retained for diagnosis. */
    clock_debug.tick_ms=tick; clock_debug.cyccnt=raw;
    if (flags || !clock_debug.generation) {
        clock_debug.demcr=demcr; clock_debug.ctrl=ctrl;
    }
    uint64_t result=cycles;
    __set_PRIMASK(key);
    return result;
}

void DWT_Init(uint32_t mhz)
{
    uint32_t key=__get_PRIMASK();
    __disable_irq();
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    CPU_FREQ_Hz=mhz*1000000u; CPU_FREQ_Hz_ms=mhz*1000u; CPU_FREQ_Hz_us=mhz;
    cycles=reference_cycles=0; last_raw=0; reference_tick=HAL_GetTick();
    memset(&clock_debug,0,sizeof(clock_debug));
    __set_PRIMASK(key);
}

void DWT_GetClockDebug(DWT_ClockDebug *out)
{
    uint32_t key=__get_PRIMASK();
    __disable_irq();
    (void)ReadCycles();
    *out=clock_debug;
    __set_PRIMASK(key);
}

float DWT_GetDeltaT(uint32_t *last)
{
    uint32_t now=(uint32_t)ReadCycles(), elapsed=(uint32_t)(now-*last);
    *last=now;
    return CPU_FREQ_Hz ? (float)elapsed/(float)CPU_FREQ_Hz : 0;
}

double DWT_GetDeltaT64(uint32_t *last)
{
    uint32_t now=(uint32_t)ReadCycles(), elapsed=(uint32_t)(now-*last);
    *last=now;
    return CPU_FREQ_Hz ? (double)elapsed/(double)CPU_FREQ_Hz : 0;
}

void DWT_SysTimeUpdate(void) { (void)ReadCycles(); }
float DWT_GetTimeline_s(void) { return CPU_FREQ_Hz ? (float)((double)ReadCycles()/CPU_FREQ_Hz) : 0; }
float DWT_GetTimeline_ms(void) { return CPU_FREQ_Hz_ms ? (float)((double)ReadCycles()/CPU_FREQ_Hz_ms) : 0; }
uint64_t DWT_GetTimeline_us(void) { return CPU_FREQ_Hz_us ? ReadCycles()/CPU_FREQ_Hz_us : 0; }

void DWT_Delay(float seconds)
{
    uint64_t start=ReadCycles();
    uint64_t duration=seconds>0 ? (uint64_t)(seconds*(float)CPU_FREQ_Hz) : 0;
    while (ReadCycles()-start<duration) { }
}
void DWT_Delay_ms(float ms) { DWT_Delay(ms*.001f); }
