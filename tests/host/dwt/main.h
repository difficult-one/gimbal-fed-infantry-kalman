#ifndef HOST_DWT_MAIN_H
#define HOST_DWT_MAIN_H
#include <stdint.h>
typedef struct { volatile uint32_t CTRL, CYCCNT; } HostDWT;
typedef struct { volatile uint32_t DEMCR; } HostCoreDebug;
extern HostDWT host_dwt;
extern HostCoreDebug host_core;
extern uint32_t host_primask;
#define DWT (&host_dwt)
#define CoreDebug (&host_core)
#define CoreDebug_DEMCR_TRCENA_Msk (1u<<24)
#define DWT_CTRL_CYCCNTENA_Msk 1u
static inline uint32_t __get_PRIMASK(void) { return host_primask; }
static inline void __disable_irq(void) { host_primask=1; }
static inline void __set_PRIMASK(uint32_t key) { host_primask=key; }
uint32_t HAL_GetTick(void);
#endif
