#ifndef MOTION_PORT_H
#define MOTION_PORT_H

#include "chassis_motion.h"

typedef struct {
    float velocity_g_mps[2], omega_radps;
    uint8_t enabled, rotation_priority; /* priority also selects spin phase lead */
} ChassisMotionCommand;

/* Drivers publish one consistent snapshot, with timestamps in this MCU's clock.
 * CAN receive time is local; never subtract unsynchronised remote timestamps.
 * write must apply zero drive when enabled=0 and control steering enable itself. */
typedef struct {
    void *context;
    uint8_t (*read)(void *context, uint32_t now_us, ChassisMotionInput *snapshot);
    void (*write)(void *context, const ChassisWheelCommand *output, uint8_t enabled);
} ChassisMotionPort;

/* Same core as project adapter. No dependence on robot_def/RTOS/DJI/INS.
 * Returns whether the estimate is valid, not whether motors are enabled.
 * Sensor read failure always emits a disabled zero drive command. */
uint8_t ChassisMotionStep(const ChassisMotionPort *port, uint32_t now_us,
                          const ChassisMotionCommand *command);

#endif
