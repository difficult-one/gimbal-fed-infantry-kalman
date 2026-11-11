#include "motion_port.h"
#include <string.h>
#include <math.h>

uint8_t ChassisMotionStep(const ChassisMotionPort *port, uint32_t now,
                          const ChassisMotionCommand *command)
{
    if (!port || !port->write) return 0;
    ChassisMotionInput input={0};
    ChassisWheelCommand output;
    float motor[4];
    uint8_t received=port->read && port->read(port->context,now,&input);
    input.time_us=now;
    if (!received) { memset(&input,0,sizeof(input)); input.time_us=now; input.cmd_age_us=UINT32_MAX; }
    ChassisMotionUpdate(&input);
    const float zero[2]={0};
    ChassisMotionConfig config;
    ChassisMotionGetConfig(&config);
    uint8_t enabled=received && command && command->enabled &&
        isfinite(command->velocity_g_mps[0]) && isfinite(command->velocity_g_mps[1]) &&
        isfinite(command->omega_radps) && isfinite(input.theta_bg) &&
        input.cmd_age_us<=config.sample_max_us && input.wheel_online_mask==15u;
    for (unsigned i=0;i<4;++i)
        if (!isfinite(input.motor_dps[i]) ||
            (uint32_t)(now-input.wheel_time_us[i])>config.sample_max_us) enabled=0;
    ChassisMotionControl(command ? command->velocity_g_mps : zero,
                         command ? command->omega_radps : 0, enabled,
                         command && command->rotation_priority ? 1 : 0,motor);
    ChassisMotionGetWheelCommand(&output);
    ChassisMotionState state;
    ChassisMotionGetState(&state);
    if (state.exit_reason==MC_EXIT_DT || state.exit_reason==MC_EXIT_GEOMETRY ||
        state.exit_reason==MC_EXIT_STEERING) enabled=0;
    if (!enabled) memset(output.motor_dps,0,sizeof(output.motor_dps));
    port->write(port->context,&output,enabled);
    ChassisMotionRecord();
    return state.estimate_valid;
}
