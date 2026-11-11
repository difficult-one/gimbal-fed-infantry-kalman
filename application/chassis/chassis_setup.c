#include "chassis_setup.h"
#include "config.h"
#include <string.h>

void ChassisMotionInit(void)
{
    const ChassisKinematicsConfig *wheels = &g_chassis_motion_config.wheels;
    ChassisDriveType profile = wheels->type == CK_CUSTOM ? CK_OMNI_X : wheels->type;
    /* The legacy custom profile began with the portable geometry in metres. */
    float base_m = g_chassis_config.wheel_base_mm * .001f;
    float track_m = g_chassis_config.track_width_mm * .001f;
    if (profile == CK_OMNI_X) {
        base_m = g_chassis_config.wheel_base_mm / 1000.0f;
        track_m = g_chassis_config.track_width_mm / 1000.0f;
    }
    ChassisMotionConfig config = g_chassis_motion_config;
    ChassisKinematicsProfile(&config.wheels, profile,
                             base_m, track_m,
                             g_chassis_config.wheel_radius_mm * .001f,
                             g_chassis_config.reduction_ratio);
    /* The deployed X-omni uses its historic rows and reflected Y convention. */
    if (wheels->type == CK_CUSTOM) {
        config.wheels.type = CK_CUSTOM;
        memcpy(config.wheels.rows, wheels->rows, sizeof(wheels->rows));
    }
    for (unsigned i = 0; i < 4; ++i) {
        config.wheels.radius_m[i] = wheels->radius_m[i];
        config.wheels.motor_sign[i] = wheels->motor_sign[i];
        config.wheels.gear_ratio[i] = wheels->gear_ratio[i];
        config.wheels.max_motor_dps[i] = wheels->max_motor_dps[i];
    }
    ChassisMotionCustomizeConfig(&config);
    (void)ChassisMotionConfigure(&config);
}

__attribute__((weak)) void ChassisMotionCustomizeConfig(ChassisMotionConfig *config)
{
    (void)config;
}
