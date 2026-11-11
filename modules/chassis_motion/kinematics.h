#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

#define CK_WHEELS 4u
typedef enum { CK_OMNI_X, CK_MECANUM_X, CK_SWERVE, CK_CUSTOM } ChassisDriveType;

/* SI/right-handed: B.x forward, B.y left, yaw CCW; order LF/RF/LB/RB.
 * CUSTOM may use a legacy algebra, but its coordinate conversion is the caller's job. */
typedef struct {
    ChassisDriveType type;
    float x_m[4], y_m[4];
    float radius_m[4], gear_ratio[4], motor_sign[4], max_motor_dps[4];
    float rows[4][3]; /* wheel contact speed = rows * body twist; fixed wheels only */
    uint8_t steer_cosine_compensation;
} ChassisKinematicsConfig;

typedef struct {
    ChassisKinematicsConfig config;
    float inverse[3][8];
    uint8_t valid;
} ChassisKinematics;

typedef struct {
    float motor_dps[4];
    float steer_rad[4]; /* continuous target nearest measured angle; radians at wheel */
} ChassisWheelCommand;

/* A profile supplies geometry, not motor IDs or steering encoder zero offsets. */
void ChassisKinematicsProfile(ChassisKinematicsConfig *config, ChassisDriveType type,
                              float wheelbase_m, float track_m, float radius_m, float ratio);
uint8_t ChassisKinematicsInit(ChassisKinematics *model, const ChassisKinematicsConfig *config);
float ChassisKinematicsMpsPerDps(const ChassisKinematics *model, unsigned wheel);
/* Uses actual steering feedback for SWERVE; all four modules required, no allocation. */
uint8_t ChassisKinematicsObserve(const ChassisKinematics *model, const float motor_dps[4],
                                 const float steer_rad[4], float twist[3]);
uint8_t ChassisKinematicsCommand(const ChassisKinematics *model, const float twist[3],
                                 const float measured_steer_rad[4], ChassisWheelCommand *command);
/* Speed feasibility ignores cosine reduction: do not borrow capacity from steering lag. */
float ChassisKinematicsScale(const ChassisKinematics *model, const float twist[3]);
float ChassisKinematicsTranslationScale(const ChassisKinematics *model,
                                       const float translation[2], float omega);

#endif
