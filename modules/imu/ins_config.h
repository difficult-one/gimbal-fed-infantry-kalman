#ifndef INS_CONFIG_H
#define INS_CONFIG_H

#include "pid.h"

typedef struct INS_Init_Config {
    PID_Init_Config_s heater_pid;
    float heater_target_c, heater_period_ms, heater_initial_duty;
    float scale[3], yaw_deg, pitch_deg, roll_deg;
    float ekf_quaternion_noise, ekf_bias_noise, ekf_accel_noise;
    float ekf_fading, ekf_accel_lpf_s, motion_lpf_s;
    uint32_t sample_gap_us, yaw_aid_max_age_us;
    uint32_t initial_attitude_samples, initial_attitude_timeout_us;
    float initial_attitude_poll_s;
    uint32_t calibration_samples, calibration_timeout_us;
    float calibration_retry_s, calibration_poll_s;
    float calibration_accel_span, calibration_gyro_span;
    float calibration_gravity, calibration_gravity_tolerance;
    float gyro_offset_limit[3], fallback_gyro_offset[3];
    float fallback_gravity, fallback_temperature_c;
    uint8_t yaw_bias_aid_enabled;
} INS_Init_Config;

#endif
