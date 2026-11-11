#ifndef CONFIG_H
#define CONFIG_H

#include "build_config.h"
#include "pid.h"
#include "ins_config.h"
#include "chassis_motion.h"

/* 手写任务的优先级与 CMSIS-RTOS osPriority 对应，栈以 32 位字计。 */
typedef struct {
    int32_t priority;
    uint32_t stack_words, period_ticks;
} TaskSettings;

typedef struct {
    TaskSettings motor, daemon, robot, ins, motion_log;
} TaskConfig;

typedef struct {
    uint8_t remote_uart, vision_uart, vt02_uart, referee_uart;
    uint8_t board_can, super_cap_can, chassis_motor_can;
    uint16_t board_tx_id, board_rx_id, super_cap_tx_id, super_cap_rx_id;
    uint8_t board_daemon_count;
} LinkConfig;

typedef struct {
    int16_t chassis_dead_band, gimbal_dead_band;
    int16_t friction_dial, loader_dial, stop_dial;
    float rc_yaw_scale, rc_pitch_scale, rc_vx_scale, rc_vy_scale;
    float rc_shoot_rate, video_shoot_rate;
    float video_speed_scale, video_force_x_step, video_force_y_step;
    float video_force_mz_scale, video_force_max, video_mz_max;
    int mouse_full_scale, mouse_angle_deg;
    uint32_t yaw_feedback_max_age_us;
} InputConfig;

extern const TaskConfig g_task_config;
extern const LinkConfig g_link_config;
extern const InputConfig g_input_config;
extern const INS_Init_Config g_imu_config;

/* 固件接线和设备初始化模板使用真实驱动接口。 */
#include "dji_motor.h"
#include "jam_detector.h"
#include "jscope_rtt.h"

typedef struct {
    Motor_Init_Config_s yaw, pitch;
    uint8_t motor_can;
    uint16_t yaw_align_ecd, pitch_horizon_ecd;
    float pitch_min_deg, pitch_max_deg, yaw_max_track_error_deg;
    int8_t gyro_yaw_direction, gyro_pitch_direction, gyro_roll_direction;
    uint32_t yaw_aid_max_age_us;
} GimbalConfig;

typedef struct {
    Motor_Init_Config_s friction, loader;
    JamDetector_Init_Config_s jam;
    uint8_t motor_can, friction_right_id;
    Motor_Reverse_Flag_e friction_right_direction;
    float loader_reduction, loader_shot_angle_deg, loader_tolerance_deg;
    uint8_t shots_per_turn;
    float friction_15_dps, friction_18_dps, friction_30_dps, friction_default_dps;
} ShootConfig;

typedef struct {
    RTTDebugJScope4FConfig_t shoot;
    RTTDebugJScope6FConfig_t gimbal;
    void (*shoot_jscope_init)(const RTTDebugJScope4FConfig_t *);
    void (*shoot_jscope_send)(float, float, float, float);
    void (*gimbal_jscope_init)(const RTTDebugJScope6FConfig_t *);
    void (*gimbal_jscope_send)(float, float, float, float, float, float);
    uint8_t shoot_debug_enabled;
    uint16_t shoot_debug_period;
} LogConfig;

extern const GimbalConfig g_gimbal_config;
extern const ShootConfig g_shoot_config;
extern const LogConfig g_log_config;
/* 底盘尺寸、执行器和旧协议转换，轮序 LF/RF/LB/RB。 */
typedef struct {
    float wheel_base_mm, track_width_mm, wheel_radius_mm, reduction_ratio;
    float center_offset_x_mm, center_offset_y_mm;
    float mps_per_motor_dps, legacy_omega_scale, legacy_spin_command, legacy_omni_center;
    float command_trim_deg, follow_gain, follow_limit, velocity_command_limit, offset_angle_limit_deg;
    uint32_t command_timeout_us;
    uint8_t command_timeout_cycles, motor_tx_id[4], motor_reversed[4];
    PID_Init_Config_s speed_pid, current_pid, power_buffer_pid;
} ChassisConfig;

typedef struct {
    uint8_t on_gimbal, follows_pitch;
    float gimbal_lever_m[3], chassis_lever_m[3]; /* INS 传感器系。 */
    float heading_offset_rad, yaw_encoder_sign;
    float still_command_max, still_raw_dps, still_filtered_dps, min_horizontal_heading;
} ChassisImuConfig;

typedef struct {
    uint32_t frame_revision, capture_duration_us;
    unsigned text_rtt_channel, capture_rtt_channel;
    uint16_t force_log_period_cycles;
} ChassisLogConfig;

extern const ChassisConfig g_chassis_config;
extern const ChassisImuConfig g_chassis_imu_config;
extern const ChassisLogConfig g_chassis_log_config;
extern const ChassisMotionConfig g_chassis_motion_config;

typedef struct {
    float wheel_radius_m;
    float gear_ratio;
    float drive_efficiency;
    float motor_kt_nm_per_a;
    float current_raw_per_a;
    float current_limit_raw;
    float current_slew_raw;
    float speed_damping_raw_per_dps;
    float speed_damping_limit_raw;
    float release_brake_raw_per_dps;
    float release_brake_limit_raw;
    float speed_limit_dps;
    float overspeed_brake_raw;
    float start_translation_n;
    float start_moment_nm;
    float start_speed_dps;
    float start_exit_speed_dps;
    uint16_t start_exit_confirm_cycles;
    uint16_t start_hold_cycles;
    uint16_t start_retry_cooldown_cycles;
    float start_wheel_comp_raw;
    float start_wheel_min_raw;
    float start_input_deadband_n;
    float start_input_deadband_nm;
    float pseudoinverse_damping;
    float current_filter_alpha;
    float gyro_damping_nm_per_dps;
    float gyro_damping_limit_nm;
    float input_max_n;
    float input_max_nm;
    float admittance_mass_x;
    float admittance_mass_y;
    float admittance_mass_z;
    float admittance_damping_x;
    float admittance_damping_y;
    float admittance_damping_z;
    float admittance_max_vx;
    float admittance_max_vy;
    float admittance_max_wz;
    uint16_t sensor_timeout_cycles;
    float current_offset_raw[4]; /* LF/RF/LB/RB，DJI CAN 原始电流零偏。 */
    float rows[4][3]; /* LF/RF/LB/RB，底盘力/力矩分配雅可比。 */
    float admittance_dt_min_s, admittance_dt_max_s, admittance_dt_fallback_s;
    PID_Init_Config_s force_xy_pid, force_yaw_pid;
} ChassisForceConfig;

extern const ChassisForceConfig g_chassis_force_config;

#endif
