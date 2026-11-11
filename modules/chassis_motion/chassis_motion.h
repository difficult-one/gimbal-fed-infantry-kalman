#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include <stdint.h>
#include "kinematics.h"

typedef enum {
    MC_BASELINE = 0, MC_ESTIMATE_ONLY, MC_SHADOW,
    MC_ACTIVE_FIXED, MC_ACTIVE_SCHEDULED
} ChassisMotionMode;

enum {
    MC_EXIT_NONE = 0, MC_EXIT_DISABLED, MC_EXIT_DT, MC_EXIT_POSE,
    MC_EXIT_WHEEL, MC_EXIT_IMU, MC_EXIT_COVARIANCE, MC_EXIT_NUMERIC,
    MC_EXIT_RECOVERING, MC_EXIT_GEOMETRY, MC_EXIT_STEERING,
    MC_EXIT_CLOCK, MC_EXIT_OBSERVATION
};

/* Core configuration has no HAL, CAN, INS or DJI dependency. Configure at startup
 * or with actuators disabled; one motion instance per MCU, single control writer. */
typedef struct {
    ChassisKinematicsConfig wheels;
    float spin_wheel_trim_dps[4]; /* per-wheel empirical balance, pure rotation only */
    uint32_t version;
    ChassisMotionMode initial_mode;
    uint8_t imu_on_gimbal, imu_follows_pitch, use_center_accel, accel_enabled;
    int8_t algebra_y_sign, center_y_sign; /* +1 standard; -1 only legacy adapter */
    float imu_offset_m[2], yaw_center_m[2], accel_bias_n[2];
    float r_velocity[2], r_accel[2], q_jerk[2], p_velocity, p_accel, max_p_velocity;
    float rotation_r_gain, max_accel, max_alpha;
    float fixed_kp, schedule_kp, schedule_omega, max_kp, max_dv, dv_slew;
    float max_velocity, velocity_slew, max_omega, omega_slew, current_limit;
    float phase_lead_s, max_phase_lead, theta_rate_lpf, max_theta_rate;
    float dt_min, dt_max;
    float accel_consistency_window_s, accel_consistency_max_dv;
    float velocity_innovation_max; /* m/s: maximum published fused/wheel discrepancy */
    /* Separate control qualification; provisional engineering limits, not sensor acceptance criteria. */
    float control_stable_s, control_velocity_max, control_accel_residual_max, control_ramp_s;
    uint32_t sample_max_us, sample_skew_us, inertial_max_us;
    uint16_t recover_samples, log_decimation;
    uint8_t rotation_priority;
    uint8_t direct_translation; /* 1 bypasses max_velocity/velocity_slew; wheel limits still apply. */
} ChassisMotionConfig;
enum { MC_LIMIT_REF = 1u, MC_LIMIT_DV = 2u, MC_LIMIT_WHEEL = 4u };

/* 已由平台层取得的一致性快照。所有角度 rad，速度 m/s、rad/s，加速度 m/s²。 */
typedef struct {
    uint32_t time_us, imu_seq, imu_time_us, cmd_age_us;
    uint32_t wheel_seq[4], wheel_time_us[4];
    float motor_dps[4], raw_motor_dps[4], current_fb[4], current_send[4];
    float current_request[4];
    float imu_q[4], imu_accel_raw[3], imu_motion_b[3], imu_motion_n[3], imu_gyro_rad[3];
    float imu_center_n[3]; /* Center acceleration in N; project INS adapter sets center_y_sign=-1. */
    float theta_bg, psi_imu_n, accel_n[2], omega_imu;
    uint8_t wheel_online_mask, pose_valid, accel_valid, imu_center_valid;
    /* SWERVE: actual wheel azimuth in B, after encoder sign/zero calibration.
     * Same local clock as wheel/IMU; stale or offline steering prohibits drive. */
    float steer_rad[4];
    uint32_t steer_time_us[4];
    uint8_t steer_online_mask;
    /* Optional source diagnostics; accel_valid must stay false during source
     * recovery. A clock generation change invalidates the complete KF history. */
    uint32_t clock_generation, imu_recovery_good_us;
    float imu_gravity_b[3], imu_pitch_rad, imu_roll_rad, attitude_chi_ratio;
    uint8_t imu_recovering, attitude_flags;
} ChassisMotionInput;

/* Queued with the same seq as state; preserve the historical 444-byte state. */
typedef struct {
    uint32_t imu_time_us, clock_generation, recovery_good_us, rejected_windows;
    float gravity_b[3], pitch_rad, roll_rad, attitude_chi_ratio;
    float r_diag[4], delta_v_residual[2], window_s;
    float delta_v_partial[2], velocity_residual[2];
    uint8_t gate; /* 0=warming, 1=consistent, 2=conflict */
    uint8_t imu_recovering, attitude_flags, reject_reason;
    float control_good_s, control_blend, control_velocity_error, control_accel_residual;
    uint16_t control_inhibit;
    uint8_t control_ready;
} ChassisMotionObservationState;
enum { MC_REJECT_PREFIX=1u, MC_REJECT_WINDOW=2u, MC_REJECT_VELOCITY=4u };
enum {
    MC_CONTROL_ESTIMATE=1u, MC_CONTROL_SOURCE=2u, MC_CONTROL_VELOCITY=4u,
    MC_CONTROL_ACCEL=8u, MC_CONTROL_ACTUATOR=16u, MC_CONTROL_DISABLED=32u,
    MC_CONTROL_TRANSITION=64u, MC_CONTROL_SETTLING=128u, MC_CONTROL_MODE=256u
};

/* 独立正常对齐对象，不改变 robot_msg.h 的压缩通信协议。可从调试器查看。 */
typedef struct {
    uint32_t version, seq, time_us, cmd_age_us, log_drop;
    uint32_t wheel_seq[4], wheel_age_us[4], imu_age_us, imu_seq;
    float dt_s, theta_bg, psi_bn, psi_gn, omega_b, omega_imu;
    float motor_dps[4], raw_motor_dps[4], wheel_mps[4], wheel_residual[4];
    float current_fb[4], current_send[4], wheel_req_dps[4], wheel_eff_dps[4];
    float current_request[4];
    float imu_q[4], imu_accel_raw[3], imu_motion_b[3], imu_motion_n[3], imu_gyro_rad[3];
    float wheel_b[2], wheel_n[2], accel_center_n[2]; // 右手 B/N，不是 J 的旧代数 Y
    float x[4], p_diag[4], innovation[4]; // [vx, ax, vy, ay]，固定 N 系
    float hat_g[2], user_g[2], ref_g[2], error_g[2], dv_raw_g[2], dv_g[2], cmd_g[2]; // G系；旧车适配由algebra_y_sign指定
    float omega_ref, kp, wheel_scale; // wheel_scale：最终轮速约束对平移的保留比例/统一缩放比例
    uint8_t mode, estimate_valid, wheel_valid_mask, accel_used;
    uint8_t observation_mask, exit_reason, limit_flags, power_limit_active;
    uint8_t current_saturated_mask, accel_center_valid, reserved[2];
} ChassisMotionState;

/* 与运动日志同帧入队，不改变历史MCL1状态结构；输出提前角不属于姿态估计。 */
typedef struct {
    float theta_output_bg, theta_rate_bg, lead_rad;
    uint8_t rate_valid;
} ChassisMotionDirectionState;

/* 历史 MCL1 二进制帧定义，供旧日志解码/布局检查；当前 RTT 通道 0 改为文本。 */
#define MC_LOG_MAGIC 0x314c434du /* 字节序列 MCL1 */
typedef struct {
    uint32_t magic, payload_size, transport_drop;
    ChassisMotionState state;
} ChassisMotionLogFrame;

/* The project adapter applies the installed vehicle calibration after these defaults. */
enum { MC_ROTATE_CONTROL_KEY = 1u };

void ChassisMotionConfigDefault(ChassisMotionConfig *config, ChassisDriveType drive);
uint8_t ChassisMotionConfigure(const ChassisMotionConfig *config);
void ChassisMotionGetConfig(ChassisMotionConfig *config);
void ChassisMotionSetMode(ChassisMotionMode mode);
void ChassisMotionUpdate(const ChassisMotionInput *input);
void ChassisMotionControl(const float user_g[2], float omega, uint8_t enabled,
                          uint8_t control_key, float motor_ref_dps[4]);
void ChassisMotionRecord(void);
/* Called by the single control writer before queueing the current frame. */
void ChassisMotionVisitRecord(void (*visit)(const ChassisMotionState *,
                                            const ChassisMotionObservationState *));
/* 单消费者低优先级日志出口；返回 0 表示队列空，满时丢新帧并计数。 */
uint8_t ChassisMotionPopLog(ChassisMotionState *state, ChassisMotionDirectionState *direction);
uint8_t ChassisMotionPopDiagnosticLog(ChassisMotionState *state,
    ChassisMotionDirectionState *direction, ChassisMotionObservationState *observation);
void ChassisMotionGetObservationState(ChassisMotionObservationState *observation);
void ChassisMotionGetState(ChassisMotionState *state);
void ChassisMotionGetDirectionState(ChassisMotionDirectionState *direction);
void ChassisMotionGetWheelCommand(ChassisWheelCommand *command);

/* Planar rotation helper. Forward/Inverse are fixed-wheel compatibility APIs;
 * for SWERVE use ChassisKinematicsObserve/Command with actual steering angles. */
void ChassisMotionRotate(float theta, const float in[2], float out[2]);
/* xn 为传感器 +x 在 INS 导航系的方向；输出为姿态 G 系 +x 在固定 N 的航向。 */
float ChassisMotionHeading(const float xn[3]);
void ChassisMotionForward(const float twist[3], float wheel_mps[4]);
uint8_t ChassisMotionInverse(const float wheel_mps[4], float twist[3]);

#endif
