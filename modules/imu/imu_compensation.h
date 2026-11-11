#ifndef IMU_COMPENSATION_H
#define IMU_COMPENSATION_H

#include <stdint.h>

/* 无 HAL/任务依赖的数学层，便于用独立运动轨迹验证。全部内部量为 SI 单位。 */
#define IMU_MOTION_LPF_S 0.0085f
/* 约 20 ms 的短窗端点二次拟合；用实际时间戳，避免把任务抖动当成采样周期。 */
#define IMU_ALPHA_HISTORY 21u
/* A clock/input fault latches recovery. Motion observation is released only
 * after 500 ms of consecutive settled samples; attitude/yaw are never reset. */
typedef struct {
    uint32_t clock_generation, good_us;
    uint8_t pending;
} IMURecoveryState;
uint8_t IMURecoveryUpdate(IMURecoveryState *state, uint32_t generation,
                         uint32_t dt_us, uint8_t valid, uint8_t settled);
enum { IMU_RIGID_SEED=1u, IMU_RIGID_TIME=2u, IMU_RIGID_INPUT=4u,
       IMU_RIGID_ALPHA=8u, IMU_RIGID_DISABLED=16u };
/* 仅诊断：保存实际参加本帧计算的量，不改变差分/滤波链。 */
typedef struct {
    float alpha[3], tangent[3], centripetal[3];
    uint32_t dt_us, reject;
} IMURigidTrace;
typedef struct {
    uint32_t time_us;
    float last_gyro[3], center_b[3];
    uint8_t ready;
    IMURigidTrace trace;
    uint32_t gyro_time[IMU_ALPHA_HISTORY];
    float gyro_history[IMU_ALPHA_HISTORY][3];
    uint8_t gyro_count, gyro_head;
} IMURigidState;

/* 每约 1 s 汇总 INS 全采样率数据。平方和留到低优先级日志任务求 RMS。
 * alpha/tangent/centripetal 为传感器机体系；before/after/correction 为 INS 导航系 XY。
 * correction=before-after，cross=sum(before*correction)，可区分有效抵消与噪声注入。 */
typedef struct {
    uint32_t seq, start_us, end_us, samples, good, rejected, reject_mask;
    uint32_t dt_min_us, dt_max_us;
    float alpha2[3], alpha_peak[3], tangent2[3], centripetal2[3];
    float tangent_peak[3], centripetal_peak[3];
    float before2[2], after2[2], before_sum[2], after_sum[2], correction2[2], cross[2];
} IMURigidStats;
typedef struct { IMURigidStats window, published; } IMURigidStatsState;
void IMURigidStatsPush(IMURigidStatsState *s, uint32_t time_us,
                       const IMURigidTrace *trace, uint8_t valid,
                       const float before_n[3], const float after_n[3]);

/* 原因位与计数数组下标一致；同一帧可同时命中多项。计数保留短暂失败，
 * 防止 10 Hz/1 Hz 日志恰好漏掉打断 3 s 学习窗口的那一个控制周期。 */
typedef enum {
    IMU_AID_CALLER, IMU_AID_CHASSIS_MODE, IMU_AID_CHASSIS_CMD_TIME,
    IMU_AID_CHASSIS_CMD_MOTION, IMU_AID_WHEEL_OFFLINE, IMU_AID_WHEEL_TIME,
    IMU_AID_WHEEL_RAW, IMU_AID_WHEEL_FILTERED, IMU_AID_CHASSIS_TIME,
    IMU_AID_CALIBRATION, IMU_AID_DISABLED, IMU_AID_IMU, IMU_AID_YAW_FEEDBACK,
    IMU_AID_PITCH_FEEDBACK, IMU_AID_REMOTE, IMU_AID_SOURCE, IMU_AID_YAW_INPUT,
    IMU_AID_GIMBAL_MODE, IMU_AID_GIMBAL_CMD_TIME, IMU_AID_INPUT,
    IMU_AID_GRAVITY, IMU_AID_GYRO, IMU_AID_POSE, IMU_AID_TIMESTEP,
    IMU_AID_YAW_TARGET, IMU_AID_PITCH_TARGET, IMU_AID_PITCH_STEP,
    IMU_AID_PITCH_SPAN, IMU_AID_YAW_STEP, IMU_AID_IMU_STEP,
    IMU_AID_WINDOW_SPEED, IMU_AID_CAUSE_COUNT
} IMUYawAidCause;
#define IMU_AID_BIT(cause) (UINT32_C(1) << (cause))
typedef struct {
    uint32_t time_us, reason, last_reason, last_block_us, resets;
    uint32_t hits[IMU_AID_CAUSE_COUNT]; /* 各原因被拒绝的调用次数，非重置次数。 */
    float settle_s, window_s; /* 读取快照时填入，反映当前进度。 */
} IMUYawBiasDebug;

/* 两阶段接口：先用本帧物理角速度计算刚体项，再用去杆臂的加速度更新
 * 姿态 EKF；最后用同一姿态去重力、低通、投影。Prepare 不需要四元数。
 * trace.tangent/centripetal 仅在 Prepare 返回 1 时有效。
 * Project 的 center_b 是已去杆臂、已去重力、尚未低通的中心加速度，
 * 不会再次扣杆臂。两阶段必须每帧成对调用，不可跨帧混用。 */
uint8_t IMURigidPrepare(IMURigidState *s, uint32_t time_us,
                        const float gyro[3], const float lever_b[3]);
uint8_t IMURigidProject(IMURigidState *s, const float q[4],
                        const float center_b[3], float out_n[3]);
uint8_t IMURigidProjectFiltered(IMURigidState *s, const float q[4],
                                const float center_b[3], float lpf_s, float out_n[3]);

/* 兼容已有独立数学调用：内部依次 Prepare、扣杆臂、Project。
 * 在 INS 的原始采样频率执行：先去杆臂加速度，再与 INS 同系数低通。
 * motion_b 为本帧去重力、尚未软件低通的机体系加速度；out_n 为 INS 导航系。
 * 首帧、超时、非法四元数/角加速度返回 0，并重新建立连续历史。 */
uint8_t IMURigidUpdate(IMURigidState *s, uint32_t time_us, const float q[4],
                       const float gyro[3], const float motion_b[3],
                       const float lever_b[3], float out_n[3]);

typedef struct {
    uint32_t time_us;
    float q[4], gyro[3], accel_norm;
    float yaw_deg, encoder_deg, pitch_encoder_deg, yaw_ref_deg, pitch_ref_deg;
    uint8_t valid; /* 调用方保证底盘静止、命令/IMU/两个编码器新鲜、遥控在线。 */
    uint32_t inhibit; /* 上游拒绝原因；valid=0 且无原因时记 CALLER。 */
} IMUYawBiasInput;

typedef struct {
    float trim_z; /* 启动标定以外、额外从机体系 gz 减去的零偏，rad/s。 */
    float last_yaw, last_encoder, last_pitch, pitch_anchor, last_yaw_ref, last_pitch_ref;
    float settle_s, window_s, yaw_delta, encoder_delta, jacobian_sum;
    uint32_t time_us, updates;
    uint8_t ready, status; /* 0=冻结，1=等待/积累，2=已更新，3=达到修正上限。 */
    IMUYawBiasDebug debug;
} IMUYawBiasState;

/* 比较 IMU 航向变化与完整精度编码器变化，绝不把“摇杆为零”当作角速度为零。
 * 仅估计低频残余零偏，不修改目标角、不跳变四元数，运动/失联时冻结已有值。 */
void IMUYawBiasUpdate(IMUYawBiasState *s, const IMUYawBiasInput *in);

#endif
