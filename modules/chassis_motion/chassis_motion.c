/**
 * @file chassis_motion.c
 * @brief 速控私有链：轮速逆解 → 固定 N 系 KF → G 系比例补偿 → SI 运动学。
 * 不改公共命令单位，不参与力控/导纳；电机启停权限始终由 chassis.c 决定。
 */
#include "chassis_motion.h"
#include "kalman_filter.h"
#include <math.h>
#include <string.h>
#ifndef MC_HOST_TEST
#include "cmsis_compiler.h"
#endif

static KalmanFilter_t kf;
static ChassisMotionState s;
static ChassisMotionConfig config;
static ChassisKinematics model;
static ChassisWheelCommand wheel_command;
static float measured_steer[4];
static uint8_t steering_valid, allocated;
static uint8_t geometry_valid, initialized, clock_valid, have_velocity, previous_enabled;
static uint8_t control_key_last, mode_last, derivative_valid;
static uint32_t last_time, last_wheel_seq[4], last_imu_seq, last_velocity_time, derivative_time;
static uint32_t last_accel_time;
static uint8_t have_accel;
static float last_omega_b, last_omega_imu;
static uint16_t recovery_count;
static volatile uint8_t requested_mode = MC_SHADOW;
#define MC_LOG_DEPTH 8u
static ChassisMotionState logs[MC_LOG_DEPTH];
static ChassisMotionDirectionState direction, direction_logs[MC_LOG_DEPTH];
static ChassisMotionObservationState observation, observation_logs[MC_LOG_DEPTH];
static uint32_t source_clock_generation, consistency_time;
static float consistency_elapsed, consistency_velocity[2], consistency_accel[2], consistency_integral[2];
static uint8_t consistency_seeded;
static float consistency_residual_s, consistency_full_accel_error;
static uint8_t control_new_pair, control_have_pair;
static uint32_t control_last_seq, control_pair_time;
static float last_theta_bg;
static uint8_t theta_rate_seeded;
static volatile uint32_t log_write, log_read;

static uint32_t Lock(void)
{
#ifdef MC_HOST_TEST
    return 0;
#else
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
#endif
}
static void Unlock(uint32_t key)
{
#ifdef MC_HOST_TEST
    (void)key;
#else
    __set_PRIMASK(key);
#endif
}

static float Clamp(float x, float limit)
{
    return fmaxf(-limit, fminf(x, limit));
}

/* 模长限幅和斜坡保留向量方向，不能分别夹紧 x/y 而改变指令方向。 */
static uint8_t LimitVector(float v[2], float limit)
{
    float norm = hypotf(v[0], v[1]);
    if (norm > limit && norm > 0.0f)
    {
        v[0] *= limit / norm;
        v[1] *= limit / norm;
        return 1u;
    }
    return 0u;
}
static void SlewVector(const float target[2], float previous[2], float step)
{
    float delta[2] = {target[0] - previous[0], target[1] - previous[1]};
    LimitVector(delta, step);
    previous[0] += delta[0];
    previous[1] += delta[1];
}

float ChassisMotionHeading(const float xn[3])
{
    /* N=(INS_n.x,-INS_n.y,-INS_n.z)，因此 yaw 的 y 分量取负。
     * 航向连接轮速与 IMU 两条独立观测，禁止按安装图前向额外翻转一次。 */
    return atan2f(-xn[1],xn[0]); /* INS adapter only; portable clients supply psi_imu_n. */
}

void ChassisMotionRotate(float theta, const float in[2], float out[2])
{
    float c = cosf(theta), sn = sinf(theta);
    float x = c * in[0] - sn * in[1];
    out[1] = sn * in[0] + c * in[1];
    out[0] = x; // 允许 in 与 out 指向同一数组。
}

void ChassisMotionForward(const float twist[3], float wheels[4])
{
    for (uint8_t i = 0; i < 4; ++i)
        wheels[i] = model.config.rows[i][0] * twist[0] + model.config.rows[i][1] * twist[1] + model.config.rows[i][2] * twist[2];
}

uint8_t ChassisMotionInverse(const float wheels[4], float twist[3])
{
    if (!geometry_valid || model.config.type==CK_SWERVE) return 0u;
    for (uint8_t r = 0; r < 3; ++r)
    {
        twist[r] = 0.0f;
        for (uint8_t i = 0; i < 4; ++i) twist[r] += model.inverse[r][i] * wheels[i];
        if (!isfinite(twist[r])) return 0u;
    }
    return 1u;
}

static float MpsPerDps(uint8_t i)
{
    return ChassisKinematicsMpsPerDps(&model,i);
}

static void ResetFilter(const float velocity[2], const float accel[2])
{
    memset(kf.P_data, 0, 16 * sizeof(float));
    for (uint8_t i = 0; i < 4; ++i)
    {
        kf.xhat_data[i] = (i & 1u) ? accel[i / 2] : velocity[i / 2];
        kf.P_data[i * 4 + i] = (i & 1u) ? config.p_accel : config.p_velocity;
    }
    recovery_count = 0u;
    initialized = 1u;
}

void ChassisMotionConfigDefault(ChassisMotionConfig *c, ChassisDriveType drive)
{
    memset(c,0,sizeof(*c));
    ChassisKinematicsProfile(&c->wheels,drive,.35f,.30f,.06f,19);
    c->version=0x20261110u; c->initial_mode=MC_SHADOW;
    c->use_center_accel=c->accel_enabled=1;
    c->algebra_y_sign=c->center_y_sign=1;
    for (unsigned i=0;i<2;++i) { c->r_velocity[i]=.0025f; c->r_accel[i]=.04f; c->q_jerk[i]=.1f; }
    c->p_velocity=.04f; c->p_accel=.25f; c->max_p_velocity=.25f;
    c->rotation_r_gain=.04f; c->max_accel=30; c->max_alpha=100;
    c->fixed_kp=.1f; c->schedule_kp=.1f; c->schedule_omega=2; c->max_kp=.3f;
    c->max_dv=.1f; c->dv_slew=.5f; c->max_velocity=1; c->velocity_slew=1;
    c->max_omega=9; c->omega_slew=10; c->current_limit=8000;
    /* Delay is vehicle-specific; portable profile deliberately defaults to zero. */
    c->max_phase_lead=.20f; c->theta_rate_lpf=.020f; c->max_theta_rate=40;
    c->dt_min=.0005f; c->dt_max=.020f;
    c->accel_consistency_window_s=.25f; c->accel_consistency_max_dv=.04f;
    c->velocity_innovation_max=.04f;
    c->control_stable_s=1.0f; c->control_velocity_max=.01f;
    c->control_accel_residual_max=.10f; c->control_ramp_s=.5f;
    c->sample_max_us=20000; c->sample_skew_us=10000; c->inertial_max_us=100000;
    c->recover_samples=20; c->log_decimation=20; c->rotation_priority=1;
}

uint8_t ChassisMotionConfigure(const ChassisMotionConfig *c)
{
    /* 与 INS 同样使用现有 KalmanFilter_t，仅初始化时分配，控制周期不分配内存。 */
    if (!allocated) { Kalman_Filter_Init(&kf, 4, 0, 4); allocated=1; }
    kf.UseAutoAdjustment = 0; // 0 m/s 和 0 m/s² 都是合法观测。
    kf.FirstMatError=ARM_MATH_SUCCESS;
    memset(&s, 0, sizeof(s));
    s.wheel_scale = 1.0f;
    geometry_valid=0;
    initialized=clock_valid=have_velocity=have_accel=previous_enabled=derivative_valid=0;
    recovery_count=0; log_write=log_read=0;
    memset(last_wheel_seq,0,sizeof(last_wheel_seq)); last_imu_seq=0;
    memset(&direction, 0, sizeof(direction));
    memset(&observation,0,sizeof(observation));
    consistency_seeded=0; consistency_elapsed=0; source_clock_generation=0;
    consistency_residual_s=consistency_full_accel_error=0;
    control_new_pair=control_have_pair=0; control_last_seq=control_pair_time=0;
    memset(&wheel_command,0,sizeof(wheel_command));
    memset(measured_steer,0,sizeof(measured_steer)); steering_valid=0;
    theta_rate_seeded = 0u;
    if (!c) return 0;
    config=*c;
    s.version=c->version; requested_mode=c->initial_mode;
    const float positive[]={c->r_velocity[0],c->r_velocity[1],c->r_accel[0],c->r_accel[1],
        c->q_jerk[0],c->q_jerk[1],c->p_velocity,c->p_accel,c->max_p_velocity,c->max_accel,
        c->max_alpha,c->schedule_omega,c->max_kp,c->max_dv,c->dv_slew,c->max_velocity,
        c->velocity_slew,c->max_omega,c->omega_slew,c->current_limit,c->max_phase_lead,
        c->max_theta_rate,c->dt_min,c->dt_max,c->accel_consistency_window_s,c->accel_consistency_max_dv,
        c->velocity_innovation_max,c->control_stable_s,c->control_velocity_max,
        c->control_accel_residual_max,c->control_ramp_s};
    for (unsigned i=0;i<sizeof(positive)/sizeof(positive[0]);++i)
        if (!isfinite(positive[i]) || positive[i]<=0) return 0;
    const float nonnegative[]={c->fixed_kp,c->schedule_kp,c->rotation_r_gain,c->phase_lead_s,c->theta_rate_lpf};
    for (unsigned i=0;i<sizeof(nonnegative)/sizeof(nonnegative[0]);++i)
        if (!isfinite(nonnegative[i]) || nonnegative[i]<0) return 0;
    for (unsigned i=0;i<4;++i)
        if (!isfinite(c->spin_wheel_trim_dps[i]) || fabsf(c->spin_wheel_trim_dps[i])>500.0f) return 0;
    for (unsigned i=0;i<2;++i)
        if (!isfinite(c->imu_offset_m[i]) || !isfinite(c->yaw_center_m[i]) || !isfinite(c->accel_bias_n[i])) return 0;
    if (c->initial_mode<MC_BASELINE || c->initial_mode>MC_ACTIVE_SCHEDULED ||
        (c->algebra_y_sign!=1 && c->algebra_y_sign!=-1) ||
        (c->center_y_sign!=1 && c->center_y_sign!=-1) || c->dt_min>c->dt_max ||
        !c->sample_max_us || !c->sample_skew_us || !c->inertial_max_us || !c->recover_samples ||
        (c->wheels.type==CK_SWERVE && c->algebra_y_sign!=1)) return 0;
    geometry_valid=ChassisKinematicsInit(&model,&c->wheels);
    return geometry_valid;
}

void ChassisMotionGetConfig(ChassisMotionConfig *c) { *c=config; }
void ChassisMotionGetWheelCommand(ChassisWheelCommand *out) { *out=wheel_command; }

void ChassisMotionSetMode(ChassisMotionMode mode)
{
    if (mode >= MC_BASELINE && mode <= MC_ACTIVE_SCHEDULED) requested_mode = (uint8_t)mode;
}

static uint8_t FilterHealthy(void)
{
    if (kf.FirstMatError != ARM_MATH_SUCCESS) return 0u;
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (!isfinite(kf.xhat_data[i]) || kf.P_data[5 * i] <= 0.0f) return 0u;
        for (uint8_t k = 0; k < 4; ++k)
            if (!isfinite(kf.P_data[4 * i + k])) return 0u;
    }
    /* 简式 P 更新后抑制单精度舍入导致的非对称；不掩盖负方差或矩阵错误。 */
    for (uint8_t i = 0; i < 4; ++i)
        for (uint8_t k = i + 1; k < 4; ++k)
        {
            float p = 0.5f * (kf.P_data[4 * i + k] + kf.P_data[4 * k + i]);
            if (p * p > kf.P_data[5 * i] * kf.P_data[5 * k] * 1.001f) return 0u;
            kf.P_data[4 * i + k] = kf.P_data[4 * k + i] = p;
        }
    return 1u;
}

static void UpdateThetaRate(const ChassisMotionInput *in)
{
    direction.rate_valid = 0u;
    if (!isfinite(in->theta_bg) || in->cmd_age_us > config.sample_max_us ||
        s.wheel_valid_mask != 15u || s.dt_s < config.dt_min || s.dt_s > config.dt_max)
    {
        theta_rate_seeded = 0u;
        direction.theta_rate_bg = 0.0f;
        return;
    }
    float delta = in->theta_bg - last_theta_bg;
    last_theta_bg = in->theta_bg;
    if (!theta_rate_seeded)
    {
        theta_rate_seeded = 1u;
        direction.theta_rate_bg = 0.0f;
        return;
    }
    /* 跨±pi时取最短角差；不使用轮速模型的omega替代编码器相对角速度。 */
    float rate = atan2f(sinf(delta), cosf(delta)) / s.dt_s;
    if (!isfinite(rate) || fabsf(rate) > config.max_theta_rate)
    {
        direction.theta_rate_bg = 0.0f;
        return;
    }
    direction.theta_rate_bg += s.dt_s / (config.theta_rate_lpf + s.dt_s) *
                               (rate - direction.theta_rate_bg);
    direction.rate_valid = 1u;
}

static void ResetConsistency(void)
{
    consistency_seeded=0; consistency_elapsed=0;
    consistency_residual_s=consistency_full_accel_error=0;
    observation.gate=0; observation.window_s=0;
    observation.reject_reason=0;
    memset(observation.delta_v_partial,0,sizeof(observation.delta_v_partial));
    memset(observation.velocity_residual,0,sizeof(observation.velocity_residual));
}

static void RejectConsistency(uint8_t reason)
{
    observation.gate=2; observation.reject_reason=reason;
    observation.rejected_windows++;
    initialized=0; have_accel=0; recovery_count=0;
    /* A fresh complete good window is required after every conflict; never
     * release the gate merely because a short prefix becomes small again. */
    consistency_seeded=0; consistency_elapsed=0;
}

/* Compare independent measurements over a finite interval. A disagreement
 * cannot identify wheel slip vs IMU bias, so it withdraws acceleration fusion
 * and active compensation. It never declares the vehicle stationary. */
static uint8_t AccelConsistent(uint32_t now, uint8_t new_velocity)
{
    float dt=(uint32_t)(now-consistency_time)*1e-6f;
    if (!consistency_seeded || dt<config.dt_min || dt>config.dt_max) {
        if (consistency_seeded) observation.gate=0;
        if (!new_velocity) { consistency_seeded=0; return 0; }
        consistency_seeded=1; consistency_elapsed=0;
        memcpy(consistency_velocity,s.wheel_n,sizeof(consistency_velocity));
        memcpy(consistency_accel,s.accel_center_n,sizeof(consistency_accel));
        memset(consistency_integral,0,sizeof(consistency_integral));
        memset(observation.delta_v_partial,0,sizeof(observation.delta_v_partial));
        consistency_residual_s=0;
    } else {
        consistency_elapsed+=dt;
        for (unsigned i=0;i<2;++i) {
            consistency_integral[i]+=.5f*(consistency_accel[i]+s.accel_center_n[i])*dt;
            consistency_accel[i]=s.accel_center_n[i];
        }
        if (new_velocity) {
            for (unsigned i=0;i<2;++i)
                observation.delta_v_partial[i]=consistency_integral[i]-(s.wheel_n[i]-consistency_velocity[i]);
            consistency_residual_s=consistency_elapsed;
            uint8_t complete=consistency_elapsed>=config.accel_consistency_window_s;
            /* Reject an already excessive prefix now, before the next KF update.
             * Small prefixes cannot grant fusion until the full window passes. */
            if (hypotf(observation.delta_v_partial[0],observation.delta_v_partial[1])>
                config.accel_consistency_max_dv) {
                memcpy(observation.delta_v_residual,observation.delta_v_partial,sizeof(observation.delta_v_residual));
                RejectConsistency(complete ? MC_REJECT_WINDOW : MC_REJECT_PREFIX);
            } else if (complete) {
                consistency_full_accel_error=hypotf(observation.delta_v_partial[0],
                    observation.delta_v_partial[1])/consistency_elapsed;
                memcpy(observation.delta_v_residual,observation.delta_v_partial,sizeof(observation.delta_v_residual));
                observation.gate=1; observation.reject_reason=0;
                consistency_elapsed=0;
                memcpy(consistency_velocity,s.wheel_n,sizeof(consistency_velocity));
                memset(consistency_integral,0,sizeof(consistency_integral));
            }
        }
    }
    consistency_time=now; observation.window_s=consistency_elapsed;
    return observation.gate==1u;
}

void ChassisMotionUpdate(const ChassisMotionInput *in)
{
    uint8_t new_wheels = 1u, new_imu, pose_ok, imu_arrived;
    float twist[3] = {0}, reconstructed[4], z[4] = {0};
    uint32_t oldest_age = 0u, newest_age = UINT32_MAX;
    s.seq++;
    s.time_us = in->time_us;
    s.cmd_age_us = in->cmd_age_us;
    s.mode = requested_mode;
    s.dt_s = clock_valid ? (uint32_t)(in->time_us - last_time) * 1e-6f : 0.0f;
    last_time = in->time_us;
    clock_valid = 1u;
    s.estimate_valid = 0u;
    control_new_pair=0;
    observation.control_ready=0;
    memset(s.innovation,0,sizeof(s.innovation));
    s.observation_mask = s.accel_used = s.wheel_valid_mask = 0u;
    s.accel_center_valid = 0u;
    s.exit_reason = MC_EXIT_NONE;
    s.imu_age_us = (uint32_t)(in->time_us - in->imu_time_us);
    s.imu_seq = in->imu_seq;
    uint8_t clock_fault=in->clock_generation!=source_clock_generation;
    source_clock_generation=in->clock_generation;
    observation.imu_time_us=in->imu_time_us;
    observation.clock_generation=in->clock_generation;
    observation.recovery_good_us=in->imu_recovery_good_us;
    observation.imu_recovering=in->imu_recovering;
    observation.attitude_flags=in->attitude_flags;
    observation.attitude_chi_ratio=in->attitude_chi_ratio;
    observation.pitch_rad=in->imu_pitch_rad; observation.roll_rad=in->imu_roll_rad;
    memcpy(observation.gravity_b,in->imu_gravity_b,sizeof(observation.gravity_b));
    memset(observation.r_diag,0,sizeof(observation.r_diag));
    memcpy(s.imu_q, in->imu_q, sizeof(s.imu_q));
    memcpy(s.imu_accel_raw, in->imu_accel_raw, sizeof(s.imu_accel_raw));
    memcpy(s.imu_motion_b, in->imu_motion_b, sizeof(s.imu_motion_b));
    memcpy(s.imu_motion_n, in->imu_motion_n, sizeof(s.imu_motion_n));
    memcpy(s.imu_gyro_rad, in->imu_gyro_rad, sizeof(s.imu_gyro_rad));
    s.current_saturated_mask = 0u;
    pose_ok = in->pose_valid && s.imu_age_us <= config.sample_max_us &&
              in->cmd_age_us <= config.sample_max_us && isfinite(in->theta_bg) && isfinite(in->psi_imu_n);
    s.theta_bg = isfinite(in->theta_bg) ? in->theta_bg : 0.0f;
    s.psi_gn = config.imu_on_gimbal ? in->psi_imu_n : in->psi_imu_n + s.theta_bg;
    s.psi_bn = s.psi_gn - s.theta_bg;
    s.omega_imu = in->omega_imu;
    if (!geometry_valid) { s.exit_reason=MC_EXIT_GEOMETRY; return; }
    steering_valid=1;
    if (model.config.type==CK_SWERVE) {
        steering_valid=in->steer_online_mask==15u;
        for (unsigned i=0;i<4;++i) {
            measured_steer[i]=in->steer_rad[i];
            uint32_t age=(uint32_t)(in->time_us-in->steer_time_us[i]);
            uint32_t wheel_age=(uint32_t)(in->time_us-in->wheel_time_us[i]);
            uint32_t skew=age>wheel_age ? age-wheel_age : wheel_age-age;
            if (!isfinite(measured_steer[i]) || age>config.sample_max_us ||
                skew>config.sample_skew_us) steering_valid=0;
        }
    }
    for (uint8_t i = 0; i < 4; ++i)
    {
        uint32_t age = (uint32_t)(in->time_us - in->wheel_time_us[i]);
        s.wheel_age_us[i] = age;
        s.wheel_seq[i] = in->wheel_seq[i];
        s.motor_dps[i] = in->motor_dps[i];
        s.raw_motor_dps[i] = in->raw_motor_dps[i];
        s.current_fb[i] = in->current_fb[i];
        s.current_send[i] = in->current_send[i];
        s.current_request[i] = in->current_request[i];
        if (fabsf(in->current_request[i]) >= config.current_limit * 0.999f) s.current_saturated_mask |= 1u << i;
        s.wheel_mps[i] = model.config.motor_sign[i] * in->motor_dps[i] * MpsPerDps(i);
        if ((in->wheel_online_mask & (1u << i)) && age <= config.sample_max_us &&
            isfinite(in->motor_dps[i]) && fabsf(in->motor_dps[i]) <= 2.0f * model.config.max_motor_dps[i])
            s.wheel_valid_mask |= 1u << i;
        if (in->wheel_seq[i] == last_wheel_seq[i]) new_wheels = 0u;
        if (age > oldest_age) oldest_age = age;
        if (age < newest_age) newest_age = age;
    }
    uint8_t wheels_ok = geometry_valid && steering_valid && s.wheel_valid_mask == 15u &&
                        oldest_age - newest_age <= config.sample_skew_us &&
                        ChassisKinematicsObserve(&model,in->motor_dps,measured_steer,twist);
    if (wheels_ok)
    {
        /* J 沿用旧电机/遥控代数，其平移 Y 与姿态所用右手 B 系相反。
         * 第十四次落地侧移确认此符号；先在 B 系反射，再按航向转到 N。
         * 不能在 N 系直接翻 Y，否则底盘有航向角时会同时污染 X。 */
        s.wheel_b[0] = twist[0]; s.wheel_b[1] = config.algebra_y_sign*twist[1]; s.omega_b = twist[2];
        if (model.config.type==CK_SWERVE) {
            for (unsigned i=0;i<4;++i) {
                float vx=twist[0]-twist[2]*model.config.y_m[i];
                float vy=twist[1]+twist[2]*model.config.x_m[i];
                reconstructed[i]=vx*cosf(measured_steer[i])+vy*sinf(measured_steer[i]);
            }
        } else ChassisMotionForward(twist, reconstructed);
        for (uint8_t i = 0; i < 4; ++i) s.wheel_residual[i] = s.wheel_mps[i] - reconstructed[i];
    }
    else s.omega_b = 0.0f; // 调度还会检查有效位，不以旧命令代替实测转速。
    UpdateThetaRate(in);
    uint32_t imu_wheel_gap = oldest_age > s.imu_age_us ? oldest_age - s.imu_age_us : s.imu_age_us - oldest_age;
    uint32_t imu_newest_gap = newest_age > s.imu_age_us ? newest_age - s.imu_age_us : s.imu_age_us - newest_age;
    new_wheels &= wheels_ok && pose_ok && oldest_age <= config.sample_skew_us &&
                  imu_wheel_gap <= config.sample_skew_us && imu_newest_gap <= config.sample_skew_us;
    imu_arrived = in->imu_seq != last_imu_seq;
    /* 中心加速度计算与是否参与 KF 分开检查，保留关闭融合的对照能力。 */
    new_imu = pose_ok && in->accel_valid &&
              imu_arrived && isfinite(in->accel_n[0]) && isfinite(in->accel_n[1]);

    /* 三维杆臂校正在 INS 同一采样帧、低通之前完成，此处只消费候选。
     * 平面安装兼容分支仍在本层求导；融合放行与候选有效性始终独立。 */
    s.accel_center_n[0] = in->accel_n[0] - config.accel_bias_n[0];
    s.accel_center_n[1] = in->accel_n[1] - config.accel_bias_n[1];
    if (new_imu)
    {
        float dtd = (uint32_t)(in->imu_time_us - derivative_time) * 1e-6f;
        float alpha_b = 0.0f, alpha_i = 0.0f;
        uint8_t rate_valid = wheels_ok;
        uint8_t offset = (config.imu_on_gimbal &&
                         (config.yaw_center_m[0] != 0 || config.yaw_center_m[1] != 0)) ||
                        (!config.use_center_accel &&
                         (config.imu_offset_m[0] != 0 || config.imu_offset_m[1] != 0));
        if (derivative_valid && dtd >= config.dt_min && dtd <= config.dt_max && wheels_ok)
        {
            alpha_b = (s.omega_b - last_omega_b) / dtd;
            alpha_i = (in->omega_imu - last_omega_imu) / dtd;
        }
        else if (offset) new_imu = 0u;
        if (!isfinite(in->omega_imu) || (offset && (!isfinite(alpha_b) || !isfinite(alpha_i) ||
            fabsf(alpha_b) > config.max_alpha || fabsf(alpha_i) > config.max_alpha)))
            new_imu = 0u;
        if (config.use_center_accel) {
          if (in->imu_center_valid) {
            s.accel_center_n[0] = in->imu_center_n[0] - config.accel_bias_n[0];
            s.accel_center_n[1] = config.center_y_sign*in->imu_center_n[1] - config.accel_bias_n[1];
          } else new_imu = 0u;
        } else {
        float ri_local[2] = {config.imu_offset_m[0], config.imu_offset_m[1]}, ri[2];
        ChassisMotionRotate(config.imu_on_gimbal ? s.psi_gn : s.psi_bn, ri_local, ri);
        float wi = config.imu_on_gimbal ? in->omega_imu : s.omega_b;
        float ai = config.imu_on_gimbal ? alpha_i : alpha_b;
        s.accel_center_n[0] += ai * ri[1] + wi * wi * ri[0];
        s.accel_center_n[1] += -ai * ri[0] + wi * wi * ri[1];
        }
        if (config.imu_on_gimbal)
        {
            float rc_local[2] = {config.yaw_center_m[0], config.yaw_center_m[1]}, rc[2];
            ChassisMotionRotate(s.psi_bn, rc_local, rc);
            s.accel_center_n[0] += alpha_b * rc[1] + s.omega_b * s.omega_b * rc[0];
            s.accel_center_n[1] += -alpha_b * rc[0] + s.omega_b * s.omega_b * rc[1];
        }
        derivative_valid = rate_valid;
        derivative_time = in->imu_time_us;
        last_omega_b = s.omega_b; last_omega_imu = in->omega_imu;
        last_imu_seq = in->imu_seq;
        if (!isfinite(s.accel_center_n[0]) || !isfinite(s.accel_center_n[1]) ||
            hypotf(s.accel_center_n[0], s.accel_center_n[1]) > config.max_accel) new_imu = 0u;
    }
    float rotation_r=0.0f;
    /* 完整三轴角速度用于观测方差；PITCH 转动也会产生杆臂残差。
     * 非法输入先拒绝，不能用于初始化滤波器或刷新加速度有效时间。 */
    if (config.imu_follows_pitch) for(uint8_t i=0;i<3;++i) {
        if(!isfinite(in->imu_gyro_rad[i])) { new_imu=0u; break; }
        float w=Clamp(in->imu_gyro_rad[i],40.0f);
        rotation_r+=config.rotation_r_gain*w*w;
    }
    /* IMU 随云台安装时，IMU 陀螺看不到底盘自身的小陀螺角速度。
     * 杆臂校正虽使用了 s.omega_b，但若只按 IMU 角速度设 R，
     * 高速自旋残差仍会被加速度观测错误地带入速度状态。 */
    if (config.imu_on_gimbal && wheels_ok)
    {
        float w_chassis = Clamp(s.omega_b, 40.0f);
        rotation_r += config.rotation_r_gain * w_chassis * w_chassis;
    }
    s.accel_center_valid = new_imu;
    if (!pose_ok) { s.exit_reason = MC_EXIT_POSE; derivative_valid = 0u; }
    if (clock_fault || s.dt_s < config.dt_min || s.dt_s > config.dt_max)
    {
        s.accel_center_valid = 0u;
        initialized = 0u; recovery_count = 0u; derivative_valid = 0u;
        have_accel=have_velocity=0; ResetConsistency();
        s.exit_reason = clock_fault ? MC_EXIT_CLOCK : MC_EXIT_DT;
        return; // 不把长停顿伪造成 5 ms 继续预测。
    }
    if (!geometry_valid) { s.exit_reason=MC_EXIT_GEOMETRY; return; }
    if (!steering_valid) {
        initialized=0; have_velocity=0; recovery_count=0;
        ResetConsistency();
        s.exit_reason=MC_EXIT_STEERING; return;
    }
    if (s.mode == MC_BASELINE)
    {
        initialized = 0u; recovery_count = 0u;
        ResetConsistency();
        s.exit_reason = MC_EXIT_DISABLED;
        return;
    }
    if (!pose_ok)
    {
        /* 没有姿态就没有 N 系观测；保留 POSE 原因，避免初始化分支
         * 用 WHEEL 覆盖，误把四轮在线的姿态故障诊断成轮速掉线。 */
        initialized = 0u; recovery_count = 0u;
        have_velocity = have_accel = 0u;
        ResetConsistency();
        return;
    }
    if (new_wheels)
    {
        ChassisMotionRotate(s.psi_bn, s.wheel_b, s.wheel_n);
        for (uint8_t i = 0; i < 4; ++i) last_wheel_seq[i] = in->wheel_seq[i];
        last_velocity_time = in->time_us;
        have_velocity = 1u;
        s.observation_mask |= 5u;
    }
    if ((imu_arrived && !new_imu) || !wheels_ok || in->imu_recovering) ResetConsistency();
    if (new_imu && config.accel_enabled) {
        if (!wheels_ok || in->imu_recovering || !AccelConsistent(in->time_us,new_wheels)) new_imu=0;
    }
    if (new_imu && config.accel_enabled)
    {
        s.observation_mask |= 10u; s.accel_used = 1u;
        have_accel = 1u; last_accel_time = in->time_us;
    }
    else new_imu = 0u; /* 禁止用未放行的候选加速度初始化或更新滤波器。 */
    if (!initialized)
    {
        if (!new_wheels) { s.exit_reason = MC_EXIT_WHEEL; return; }
        const float zero[2] = {0};
        ResetFilter(s.wheel_n, new_imu ? s.accel_center_n : zero);
    }
    z[0] = s.wheel_n[0]; z[1] = s.accel_center_n[0];
    z[2] = s.wheel_n[1]; z[3] = s.accel_center_n[1];
    const float rdiag[4] = {config.r_velocity[0], config.r_accel[0]+rotation_r, config.r_velocity[1], config.r_accel[1]+rotation_r};
    memcpy(observation.r_diag,rdiag,sizeof(rdiag));
    memset(kf.F_data, 0, 16 * sizeof(float));
    memset(kf.Q_data, 0, 16 * sizeof(float));
    memset(kf.H_data, 0, 16 * sizeof(float));
    memset(kf.R_data, 0, 16 * sizeof(float));
    for (uint8_t i = 0; i < 4; ++i)
    {
        uint8_t valid = (s.observation_mask >> i) & 1u;
        kf.F_data[5 * i] = 1.0f;
        kf.H_data[5 * i] = valid ? 1.0f : 0.0f;
        kf.R_data[5 * i] = rdiag[i]; // 屏蔽槽位仍保留正 R，保证 S 可逆。
        kf.MeasuredVector[i] = valid ? z[i] : 0.0f;
        float predicted = kf.xhat_data[i] + ((i & 1u) ? 0.0f : s.dt_s * kf.xhat_data[i + 1]);
        s.innovation[i] = ((i&1u) ? s.accel_center_valid : new_wheels) ? z[i]-predicted : 0.0f;
    }
    kf.F_data[1] = kf.F_data[11] = s.dt_s;
    for (uint8_t a = 0; a < 2; ++a)
    {
        uint8_t base = a * 10;
        float q = a ? config.q_jerk[1] : config.q_jerk[0], dt = s.dt_s;
        kf.Q_data[base] = q * dt * dt * dt / 3.0f;
        kf.Q_data[base + 1] = kf.Q_data[base + 4] = q * dt * dt / 2.0f;
        kf.Q_data[base + 5] = q * dt;
    }
    Kalman_Filter_Update(&kf);
    if (!FilterHealthy())
    {
        initialized = 0u; recovery_count = 0u;
        s.exit_reason = MC_EXIT_NUMERIC;
        return;
    }
    if (new_wheels) {
        observation.velocity_residual[0]=kf.xhat_data[0]-s.wheel_n[0];
        observation.velocity_residual[1]=kf.xhat_data[2]-s.wheel_n[1];
        if (have_accel && hypotf(observation.velocity_residual[0],observation.velocity_residual[1])>
            config.velocity_innovation_max) {
            /* Check the posterior too: an acceleration observation can move
             * velocity through cross-covariance before integral(a) is excessive.
             * Withdraw permission and publish wheel-only diagnostics this frame.
             * This is not a zero-speed/slip verdict; moving wheels stay moving. */
            const float zero[2]={0};
            RejectConsistency(MC_REJECT_VELOCITY);
            ResetFilter(s.wheel_n,zero);
            s.accel_used=0; s.observation_mask&=5u; new_imu=0;
        }
    }
    memcpy(s.x, kf.xhat_data, sizeof(s.x));
    for (uint8_t i = 0; i < 4; ++i) s.p_diag[i] = kf.P_data[5 * i];
    float velocity_n[2] = {s.x[0], s.x[2]};
    if (pose_ok)
    {
        /* 估计 N → 右手 B → 旧代数 B → 旧命令 G。
         * theta_BG=psi_GN-psi_BN 属右手姿态角，旧代数 Y 已反射，
         * 故返回命令用 R(+theta_BG)，与输出 R(-theta_BG) 互逆。
         * 等价于 S*R(-psi_GN)*v_N；反射前后不能沿用相同旋转符号。 */
        float velocity_b[2];
        ChassisMotionRotate(-s.psi_bn, velocity_n, velocity_b);
        velocity_b[1] *= config.algebra_y_sign;
        ChassisMotionRotate(-config.algebra_y_sign*s.theta_bg, velocity_b, s.hat_g);
    }
    if (!pose_ok) { recovery_count = 0u; return; }
    if (!have_velocity || (uint32_t)(in->time_us - last_velocity_time) > config.inertial_max_us)
    {
        initialized = 0u; recovery_count = 0u;
        s.exit_reason = MC_EXIT_WHEEL; return;
    }
    if (s.p_diag[0] > config.max_p_velocity || s.p_diag[2] > config.max_p_velocity)
    { recovery_count = 0u; s.exit_reason = MC_EXIT_COVARIANCE; return; }
    /* 轮速诊断可以运行，但完整融合未经安装验证/IMU 超时时不准主动补偿。 */
    if (in->imu_recovering || (config.accel_enabled && s.accel_center_valid && observation.gate!=1u)) {
        recovery_count=0;
        s.exit_reason=observation.gate==2u ? MC_EXIT_OBSERVATION : MC_EXIT_RECOVERING;
        return;
    }
    if (!config.accel_enabled || !in->accel_valid || !have_accel || (imu_arrived && !new_imu) ||
        (uint32_t)(in->time_us - last_accel_time) > config.sample_max_us)
    { recovery_count = 0u; s.exit_reason = MC_EXIT_IMU; return; }
    if (new_wheels && new_imu && recovery_count < config.recover_samples) recovery_count++;
    if (recovery_count < config.recover_samples)
    { s.exit_reason = MC_EXIT_RECOVERING; return; }
    s.estimate_valid = 1u;
    control_new_pair=new_wheels && new_imu;
}

/* No command-based stationary inference. Matching real acceleration/coasting is
 * eligible; sensor disagreement below the estimator's 0.04 m/s rejection limit
 * can still withhold control. Call once after Update, before constructing cmd. */
static void UpdateControlPermission(uint8_t enabled, uint8_t transition)
{
    uint16_t reason=0;
    uint8_t new_tick=s.seq!=control_last_seq;
    control_last_seq=s.seq;
    float pair_dt=s.dt_s;
    if (new_tick && control_new_pair) {
        uint32_t gap=(uint32_t)(s.time_us-control_pair_time);
        if (control_have_pair && gap<=config.sample_max_us) pair_dt=gap*1e-6f;
        else observation.control_good_s=observation.control_blend=0;
        control_pair_time=s.time_us; control_have_pair=1;
    }
    observation.control_velocity_error=hypotf(s.x[0]-s.wheel_n[0],s.x[2]-s.wheel_n[1]);
    float partial=consistency_residual_s>0 ?
        hypotf(observation.delta_v_partial[0],observation.delta_v_partial[1])/consistency_residual_s : 0;
    observation.control_accel_residual=fmaxf(partial,consistency_full_accel_error);
    if (!s.estimate_valid || observation.gate!=1u) reason|=MC_CONTROL_ESTIMATE;
    if (!control_have_pair || (uint32_t)(s.time_us-control_pair_time)>config.sample_max_us ||
        s.wheel_valid_mask!=15u || s.imu_age_us>config.sample_max_us || s.cmd_age_us>config.sample_max_us)
        reason|=MC_CONTROL_SOURCE;
    if (!isfinite(observation.control_velocity_error) || observation.control_velocity_error>config.control_velocity_max)
        reason|=MC_CONTROL_VELOCITY;
    if (!isfinite(partial) || !isfinite(consistency_full_accel_error) ||
        observation.control_accel_residual>config.control_accel_residual_max) reason|=MC_CONTROL_ACCEL;
    if (s.current_saturated_mask || s.wheel_scale<.999f) reason|=MC_CONTROL_ACTUATOR;
    for (unsigned i=0;i<4;++i)
        if (!isfinite(s.current_request[i]) || !isfinite(s.current_send[i]) || !isfinite(s.current_fb[i]) ||
            fabsf(s.current_request[i]-s.current_send[i])>=1.1f) reason|=MC_CONTROL_ACTUATOR;
    if (!enabled) reason|=MC_CONTROL_DISABLED;
    if (transition) reason|=MC_CONTROL_TRANSITION;
    if (s.mode<MC_SHADOW) reason|=MC_CONTROL_MODE;
    observation.control_ready=0;
    if (reason) {
        observation.control_good_s=observation.control_blend=0;
    } else {
        if (new_tick && control_new_pair) observation.control_good_s=fminf(config.control_stable_s,
            observation.control_good_s+pair_dt);
        if (observation.control_good_s+1e-6f>=config.control_stable_s) {
            observation.control_ready=1;
            if (new_tick && control_new_pair) observation.control_blend=fminf(1,
                observation.control_blend+pair_dt/config.control_ramp_s);
        } else reason=MC_CONTROL_SETTLING;
    }
    observation.control_inhibit=reason;
}

static float WheelRequests(const float g[2], float omega, float motor[4])
{
    float b[2], twist[3];
    /* 旧命令 G/B 的 Y 与姿态右手系反向，S*R(theta)*S=R(-theta)。
     * 入口 theta 已按编码器符号转为右手角，不能直接用于旧代数旋转。
     * 使用 -theta 恢复原 offset_angle 的输出方向：底盘自旋时平移仍跟随云台。 */
    ChassisMotionRotate(config.algebra_y_sign*direction.theta_output_bg, g, b);
    twist[0] = b[0]; twist[1] = b[1]; twist[2] = omega;
    if (!ChassisKinematicsCommand(&model,twist,measured_steer,&wheel_command)) {
        memset(motor,0,4*sizeof(float)); return 0;
    }
    memcpy(motor,wheel_command.motor_dps,4*sizeof(float));
    return ChassisKinematicsScale(&model,twist);
}

/* 在固定 omega 下求最大的可行平移比例，避免小陀螺模式把转速一起缩掉。 */
static float TranslationScaleForOmega(const float g[2], float omega)
{
    float b[2];
    ChassisMotionRotate(config.algebra_y_sign*direction.theta_output_bg, g, b);
    return ChassisKinematicsTranslationScale(&model,b,omega);
}

static void LimitTwistToWheelSpeed(float translation[2], float *omega,
                                   uint8_t control_key)
{
    float wheel[4];
    if (config.rotation_priority && control_key == MC_ROTATE_CONTROL_KEY)
    {
        const float zero[2] = {0.0f, 0.0f};
        float spin_scale = WheelRequests(zero, *omega, wheel);
        if (spin_scale < 1.0f)
        {
            *omega *= spin_scale * 0.999f; // 给浮点/电机参考量化留少量余量。
            s.limit_flags |= MC_LIMIT_REF;
        }
        float translation_scale = TranslationScaleForOmega(translation, *omega);
        if (translation_scale < 1.0f)
        {
            translation_scale *= 0.999f;
            translation[0] *= translation_scale;
            translation[1] *= translation_scale;
            s.limit_flags |= MC_LIMIT_REF;
        }
        return;
    }
    float scale = WheelRequests(translation, *omega, wheel);
    if (scale < 1.0f)
    {
        translation[0] *= scale;
        translation[1] *= scale;
        *omega *= scale;
        s.limit_flags |= MC_LIMIT_REF;
    }
}

void ChassisMotionControl(const float user[2], float omega, uint8_t enabled,
                          uint8_t control_key, float motor[4])
{
    float target[2] = {user[0], user[1]};
    uint8_t transition = !previous_enabled || control_key != control_key_last || mode_last != s.mode;
    float previous_scale = s.wheel_scale;
    s.limit_flags = 0u; s.kp = 0.0f;
    direction.lead_rad = 0.0f;
    direction.theta_output_bg = s.theta_bg;
    memcpy(s.user_g, user, sizeof(s.user_g));
    memset(s.dv_raw_g, 0, sizeof(s.dv_raw_g));
    if (transition || !s.estimate_valid) memset(s.dv_g, 0, sizeof(s.dv_g));
    previous_enabled = enabled;
    control_key_last = control_key; mode_last = s.mode;
    if (!enabled || !geometry_valid || !steering_valid || !isfinite(user[0]) || !isfinite(user[1]) || !isfinite(omega) ||
        !isfinite(s.theta_bg) || s.dt_s < config.dt_min || s.dt_s > config.dt_max)
    {
        UpdateControlPermission(0,transition);
        memset(s.ref_g, 0, sizeof(s.ref_g)); memset(s.cmd_g, 0, sizeof(s.cmd_g));
        memset(s.dv_g, 0, sizeof(s.dv_g)); memset(s.wheel_req_dps, 0, sizeof(s.wheel_req_dps));
        memset(s.wheel_eff_dps, 0, sizeof(s.wheel_eff_dps)); memset(motor, 0, 4 * sizeof(float));
        memset(wheel_command.motor_dps,0,sizeof(wheel_command.motor_dps));
        for (unsigned i=0;i<4;++i) wheel_command.steer_rad[i]=isfinite(measured_steer[i]) ? measured_steer[i] : 0;
        s.omega_ref = 0.0f; s.wheel_scale = 1.0f;
        if (!enabled) s.exit_reason = MC_EXIT_DISABLED;
        return; // 急停不经过斜坡；估计本身继续观察真实滑行。
    }
    if (control_key == MC_ROTATE_CONTROL_KEY && direction.rate_valid && fabsf(omega) > 0.5f)
    {
        direction.lead_rad = Clamp(direction.theta_rate_bg * config.phase_lead_s,
                                   config.max_phase_lead);
        direction.theta_output_bg += direction.lead_rad;
    }
    if (LimitVector(target, config.max_velocity)) s.limit_flags |= MC_LIMIT_REF;
    omega = Clamp(omega, config.max_omega);
    LimitTwistToWheelSpeed(target, &omega, control_key);
    SlewVector(target, s.ref_g, config.velocity_slew * s.dt_s);
    s.omega_ref += Clamp(omega - s.omega_ref, config.omega_slew * s.dt_s);
    /* 平移与旋转独立斜坡的中间组合也需可实现，再把它作为误差目标。 */
    LimitTwistToWheelSpeed(s.ref_g, &s.omega_ref, control_key);
    /* 以经过上游可行性处理的 ref 比较，不把上次 cmd 再当本次目标。 */
    for (uint8_t i = 0; i < 2; ++i) s.error_g[i] = s.ref_g[i] - s.hat_g[i];
    UpdateControlPermission(enabled,transition);
    if (s.estimate_valid && s.mode >= MC_SHADOW)
    {
        s.kp = s.mode == MC_ACTIVE_SCHEDULED ?
            fminf(config.max_kp, config.schedule_kp * fabsf(s.omega_b) / config.schedule_omega) : config.fixed_kp;
        if (s.mode == MC_ACTIVE_SCHEDULED && s.wheel_valid_mask != 15u) s.kp = 0.0f;
        s.kp = fminf(s.kp, config.max_kp) * previous_scale;
        if (s.current_saturated_mask) s.kp = 0.0f; // 上次执行已限流，撤去外层追加权限。
        float dv[2];
        for (uint8_t i = 0; i < 2; ++i) {
            s.dv_raw_g[i] = s.kp * s.error_g[i];
            dv[i] = s.dv_raw_g[i] * observation.control_blend;
        }
        if (LimitVector(dv, config.max_dv)) s.limit_flags |= MC_LIMIT_DV;
        if (observation.control_ready) SlewVector(dv, s.dv_g, config.dv_slew * s.dt_s);
        else memset(s.dv_g,0,sizeof(s.dv_g));
    }
    else memset(s.dv_g, 0, sizeof(s.dv_g));
    for (uint8_t i = 0; i < 2; ++i)
        s.cmd_g[i] = s.ref_g[i] + (s.mode >= MC_ACTIVE_FIXED && observation.control_ready ? s.dv_g[i] : 0.0f);
    s.wheel_scale = WheelRequests(s.cmd_g, s.omega_ref, s.wheel_req_dps);
    if (config.rotation_priority && control_key == MC_ROTATE_CONTROL_KEY)
    {
        if (s.wheel_scale < 1.0f)
        {
            s.limit_flags |= MC_LIMIT_WHEEL;
            /* 基础 ref 已在前面限为可行；只撤掉会再次挤占旋转余量的外层补偿。 */
            s.kp = 0.0f;
            memset(s.dv_raw_g, 0, sizeof(s.dv_raw_g));
            memset(s.dv_g, 0, sizeof(s.dv_g));
            memcpy(s.cmd_g, s.ref_g, sizeof(s.cmd_g));
            s.wheel_scale = WheelRequests(s.cmd_g, s.omega_ref, s.wheel_req_dps);
            if (s.wheel_scale < 1.0f)
            {
                LimitTwistToWheelSpeed(s.ref_g, &s.omega_ref, control_key);
                memcpy(s.cmd_g, s.ref_g, sizeof(s.cmd_g));
                s.wheel_scale = WheelRequests(s.cmd_g, s.omega_ref, s.wheel_req_dps);
            }
        }
        for (uint8_t i = 0; i < 4; ++i)
            motor[i] = s.wheel_eff_dps[i] = s.wheel_req_dps[i];
    }
    else
    {
        if (s.wheel_scale < 1.0f) s.limit_flags |= MC_LIMIT_WHEEL;
        for (uint8_t i = 0; i < 4; ++i)
            motor[i] = s.wheel_eff_dps[i] = s.wheel_req_dps[i] * s.wheel_scale;
    }
    /* Empirical balance is limited to centered pure rotation. Keep the trim out
     * of translations, active translational correction, and wheel-saturated motion. */
    if (control_key==MC_ROTATE_CONTROL_KEY && hypotf(s.cmd_g[0],s.cmd_g[1])<0.005f &&
        fabsf(s.omega_ref)>0.5f && s.wheel_scale>=0.9999f)
    {
        const float spin_sign=s.omega_ref>0.0f ? 1.0f : -1.0f;
        uint8_t within_limit=1u;
        for (uint8_t i=0;i<4;++i)
            if (fabsf(motor[i]+spin_sign*config.spin_wheel_trim_dps[i])>
                config.wheels.max_motor_dps[i]) within_limit=0u;
        if (within_limit)
            for (uint8_t i=0;i<4;++i)
                motor[i]=s.wheel_eff_dps[i]=motor[i]+spin_sign*config.spin_wheel_trim_dps[i];
    }
    memcpy(wheel_command.motor_dps,motor,sizeof(wheel_command.motor_dps));
}

void ChassisMotionVisitRecord(void (*visit)(const ChassisMotionState *,
                                            const ChassisMotionObservationState *))
{
    visit(&s, &observation);
}

void ChassisMotionRecord(void)
{
    if (!config.log_decimation || s.seq % config.log_decimation) return;
    uint32_t key = Lock();
    if ((uint32_t)(log_write - log_read) >= MC_LOG_DEPTH) s.log_drop++;
    else {
        logs[log_write % MC_LOG_DEPTH] = s;
        direction_logs[log_write % MC_LOG_DEPTH] = direction;
        observation_logs[log_write % MC_LOG_DEPTH] = observation;
        log_write++;
    }
    Unlock(key);
}
uint8_t ChassisMotionPopLog(ChassisMotionState *out, ChassisMotionDirectionState *out_direction)
{
    return ChassisMotionPopDiagnosticLog(out,out_direction,NULL);
}
uint8_t ChassisMotionPopDiagnosticLog(ChassisMotionState *out,
    ChassisMotionDirectionState *out_direction, ChassisMotionObservationState *out_observation)
{
    uint32_t key = Lock();
    uint8_t present = log_write != log_read;
    if (present) {
        *out = logs[log_read % MC_LOG_DEPTH];
        if (out_direction) *out_direction = direction_logs[log_read % MC_LOG_DEPTH];
        if (out_observation) *out_observation = observation_logs[log_read % MC_LOG_DEPTH];
        log_read++;
    }
    Unlock(key);
    return present;
}
void ChassisMotionGetState(ChassisMotionState *out)
{
    uint32_t key = Lock();
    *out = s;
    Unlock(key);
}
void ChassisMotionGetObservationState(ChassisMotionObservationState *out)
{
    uint32_t key=Lock(); *out=observation; Unlock(key);
}
void ChassisMotionGetDirectionState(ChassisMotionDirectionState *out)
{
    uint32_t key = Lock();
    *out = direction;
    Unlock(key);
}
