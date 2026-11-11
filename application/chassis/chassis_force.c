#include "chassis_force.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "config.h"
#include "math_utils.h"
#include <math.h>
#include <string.h>

static float force_current_ref[4];                         // LF, RF, LB, RB，DJI 原始电流量
static float force_velocity_damping[4];                    // 根据轮速生成的反向阻尼电流
static uint16_t force_start_hold_cycles[4];                // 每轮启速补偿剩余保持周期
static uint16_t force_start_cooldown_cycles[4];            // 堵转后的重试冷却周期
static uint8_t force_start_exit_cycles[4];                 // 连续达到退出速度的周期数
static uint8_t force_start_active[4];                      // 每轮启速补偿锁存状态
static uint8_t force_start_armed[4] = {1u, 1u, 1u, 1u};   // 非零命令沿触发一次补偿
static int8_t force_start_sign[4];                         // 上一次非零轮端电流方向
static uint16_t force_current_log_count;
static float force_current_filter_history[4][2];
static float force_current_filtered[4];
static uint8_t force_current_filter_initialized[4];
static Chassis_Force_Feed_s chassis_force_feedback;
static Chassis_Force_Sensor_s chassis_force_sensor;
static uint32_t force_sensor_last_seq;
static uint8_t force_sensor_stale_cycles;
static uint8_t force_sensor_have_sample;
static PIDInstance force_pid_x, force_pid_y, force_pid_z;
static float admittance_vx, admittance_vy, admittance_wz;
static uint32_t admittance_dwt_cnt;

static uint8_t ForceValueValid(float value, float limit)
{
    return isfinite(value) && fabsf(value) <= limit;
}

uint8_t ChassisForceCommandValid(const Chassis_Ctrl_Cmd_s *command)
{
    return ForceValueValid(command->fx_ref, g_chassis_force_config.input_max_n) &&
           ForceValueValid(command->fy_ref, g_chassis_force_config.input_max_n) &&
           ForceValueValid(command->mz_ref, g_chassis_force_config.input_max_nm);
}

static uint8_t ForceSequenceNewer(uint32_t sequence)
{
    uint32_t delta;
    if (!force_sensor_have_sample)
        return 1u;
    delta = sequence - force_sensor_last_seq;
    return delta != 0u && delta < 0x80000000u;
}

static void ResetPIDState(PIDInstance *pid)
{
    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
}

static float ConstrainSlew(float target, float previous)
{
    float delta = target - previous;
    if (delta > g_chassis_force_config.current_slew_raw)
        delta = g_chassis_force_config.current_slew_raw;
    else if (delta < -g_chassis_force_config.current_slew_raw)
        delta = -g_chassis_force_config.current_slew_raw;
    return previous + delta;
}

static float VelocityDampingCurrent(float speed_aps)
{
    return float_constrain(g_chassis_force_config.speed_damping_raw_per_dps * speed_aps,
                           -g_chassis_force_config.speed_damping_limit_raw,
                           g_chassis_force_config.speed_damping_limit_raw);
}

/*
 * 理想模型的力矩前馈不包含轮胎/轴承静摩擦。起步时在任务空间增加一小段
 * 同方向的补偿，再由同一个雅可比矩阵分配到各轮，避免直接给四个轮子加
 * 相同电流而放大旋转力矩。只在轮速很低时生效，运动起来后交回理想力矩
 * 前馈和速度阻尼控制。
 */
static void ApplyForceStartCompensation(float *fx, float *fy, float *mz,
                                        DJIMotorInstance *const motors[4])
{
    float wheel_speed_max = 0.0f;
    float translation_norm;
    float speed;
    const float wheel_speed[4] = {
        fabsf(motors[0]->measure.speed_aps),
        fabsf(motors[1]->measure.speed_aps),
        fabsf(motors[2]->measure.speed_aps),
        fabsf(motors[3]->measure.speed_aps),
    };

    for (uint8_t i = 0; i < 4; ++i)
    {
        if (wheel_speed[i] > wheel_speed_max)
            wheel_speed_max = wheel_speed[i];
    }
    if (wheel_speed_max > g_chassis_force_config.start_speed_dps)
        return;

    translation_norm = sqrtf((*fx) * (*fx) + (*fy) * (*fy));
    if (translation_norm > g_chassis_force_config.start_input_deadband_n)
    {
        *fx += *fx / translation_norm * g_chassis_force_config.start_translation_n;
        *fy += *fy / translation_norm * g_chassis_force_config.start_translation_n;
    }

    speed = fabsf(*mz);
    if (speed > g_chassis_force_config.start_input_deadband_nm)
        *mz += (*mz > 0.0f ? 1.0f : -1.0f) * g_chassis_force_config.start_moment_nm;
}

/* 4x3 轮子雅可比的阻尼最小二乘分配：f = J (J^T J + λI)^-1 W。 */
static uint8_t AllocateChassisForce(float fx, float fy, float mz, float wheel_force[4])
{
    /* 与 OmniCalculate 的电机顺序和正负号一致，旋转列使用真实米制力臂。 */
    const float (*j)[3] = g_chassis_force_config.rows;
    float a[3][3] = {{0.0f}};
    float inv[3][3];
    float w[3] = {fx, fy, mz};
    float det;

    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t r = 0; r < 3; ++r)
            for (uint8_t c = 0; c < 3; ++c)
                a[r][c] += j[i][r] * j[i][c];
    }
    a[0][0] += g_chassis_force_config.pseudoinverse_damping;
    a[1][1] += g_chassis_force_config.pseudoinverse_damping;
    a[2][2] += g_chassis_force_config.pseudoinverse_damping;

    det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
          a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
          a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (fabsf(det) < 1.0e-8f)
        return 0;

    inv[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det;
    inv[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det;
    inv[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det;
    inv[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det;
    inv[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det;
    inv[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det;
    inv[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det;
    inv[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det;
    inv[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det;

    for (uint8_t i = 0; i < 4; ++i)
    {
        float projected[3] = {0.0f};
        for (uint8_t r = 0; r < 3; ++r)
            for (uint8_t c = 0; c < 3; ++c)
                projected[r] += inv[r][c] * w[c];
        wheel_force[i] = j[i][0] * projected[0] + j[i][1] * projected[1] + j[i][2] * projected[2];
    }
    return 1;
}

static float WheelForceToCurrent(float wheel_force)
{
    float torque_motor = wheel_force * g_chassis_force_config.wheel_radius_m /
                         (g_chassis_force_config.gear_ratio * g_chassis_force_config.drive_efficiency);
    return torque_motor / g_chassis_force_config.motor_kt_nm_per_a * g_chassis_force_config.current_raw_per_a;
}

static float CurrentToWheelForce(float current_raw)
{
    return current_raw / g_chassis_force_config.current_raw_per_a * g_chassis_force_config.motor_kt_nm_per_a *
           g_chassis_force_config.gear_ratio * g_chassis_force_config.drive_efficiency / g_chassis_force_config.wheel_radius_m;
}

static float Median3(float a, float b, float c)
{
    float temp;
    if (a > b)
    {
        temp = a;
        a = b;
        b = temp;
    }
    if (b > c)
    {
        temp = b;
        b = c;
        c = temp;
    }
    if (a > b)
        b = a;
    return b;
}

static float FilterForceCurrent(uint8_t index, float sample)
{
    float median;
    if (!force_current_filter_initialized[index])
    {
        force_current_filter_history[index][0] = sample;
        force_current_filter_history[index][1] = sample;
        force_current_filtered[index] = sample;
        force_current_filter_initialized[index] = 1u;
        return force_current_filtered[index];
    }

    median = Median3(force_current_filter_history[index][0],
                     force_current_filter_history[index][1], sample);
    force_current_filter_history[index][0] = force_current_filter_history[index][1];
    force_current_filter_history[index][1] = sample;
    force_current_filtered[index] += g_chassis_force_config.current_filter_alpha *
                                     (median - force_current_filtered[index]);
    return force_current_filtered[index];
}

static void EstimateChassisForce(float wheel_force[4], float *fx, float *fy, float *mz,
                                 DJIMotorInstance *const motors[4])
{
    const float (*j)[3] = g_chassis_force_config.rows;
    const float raw_current[4] = {
        (float)motors[0]->measure.real_current - g_chassis_force_config.current_offset_raw[0],
        (float)motors[1]->measure.real_current - g_chassis_force_config.current_offset_raw[1],
        (float)motors[2]->measure.real_current - g_chassis_force_config.current_offset_raw[2],
        (float)motors[3]->measure.real_current - g_chassis_force_config.current_offset_raw[3],
    };
    *fx = *fy = *mz = 0.0f;
    for (uint8_t i = 0; i < 4; ++i)
        wheel_force[i] = CurrentToWheelForce(FilterForceCurrent(i, raw_current[i]));
    for (uint8_t i = 0; i < 4; ++i)
    {
        *fx += j[i][0] * wheel_force[i];
        *fy += j[i][1] * wheel_force[i];
        *mz += j[i][2] * wheel_force[i];
    }
}

const Chassis_Force_Feed_s *ChassisForceUpdateFeedback(uint8_t motors_online,
                                                      const Chassis_Ctrl_Cmd_s *command,
                                                      DJIMotorInstance *const motors[4])
{
    /* Generic LOGINFO may be compiled out by BSP. */
    (void)command;
    float sensor_wheel_force[4];
    float sensor_fx, sensor_fy, sensor_mz;
    if (!motors_online)
        memset(force_current_filter_initialized, 0, sizeof(force_current_filter_initialized));

    chassis_force_feedback.protocol_version = CHASSIS_FORCE_FEED_VERSION;
    chassis_force_feedback.force_source = CHASSIS_FORCE_SOURCE_ESTIMATE;
    chassis_force_feedback.force_valid = 1u;
    chassis_force_feedback.reserved = 0u;
    chassis_force_feedback.sample_seq++;
    EstimateChassisForce(chassis_force_feedback.wheel_force,
                         &chassis_force_feedback.fx,
                         &chassis_force_feedback.fy,
                         &chassis_force_feedback.mz, motors);
    if (force_sensor_have_sample && force_sensor_stale_cycles <= g_chassis_force_config.sensor_timeout_cycles)
    {
        memcpy(sensor_wheel_force, chassis_force_sensor.wheel_force, sizeof(sensor_wheel_force));
        sensor_fx = chassis_force_sensor.fx;
        sensor_fy = chassis_force_sensor.fy;
        sensor_mz = chassis_force_sensor.mz;
        memcpy(chassis_force_feedback.wheel_force, sensor_wheel_force, sizeof(sensor_wheel_force));
        chassis_force_feedback.fx = sensor_fx;
        chassis_force_feedback.fy = sensor_fy;
        chassis_force_feedback.mz = sensor_mz;
        chassis_force_feedback.force_source = CHASSIS_FORCE_SOURCE_SENSOR;
    }
    else if (force_sensor_have_sample)
    {
        chassis_force_feedback.force_source = CHASSIS_FORCE_SOURCE_FAULT;
        chassis_force_feedback.force_valid = 0u;
    }
    if (!motors_online)
    {
        chassis_force_feedback.force_source = CHASSIS_FORCE_SOURCE_FAULT;
        chassis_force_feedback.force_valid = 0u;
    }
    chassis_force_feedback.current_ref[0] = force_current_ref[0];
    chassis_force_feedback.current_ref[1] = force_current_ref[1];
    chassis_force_feedback.current_ref[2] = force_current_ref[2];
    chassis_force_feedback.current_ref[3] = force_current_ref[3];
    chassis_force_feedback.current_feedback[0] = force_current_filtered[0];
    chassis_force_feedback.current_feedback[1] = force_current_filtered[1];
    chassis_force_feedback.current_feedback[2] = force_current_filtered[2];
    chassis_force_feedback.current_feedback[3] = force_current_filtered[3];
    if (++force_current_log_count >= g_chassis_log_config.force_log_period_cycles)
    {
        force_current_log_count = 0u;
        LOGINFO("[CHASSIS_FORCE v20261008B] ref=%d,%d,%d,%d fb=%d,%d,%d,%d force=%d,%d,%d target=%d,%d,%d wheel_spd=%d,%d,%d,%d damp=%d,%d,%d,%d start=%d,%d,%d,%d",
                (int)force_current_ref[0],
                (int)force_current_ref[1],
                (int)force_current_ref[2],
                (int)force_current_ref[3],
                (int)chassis_force_feedback.current_feedback[0],
                (int)chassis_force_feedback.current_feedback[1],
                (int)chassis_force_feedback.current_feedback[2],
                (int)chassis_force_feedback.current_feedback[3],
                (int)(chassis_force_feedback.fx * 100.0f),
                (int)(chassis_force_feedback.fy * 100.0f),
                (int)(chassis_force_feedback.mz * 100.0f),
                (int)(command->fx_ref * 100.0f),
                (int)(command->fy_ref * 100.0f),
                (int)(command->mz_ref * 100.0f),
                (int)motors[0]->measure.speed_aps,
                (int)motors[1]->measure.speed_aps,
                (int)motors[2]->measure.speed_aps,
                (int)motors[3]->measure.speed_aps,
                (int)force_velocity_damping[0],
                (int)force_velocity_damping[1],
                (int)force_velocity_damping[2],
                (int)force_velocity_damping[3],
                (int)force_start_active[0],
                (int)force_start_active[1],
                (int)force_start_active[2],
                (int)force_start_active[3]);
    }
    return &chassis_force_feedback;
}

uint8_t ChassisForceReadMeasurement(const Chassis_Force_Sensor_s *sample,
                                    float *fx, float *fy, float *mz)
{
    if (sample)
    {
        chassis_force_sensor = *sample;
        if (chassis_force_sensor.protocol_version != CHASSIS_FORCE_FEED_VERSION ||
            chassis_force_sensor.force_source != CHASSIS_FORCE_SOURCE_SENSOR ||
            !chassis_force_sensor.force_valid)
        {
            force_sensor_stale_cycles = g_chassis_force_config.sensor_timeout_cycles + 1u;
        }
        else if (!ForceValueValid(chassis_force_sensor.fx, g_chassis_force_config.input_max_n) ||
                 !ForceValueValid(chassis_force_sensor.fy, g_chassis_force_config.input_max_n) ||
                 !ForceValueValid(chassis_force_sensor.mz, g_chassis_force_config.input_max_nm) ||
                 !ForceValueValid(chassis_force_sensor.wheel_force[0], g_chassis_force_config.input_max_n) ||
                 !ForceValueValid(chassis_force_sensor.wheel_force[1], g_chassis_force_config.input_max_n) ||
                 !ForceValueValid(chassis_force_sensor.wheel_force[2], g_chassis_force_config.input_max_n) ||
                 !ForceValueValid(chassis_force_sensor.wheel_force[3], g_chassis_force_config.input_max_n))
        {
            force_sensor_stale_cycles = g_chassis_force_config.sensor_timeout_cycles + 1u;
        }
        else if (!ForceSequenceNewer(chassis_force_sensor.sample_seq))
        {
            force_sensor_stale_cycles++;
        }
        else
        {
            force_sensor_last_seq = chassis_force_sensor.sample_seq;
            force_sensor_have_sample = 1u;
            force_sensor_stale_cycles = 0u;
        }
    }
    else if (force_sensor_stale_cycles < 0xffu)
    {
        force_sensor_stale_cycles++;
    }

    if (force_sensor_have_sample && force_sensor_stale_cycles <= g_chassis_force_config.sensor_timeout_cycles)
    {
        *fx = chassis_force_sensor.fx;
        *fy = chassis_force_sensor.fy;
        *mz = chassis_force_sensor.mz;
        return 1u;
    }

    /* 估算力矩模式使用 RunTorqueControl()，真实力跟踪模式不接受电流估算回退。 */
    return 0u;
}

void ChassisForceResetControl(void)
{
    memset(force_current_ref, 0, sizeof(force_current_ref));
    memset(force_velocity_damping, 0, sizeof(force_velocity_damping));
    ResetPIDState(&force_pid_x);
    ResetPIDState(&force_pid_y);
    ResetPIDState(&force_pid_z);
    admittance_vx = 0.0f;
    admittance_vy = 0.0f;
    admittance_wz = 0.0f;
    memset(force_start_hold_cycles, 0, sizeof(force_start_hold_cycles));
    memset(force_start_cooldown_cycles, 0, sizeof(force_start_cooldown_cycles));
    memset(force_start_exit_cycles, 0, sizeof(force_start_exit_cycles));
    memset(force_start_active, 0, sizeof(force_start_active));
    for (uint8_t i = 0; i < 4; ++i)
    {
        force_start_armed[i] = 1u;
        force_start_sign[i] = 0;
    }
}

static uint8_t ForceStartCompensationActive(uint8_t index,
                                            float feedforward_current,
                                            float speed_aps)
{
    const float current_abs = fabsf(feedforward_current);
    const float speed_abs = fabsf(speed_aps);
    const int8_t sign = feedforward_current > 0.0f ? 1 : -1;

    if (current_abs <= g_chassis_force_config.start_wheel_min_raw)
    {
        force_start_active[index] = 0u;
        force_start_hold_cycles[index] = 0u;
        force_start_cooldown_cycles[index] = 0u;
        force_start_exit_cycles[index] = 0u;
        force_start_armed[index] = 1u;
        force_start_sign[index] = 0;
        return 0u;
    }

    /* 力方向改变时重新允许一次起步补偿；同一方向持续按键不反复触发。 */
    if (force_start_sign[index] != 0 && force_start_sign[index] != sign)
    {
        force_start_active[index] = 0u;
        force_start_hold_cycles[index] = 0u;
        force_start_cooldown_cycles[index] = 0u;
        force_start_exit_cycles[index] = 0u;
        force_start_armed[index] = 1u;
    }
    force_start_sign[index] = sign;

    if (force_start_cooldown_cycles[index] > 0u)
    {
        force_start_cooldown_cycles[index]--;
        return 0u;
    }

    if (force_start_armed[index] && speed_abs <= g_chassis_force_config.start_speed_dps)
    {
        force_start_armed[index] = 0u;
        force_start_active[index] = 1u;
        force_start_hold_cycles[index] = g_chassis_force_config.start_hold_cycles;
        force_start_exit_cycles[index] = 0u;
    }

    if (!force_start_active[index])
        return 0u;

    /* 轮子已经明确转起来，提前结束补偿；否则至少保持固定周期。 */
    if (speed_abs >= g_chassis_force_config.start_exit_speed_dps)
    {
        if (++force_start_exit_cycles[index] >= g_chassis_force_config.start_exit_confirm_cycles)
        {
            force_start_active[index] = 0u;
            /* 运行中再次降到低速时，允许同一方向重新触发启速补偿。 */
            force_start_armed[index] = 1u;
            force_start_exit_cycles[index] = 0u;
            return 0u;
        }
    }
    else
    {
        force_start_exit_cycles[index] = 0u;
    }

    if (force_start_hold_cycles[index] > 0u)
    {
        force_start_hold_cycles[index]--;
    }
    else if (speed_abs <= g_chassis_force_config.start_speed_dps)
    {
        /* 保持时间后仍完全不动：短暂撤力，再允许下一次起步脉冲，避免堵转发热。 */
        force_start_active[index] = 0u;
        force_start_armed[index] = 1u;
        force_start_cooldown_cycles[index] = g_chassis_force_config.start_retry_cooldown_cycles;
        return 0u;
    }
    return 1u;
}

const float *ChassisForceTorque(float fx, float fy, float mz, float gyro_z_dps,
                                DJIMotorInstance *const motors[4])
{
    float wheel_force[4];
    float current_target[4];
    float max_abs = 0.0f;
    float gyro_damping = 0.0f;
    const uint8_t force_command_active =
        fabsf(fx) > g_chassis_force_config.start_input_deadband_n ||
        fabsf(fy) > g_chassis_force_config.start_input_deadband_n ||
        fabsf(mz) > g_chassis_force_config.start_input_deadband_nm;

    ApplyForceStartCompensation(&fx, &fy, &mz, motors);
    if (isfinite(gyro_z_dps))
    {
        gyro_damping = float_constrain(g_chassis_force_config.gyro_damping_nm_per_dps * g_gimbal_config.gyro_yaw_direction *
                                           gyro_z_dps,
                                       -g_chassis_force_config.gyro_damping_limit_nm,
                                       g_chassis_force_config.gyro_damping_limit_nm);
        mz -= gyro_damping;
    }
    if (!AllocateChassisForce(fx, fy, mz, wheel_force))
    {
        memset(force_current_ref, 0, sizeof(force_current_ref));
        memset(force_velocity_damping, 0, sizeof(force_velocity_damping));
    }
    else
    {
        for (uint8_t i = 0; i < 4; ++i)
        {
            float feedforward_current = WheelForceToCurrent(wheel_force[i]);
            if (force_command_active)
            {
                force_velocity_damping[i] = VelocityDampingCurrent(motors[i]->measure.speed_aps);
            }
            else
            {
                /* 松开按键后主动反向制动；仅撤掉力矩会让底盘靠惯性滑行。 */
                force_velocity_damping[i] = float_constrain(
                    g_chassis_force_config.release_brake_raw_per_dps * motors[i]->measure.speed_aps,
                    -g_chassis_force_config.release_brake_limit_raw,
                    g_chassis_force_config.release_brake_limit_raw);
            }
            /* 每个轮子独立锁存起步补偿，避免 RF 在静摩擦附近反复开关。 */
            if (ForceStartCompensationActive(i,
                                             feedforward_current,
                                             motors[i]->measure.speed_aps))
            {
                feedforward_current += copysignf(g_chassis_force_config.start_wheel_comp_raw,
                                                 feedforward_current);
            }
            current_target[i] = force_command_active ?
                                    feedforward_current - force_velocity_damping[i] :
                                    -force_velocity_damping[i];
            /* 超过安全轮速后禁止继续沿当前转向加速，直接施加有限反向制动。 */
            if (fabsf(motors[i]->measure.speed_aps) > g_chassis_force_config.speed_limit_dps &&
                current_target[i] * motors[i]->measure.speed_aps > 0.0f)
            {
                current_target[i] = -copysignf(g_chassis_force_config.overspeed_brake_raw,
                                               motors[i]->measure.speed_aps);
            }
            if (fabsf(current_target[i]) > max_abs)
                max_abs = fabsf(current_target[i]);
        }
        if (max_abs > g_chassis_force_config.current_limit_raw)
        {
            float scale = g_chassis_force_config.current_limit_raw / max_abs;
            for (uint8_t i = 0; i < 4; ++i)
                current_target[i] *= scale;
        }
        /*
         * 底盘电机通过 DJIMotorInit() 注册，旧 PowerControl() 使用的是另一套
         * 注册表，当前并没有对这四个电机实施正式裁判功率控制。这里不能再用
         * “24 V * 指令电流”做伪功率缩放，否则启速补偿会被压到约 537 raw，
         * RF 在落地静摩擦下无法启动。正式功率控制接入共享电机注册表后，
         * 再在底层统一限幅；当前先使用电流总限幅和变化率保护。
         */
        for (uint8_t i = 0; i < 4; ++i)
        {
            current_target[i] = float_constrain(current_target[i],
                                                -g_chassis_force_config.current_limit_raw, g_chassis_force_config.current_limit_raw);
            force_current_ref[i] = ConstrainSlew(current_target[i], force_current_ref[i]);
        }
    }

    return force_current_ref;
}

const float *ChassisForceTrack(float fx, float fy, float mz, const Chassis_Ctrl_Cmd_s *command,
                                float gyro_z_dps, DJIMotorInstance *const motors[4])
{
    float fx_cmd, fy_cmd, mz_cmd;

    /* 力环输出 = 目标力前馈 + 反馈误差修正。
       只使用 PID 输出会在误差归零后把电机电流降回零，无法维持目标力。 */
    fx_cmd = command->fx_ref +
             PIDCalculate(&force_pid_x, fx, command->fx_ref);
    fy_cmd = command->fy_ref +
             PIDCalculate(&force_pid_y, fy, command->fy_ref);
    mz_cmd = command->mz_ref +
             PIDCalculate(&force_pid_z, mz, command->mz_ref);
    return ChassisForceTorque(fx_cmd, fy_cmd, mz_cmd, gyro_z_dps, motors);
}

void ChassisForceAdmittance(float fx, float fy, float mz, const Chassis_Ctrl_Cmd_s *command,
                           float velocity[3])
{
    float dt = DWT_GetDeltaT(&admittance_dwt_cnt);
    if (dt < g_chassis_force_config.admittance_dt_min_s || dt > g_chassis_force_config.admittance_dt_max_s)
        dt = g_chassis_force_config.admittance_dt_fallback_s;
    admittance_vx += (command->fx_ref - fx - g_chassis_force_config.admittance_damping_x * admittance_vx) /
                     g_chassis_force_config.admittance_mass_x * dt;
    admittance_vy += (command->fy_ref - fy - g_chassis_force_config.admittance_damping_y * admittance_vy) /
                     g_chassis_force_config.admittance_mass_y * dt;
    admittance_wz += (command->mz_ref - mz - g_chassis_force_config.admittance_damping_z * admittance_wz) /
                    g_chassis_force_config.admittance_mass_z * dt;
    admittance_vx = float_constrain(admittance_vx, -g_chassis_force_config.admittance_max_vx, g_chassis_force_config.admittance_max_vx);
    admittance_vy = float_constrain(admittance_vy, -g_chassis_force_config.admittance_max_vy, g_chassis_force_config.admittance_max_vy);
    admittance_wz = float_constrain(admittance_wz, -g_chassis_force_config.admittance_max_wz, g_chassis_force_config.admittance_max_wz);

    velocity[0] = admittance_vx;
    velocity[1] = admittance_vy;
    velocity[2] = admittance_wz;
}

void ChassisForceInit(void)
{
    memset(&chassis_force_feedback, 0, sizeof(chassis_force_feedback));
    memset(&chassis_force_sensor, 0, sizeof(chassis_force_sensor));
    force_sensor_last_seq = 0u;
    force_sensor_stale_cycles = g_chassis_force_config.sensor_timeout_cycles + 1u;
    force_sensor_have_sample = 0u;
    force_current_log_count = 0u;
    memset(force_current_filter_history, 0, sizeof(force_current_filter_history));
    memset(force_current_filtered, 0, sizeof(force_current_filtered));
    memset(force_current_filter_initialized, 0, sizeof(force_current_filter_initialized));
    memset(force_velocity_damping, 0, sizeof(force_velocity_damping));
    PID_Init_Config_s xy = g_chassis_force_config.force_xy_pid;
    PID_Init_Config_s yaw = g_chassis_force_config.force_yaw_pid;
    PIDInit(&force_pid_x, &xy);
    PIDInit(&force_pid_y, &xy);
    PIDInit(&force_pid_z, &yaw);
}
