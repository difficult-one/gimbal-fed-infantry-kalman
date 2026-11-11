
/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.c
 * @brief      mpu6500 module driver, configurate MPU6500 and Read the Accelerator
 *             and Gyrometer data using SPI interface      
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#include "ins.h"
#include "ins_config.h"
#include "build_config.h"
#include <math.h>
#include <string.h>
#include "pid.h"
#include "imu_temperature.h"
#include "math_utils.h"
#include "bsp_dwt.h"
#include "mpu6500_spi.h"
#include "quaternion_ekf.h"
#include "math_units.h"
#include "SEGGER_RTT.h"
#if defined(WHEEL_IMU_CAPTURE) && WHEEL_IMU_CAPTURE
#include "wheel_imu_capture.h"
#endif
#include "bsp_log.h"
static const INS_Init_Config *ins_config;
float MPU6500_ACCEL_SEN = MPU6500_ACCEL_8G_SEN;
float MPU6500_GYRO_SEN = MPU6500_GYRO_1000_SEN;
																			
static INS_t INS;
static INS_MotionSnapshot motion_snapshot;
static INS_SampleDebug sample_debug;
static uint32_t last_sample_time_us;
static uint8_t have_sample_time, read_interrupted;
static IMURecoveryState motion_recovery={.pending=1};
static IMURigidState rigid_state;
static IMURigidStatsState rigid_stats;
static float imu_lever_b[3];
static uint8_t lever_configured;
static IMUYawBiasState yaw_bias;
static uint32_t chassis_still_time, yaw_encoder_time;
static uint8_t chassis_still;
static uint32_t chassis_reason;
static float yaw_encoder_deg;

void INS_SetLeverArm(const float sensor_m[3])
{
    memcpy(imu_lever_b, sensor_m, sizeof(imu_lever_b));
    memset(&rigid_state, 0, sizeof(rigid_state));
    lever_configured = 1u;
}

void INS_SetChassisStill(uint32_t time_us, uint8_t still, uint32_t reason)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    chassis_still_time = time_us;
    chassis_still = still;
    chassis_reason = reason;
    __set_PRIMASK(key);
}

uint8_t INS_GetMotionSnapshot(INS_MotionSnapshot *snapshot)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    *snapshot = motion_snapshot;
    __set_PRIMASK(key);
    return snapshot->valid;
}
void INS_GetSampleDebug(INS_SampleDebug *debug)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    *debug = sample_debug;
    __set_PRIMASK(key);
}

static mpu_data_t  mpu_data;
static uint8_t calibration_status;
static float calibration_temperature;

void INS_UpdateYawBias(const IMUYawBiasInput *input, uint32_t encoder_time_us)
{
    IMUYawBiasInput checked = *input;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    checked.valid &= chassis_still &&
        (uint32_t)(input->time_us-chassis_still_time) <= ins_config->yaw_aid_max_age_us;
    if (!chassis_still) checked.inhibit |= chassis_reason ? chassis_reason : IMU_AID_BIT(IMU_AID_CALLER);
    if ((uint32_t)(input->time_us-chassis_still_time) > ins_config->yaw_aid_max_age_us)
        checked.inhibit |= IMU_AID_BIT(IMU_AID_CHASSIS_TIME);
    __set_PRIMASK(key);
    checked.valid &= ins_config->yaw_bias_aid_enabled && calibration_status == 1u;
    if (!ins_config->yaw_bias_aid_enabled) checked.inhibit |= IMU_AID_BIT(IMU_AID_DISABLED);
    if (calibration_status != 1u) checked.inhibit |= IMU_AID_BIT(IMU_AID_CALIBRATION);
    IMUYawBiasUpdate(&yaw_bias, &checked);
    yaw_encoder_deg = input->encoder_deg;
    yaw_encoder_time = encoder_time_us;
}

void INS_GetCompensationDebug(IMURigidStats *rigid, IMUYawBiasDebug *aid)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    *rigid=rigid_stats.published;
    *aid=yaw_bias.debug;
    aid->settle_s=yaw_bias.settle_s;
    aid->window_s=yaw_bias.window_s;
    __set_PRIMASK(key);
}

void INS_GetCalibrationDebug(INS_CalibrationDebug *snapshot)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    snapshot->temperature = mpu_data.Temperature;
    snapshot->calibration_temperature = calibration_temperature;
    snapshot->gyro_offset_z = mpu_data.GyroOffset[2];
    snapshot->status = calibration_status;
    snapshot->yaw_trim_z = yaw_bias.trim_z;
    snapshot->trim_status = yaw_bias.status;
    snapshot->trim_updates = yaw_bias.updates;
    snapshot->yaw_encoder_deg = yaw_encoder_deg;
    snapshot->encoder_time_us = yaw_encoder_time;
    __set_PRIMASK(key);
}
static IMU_Param_t IMU_Param;
static PIDInstance TempCtrl = {0};

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

static uint32_t INS_DWT_Count = 0;
static uint8_t INS_Init_flag = 0;                           /* INS system init flag */


static void mpu_offset_call(void);
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);
static INS_SampleStatus mpu_get_data(int16_t raw[7]);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
/**
 * @brief 温度控制
 *
 */
static void IMU_Temperature_Ctrl(void)
{
    PIDCalculate(&TempCtrl,mpu_data.Temperature , ins_config->heater_target_c);
    IMU_Temperature_Update(float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}
static INS_SampleStatus MpuReadRaw(int16_t raw[7])
{
    uint8_t fresh;
    HAL_StatusTypeDef spi_status = mpu_read_sample(raw, &fresh);
    uint32_t read_time_us = (uint32_t)DWT_GetTimeline_us();
    INS_SampleStatus status;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    sample_debug.attempt_seq++;
    sample_debug.read_time_us = read_time_us;
    switch (spi_status)
    {
    case HAL_OK:
        if (fresh) {
            sample_debug.accepted_seq++;
            status = INS_SAMPLE_FRESH;
        } else {
            sample_debug.not_ready_count++;
            status = INS_SAMPLE_NOT_READY;
        }
        break;
    case HAL_BUSY:
        sample_debug.busy_count++;
        status = INS_SAMPLE_SPI_BUSY;
        break;
    case HAL_TIMEOUT:
        sample_debug.timeout_count++;
        status = INS_SAMPLE_SPI_TIMEOUT;
        break;
    default:
        sample_debug.error_count++;
        status = INS_SAMPLE_SPI_ERROR;
        break;
    }
    sample_debug.status = status;
    __set_PRIMASK(key);
    return status;
}

/* Retain the magnetometer read entry point for diagnostics. */
void ist8310_get_data(uint8_t *buff)
{
    (void)mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
}

static INS_SampleStatus mpu_get_data(int16_t raw[7])
{
    INS_SampleStatus status = MpuReadRaw(raw);
    if (status != INS_SAMPLE_FRESH)
        return status;
    for (unsigned i = 0; i < 3; ++i) {
        mpu_data.Accel[i] = raw[i] * MPU6500_ACCEL_SEN * mpu_data.AccelScale;
        mpu_data.Gyro[i] = raw[4u+i] * MPU6500_GYRO_SEN - mpu_data.GyroOffset[i];
    }
    mpu_data.Temperature = raw[3]/333.87f + 21.0f;
    return status;
}

/* Estimate initial roll/pitch from 100 distinct accelerometer samples. */
static uint8_t InitQuaternion(float q[4])
{
    float sum[3] = {0};
    const uint32_t start_us = (uint32_t)DWT_GetTimeline_us();
    unsigned count = 0;
    while (count < ins_config->initial_attitude_samples && (uint32_t)((uint32_t)DWT_GetTimeline_us() - start_us) < ins_config->initial_attitude_timeout_us)
    {
        int16_t raw[7];
        if (mpu_get_data(raw) == INS_SAMPLE_FRESH) {
            for (unsigned i = 0; i < 3; ++i)
                sum[i] += mpu_data.Accel[i];
            ++count;
        }
        DWT_Delay(ins_config->initial_attitude_poll_s);
    }
    if (count != ins_config->initial_attitude_samples)
        return 0u;

    const float norm = sqrtf(Dot3d(sum, sum));
    if (!isfinite(norm) || norm <= 0.0f)
        return 0u;
    q[0] = sqrtf(0.5f * (1.0f + sum[2]/norm));
    q[3] = 0.0f;
    if (q[0] > 1e-6f) {
        q[1] = sum[1]/(2.0f*norm*q[0]);
        q[2] = -sum[0]/(2.0f*norm*q[0]);
    } else {
        q[0] = 0.0f;
        q[1] = 1.0f; /* Gravity is antiparallel to navigation Z. */
        q[2] = 0.0f;
    }
    return 1u;
}

/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
attitude_t *INS_Init(const INS_Init_Config *config)
{
 	if (!INS.init)
    {
        INS.init = 1;
        ins_config = config;
    }else{
        return (attitude_t *)&INS.Gyro;
    }
	
    HAL_StatusTypeDef config_status = mpu6500_device_init();
    if (config_status != HAL_OK) {
        sample_debug.startup_status = INS_START_CONFIG_FAILED;
        if (config_status == HAL_TIMEOUT) sample_debug.timeout_count++;
        else if (config_status == HAL_BUSY) sample_debug.busy_count++;
        else sample_debug.error_count++;
        return (attitude_t *)&INS.Gyro;
    }
    IMU_Temperature_Init(config->heater_period_ms, config->heater_initial_duty);
    IMU_Temperature_Update(0.0f);
    PID_Init_Config_s heater_pid = config->heater_pid;
    PIDInit(&TempCtrl, &heater_pid);
    mpu_offset_call();
    memcpy(IMU_Param.scale, config->scale, sizeof(IMU_Param.scale));
    IMU_Param.Yaw = config->yaw_deg;
    IMU_Param.Pitch = config->pitch_deg;
    IMU_Param.Roll = config->roll_deg;
    IMU_Param.flag = 1;

    float init_quaternion[4] = {0};
    if (!InitQuaternion(init_quaternion)) {
        sample_debug.startup_status = INS_START_ATTITUDE_FAILED;
        IMU_Temperature_Update(0.0f);
        return (attitude_t *)&INS.Gyro;
    }
    last_sample_time_us = sample_debug.read_time_us;
    have_sample_time = 1u;
    IMU_QuaternionEKF_Init(init_quaternion, config->ekf_quaternion_noise, config->ekf_bias_noise,
                           config->ekf_accel_noise, config->ekf_fading, config->ekf_accel_lpf_s);
    INS.AccelLPF = config->motion_lpf_s;
	DWT_GetDeltaT(&INS_DWT_Count);
    sample_debug.startup_status = INS_START_READY;
	INS_Init_flag = 1;
	return (attitude_t *)&INS.Gyro;
}

/**
	* @brief  get the offset data of MPU6500
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
static void mpu_offset_call(void)
{
    const uint32_t start_us = (uint32_t)DWT_GetTimeline_us();
    const unsigned required_samples = ins_config->calibration_samples;
    calibration_status = 0u;
    uint32_t temperature_us = start_us;
    while ((uint32_t)((uint32_t)DWT_GetTimeline_us() - start_us) < ins_config->calibration_timeout_us)
    {
        float gyro_sum[3] = {0}, gyro_min[3] = {0}, gyro_max[3] = {0};
        float gravity_sum = 0.0f, gravity_min = 0.0f, gravity_max = 0.0f;
        unsigned count = 0u;
        uint8_t still = 1u;

        DWT_Delay(ins_config->calibration_retry_s);
        while (count < required_samples && still &&
               (uint32_t)((uint32_t)DWT_GetTimeline_us() - start_us) < ins_config->calibration_timeout_us)
        {
            int16_t raw[7];
            INS_SampleStatus status = MpuReadRaw(raw);
            if (status == INS_SAMPLE_FRESH) {
                float gravity = sqrtf((float)raw[0]*raw[0] + (float)raw[1]*raw[1] +
                                      (float)raw[2]*raw[2]) * MPU6500_ACCEL_SEN;
                gravity_sum += gravity;
                if (count == 0u || gravity < gravity_min) gravity_min = gravity;
                if (count == 0u || gravity > gravity_max) gravity_max = gravity;
                still = gravity_max - gravity_min <= ins_config->calibration_accel_span;
                for (unsigned i = 0; i < 3; ++i) {
                    float gyro = raw[4u+i] * MPU6500_GYRO_SEN;
                    gyro_sum[i] += gyro;
                    if (count == 0u || gyro < gyro_min[i]) gyro_min[i] = gyro;
                    if (count == 0u || gyro > gyro_max[i]) gyro_max[i] = gyro;
                    still &= gyro_max[i] - gyro_min[i] <= ins_config->calibration_gyro_span;
                }
                mpu_data.Temperature = raw[3]/333.87f + 21.0f;
                temperature_us = sample_debug.read_time_us;
                IMU_Temperature_Ctrl();
                ++count;
            } else if (status != INS_SAMPLE_NOT_READY ||
                       (uint32_t)(sample_debug.read_time_us - temperature_us) > ins_config->sample_gap_us) {
                IMU_Temperature_Update(0.0f);
            }
            DWT_Delay(ins_config->calibration_poll_s);
        }

        if (count != required_samples || !still)
            continue;
        const float gravity_mean = gravity_sum/(float)count;
        uint8_t plausible = fabsf(gravity_mean - ins_config->calibration_gravity) <= ins_config->calibration_gravity_tolerance;
        for (unsigned i = 0; i < 3; ++i) {
            mpu_data.GyroOffset[i] = gyro_sum[i]/(float)count;
            plausible &= fabsf(mpu_data.GyroOffset[i]) <= ins_config->gyro_offset_limit[i];
        }
        if (plausible) {
            mpu_data.gNorm = gravity_mean;
            calibration_status = 1u;
            break;
        }
    }

    if (!calibration_status) {
        memcpy(mpu_data.GyroOffset, ins_config->fallback_gyro_offset, sizeof(mpu_data.GyroOffset));
        mpu_data.gNorm = ins_config->fallback_gravity;
        mpu_data.TempWhenCali = ins_config->fallback_temperature_c;
        calibration_status = 2u;
        IMU_Temperature_Update(0.0f);
    }
    calibration_temperature = mpu_data.Temperature;
    mpu_data.AccelScale = 9.81f/mpu_data.gNorm;
}


#if defined(WHEEL_IMU_CAPTURE) && WHEEL_IMU_CAPTURE
static void CaptureIMUAttempt(uint32_t time_us, uint32_t generation, INS_SampleStatus status,
                              const int16_t raw[7], const float *accel_at_intersection)
{
    if (!WheelIMUCaptureActive()) return;
    const INS_MotionSnapshot *snapshot = &motion_snapshot;
    WheelIMUCaptureImu sample = {
        .time_us=time_us, .accepted_seq=snapshot->seq, .attempt_seq=sample_debug.attempt_seq,
        .generation=generation, .read_status=(uint8_t)status, .attitude_flags=snapshot->attitude_flags,
        .rigid_reject=(uint8_t)rigid_state.trace.reject,
        .accel_at_intersection_b={NAN,NAN,NAN}
    };
    if (status == INS_SAMPLE_FRESH) {
        sample.valid_flags = (uint8_t)((!!snapshot->valid) | ((!!snapshot->center_valid)<<1) |
                                      ((!!snapshot->motion_ready)<<2));
        memcpy(sample.accel_counts, raw, sizeof(sample.accel_counts));
        memcpy(sample.gyro_counts, raw+4, sizeof(sample.gyro_counts));
    }
    memcpy(sample.accel_b, snapshot->accel_raw, sizeof(sample.accel_b));
    memcpy(sample.gyro_b, snapshot->gyro_rad, sizeof(sample.gyro_b));
    memcpy(sample.q, snapshot->q, sizeof(sample.q));
    memcpy(sample.gravity_b, snapshot->gravity_b, sizeof(sample.gravity_b));
    if (accel_at_intersection)
        memcpy(sample.accel_at_intersection_b, accel_at_intersection, sizeof(sample.accel_at_intersection_b));
    memcpy(sample.motion_n, snapshot->motion_n, sizeof(sample.motion_n));
    memcpy(sample.center_n, snapshot->center_n, sizeof(sample.center_n));
    WheelIMUCaptureRecordImu(&sample);
}
#endif

void INS_Task(void)
{
	if(!INS_Init_flag)
	{
		return;
	}
	static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};

    const float task_dt = DWT_GetDeltaT(&INS_DWT_Count);
    const uint8_t update_temperature = (count++ % 2u) == 0u;
    int16_t raw[7] = {0};
    INS_SampleStatus read_status = mpu_get_data(raw);
    const uint32_t sample_time_us = sample_debug.read_time_us;
    DWT_ClockDebug clock;
    DWT_GetClockDebug(&clock);

    const uint32_t sample_dt_us = (uint32_t)(sample_time_us - last_sample_time_us);
    const uint8_t sample_gap = have_sample_time && sample_dt_us > ins_config->sample_gap_us;
    const uint8_t clock_ok = clock.generation == motion_recovery.clock_generation;
    const uint8_t fresh = read_status == INS_SAMPLE_FRESH;
    if (read_status == INS_SAMPLE_NOT_READY && clock_ok && (!sample_gap || read_interrupted)) {
#if defined(WHEEL_IMU_CAPTURE) && WHEEL_IMU_CAPTURE
        CaptureIMUAttempt(sample_time_us, clock.generation, read_status, raw, NULL);
#endif
        return;
    }
    if (sample_gap && !read_interrupted)
        sample_debug.gap_count++;

    const uint8_t time_ok = clock_ok && !read_interrupted &&
        isfinite(task_dt) && task_dt > 0.0f && task_dt <= 0.02f &&
        (!have_sample_time || (sample_dt_us != 0u && !sample_gap));
    const float dt = have_sample_time ? sample_dt_us*1e-6f : task_dt;
    if (fresh) {
        last_sample_time_us = sample_time_us;
        have_sample_time = 1u;
        read_interrupted = 0u;
    } else {
        read_interrupted = 1u;
        IMU_Temperature_Update(0.0f);
    }

    {
        float accel[3] = {0}, gyro_rad[3] = {0};
        uint8_t input_ok = fresh;
        if (fresh) {
            memcpy(accel, mpu_data.Accel, sizeof(accel));
            memcpy(gyro_rad, mpu_data.Gyro, sizeof(gyro_rad));
            IMU_Param_Correction(&IMU_Param, gyro_rad, accel);
            gyro_rad[2] -= yaw_bias.trim_z;
            for (uint8_t i = 0; i < 3; ++i)
                input_ok &= isfinite(gyro_rad[i]) && isfinite(accel[i]);
        }
        if (!time_ok || !input_ok)
        {
            IMU_QuaternionEKF_SetGravityRecoveryAllowed(0);
            INS_MotionSnapshot next = motion_snapshot;
            (void)IMURecoveryUpdate(&motion_recovery, clock.generation, 0, 0, 0);
            /* A fault report is not a sample. A fresh read after a time
             * discontinuity seeds the new interval without updating attitude. */
            if (fresh) {
                next.seq++;
                next.time_us = sample_time_us;
                memcpy(next.accel_raw, accel, sizeof(next.accel_raw));
                memcpy(next.gyro_rad, gyro_rad, sizeof(next.gyro_rad));
            }
            next.valid = next.center_valid = next.motion_ready = 0u;
            next.clock_generation = clock.generation;
            next.recovery_good_us = 0u;
            next.attitude_flags = 16u;
            next.attitude_chi_ratio = NAN;
            rigid_state.ready = rigid_state.gyro_count = 0u;
            memset(&rigid_state.trace, 0, sizeof(rigid_state.trace));
            rigid_state.trace.reject = fresh && !time_ok ? IMU_RIGID_TIME :
                (read_status == INS_SAMPLE_NOT_READY ? IMU_RIGID_TIME : IMU_RIGID_INPUT);
            IMURigidStatsPush(&rigid_stats, sample_time_us, &rigid_state.trace,
                0u, INS.MotionAccel_n, next.center_n);
            uint32_t key = __get_PRIMASK();
            __disable_irq();
            motion_snapshot = next;
            __set_PRIMASK(key);
#if defined(WHEEL_IMU_CAPTURE) && WHEEL_IMU_CAPTURE
            CaptureIMUAttempt(sample_time_us, clock.generation, read_status, raw, NULL);
#endif
            goto ins_temperature_control;
        }

        memcpy(INS.Accel, accel, sizeof(accel));
        /* EKF consumes rad/s. Public gyro feedback stays in deg/s for motor PID. */
        for (uint8_t i = 0; i < 3; ++i)
            INS.Gyro[i] = gyro_rad[i] * RAD_2_DEGREE;

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        /* 先扣除确定的旋转杆臂项，再把加速度交给姿态 EKF。
         * 否则缓慢 YAW/PITCH 时，EKF 会把其中一部分当作重力方向偏转；
         * 下游再扣完整杆臂项无法消除已经污染的姿态，反而产生虚假平移。
         * 使用本帧 EKF 预测所用的先前零偏，避免把本帧观测更新的零偏增量
         * 差分成机械角加速度。INS.Accel 仍保留传感器实测值供诊断。 */
        float physical_gyro[3] = {gyro_rad[0]-QEKF_INS.GyroBias[0],
                                 gyro_rad[1]-QEKF_INS.GyroBias[1], gyro_rad[2]};
        uint8_t rigid_valid = lever_configured &&
            IMURigidPrepare(&rigid_state, sample_time_us, physical_gyro, imu_lever_b);
        float attitude_accel[3];
        for (uint8_t i=0; i<3; ++i)
            attitude_accel[i] = INS.Accel[i] - (rigid_valid ?
                rigid_state.trace.tangent[i]+rigid_state.trace.centripetal[i] : 0.0f);
        /* 首帧/时间异常时按原始加速度更新姿态，本帧中心加速度不放行。 */
        /* A 板随云台安装时，低 IMU 角速度不代表底盘未自旋。
         * 复用底盘严格静止许可（含原始轮速/命令/模式/新鲜度），
         * 缺失或过期时只禁止强制重捕获，不停掉正常姿态更新。 */
        uint32_t still_key=__get_PRIMASK();
        __disable_irq();
        uint8_t gravity_recovery_allowed=chassis_still &&
            (uint32_t)((uint32_t)DWT_GetTimeline_us()-chassis_still_time)<=ins_config->yaw_aid_max_age_us;
        __set_PRIMASK(still_key);
        IMU_QuaternionEKF_SetGravityRecoveryAllowed(gravity_recovery_allowed);
        IMU_QuaternionEKF_Update(gyro_rad[0], gyro_rad[1], gyro_rad[2],
            attitude_accel[0], attitude_accel[1], attitude_accel[2], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
        /* EKF 使用近似快平方根且观测更新后不再归一化；旋转/去重力公式
         * 要求单位四元数。这里归一化发布副本，让去重力与刚体投影使用
         * 同一单位旋转，不修改 EKF 内部状态或公共 YAW/PITCH 角度目标。 */
        float rotation_norm2 = 0.0f;
        for (uint8_t i=0; i<4; ++i) rotation_norm2 += INS.q[i]*INS.q[i];
        if (isfinite(rotation_norm2) && fabsf(rotation_norm2-1.0f)<0.05f)
        {
            float inv_norm = 1.0f/sqrtf(rotation_norm2);
            for (uint8_t i=0; i<4; ++i) INS.q[i] *= inv_norm;
        }

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        float motion_b_raw[3];
        /* before/after 的低通共用 SPI 采样时间差，避免任务起点抖动制造比较差异。 */
        float motion_dt = rigid_valid ? rigid_state.trace.dt_us*1e-6f : dt;
        for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
        {
            motion_b_raw[i] = INS.Accel[i] - gravity_b[i];
            INS.MotionAccel_b[i] = motion_b_raw[i] * motion_dt / (INS.AccelLPF + motion_dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + motion_dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Roll;
        INS.Roll = QEKF_INS.Pitch;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

        /* 先在私有局部帧组装，发布时仅复制；不在整个 EKF/SPI 期间关中断。 */
        INS_MotionSnapshot next = {0};
        next.seq = motion_snapshot.seq + 1u;
        next.time_us = sample_time_us;
        memcpy(next.q, INS.q, sizeof(next.q));
        memcpy(next.xn, INS.xn, sizeof(next.xn));
        memcpy(next.accel_raw, INS.Accel, sizeof(next.accel_raw));
        memcpy(next.motion_b, INS.MotionAccel_b, sizeof(next.motion_b));
        memcpy(next.motion_n, INS.MotionAccel_n, sizeof(next.motion_n));
        memcpy(next.gyro_rad, gyro_rad, sizeof(next.gyro_rad));
        next.yaw_deg = INS.YawTotalAngle;
        float qnorm = 0.0f;
        for (uint8_t i = 0; i < 4; ++i) qnorm += next.q[i] * next.q[i];
        next.valid = isfinite(qnorm) && fabsf(qnorm - 1.0f) < 0.05f && dt > 0.0f && dt <= 0.02f &&
                     QEKF_INS.IMU_QuaternionEKF.FirstMatError == ARM_MATH_SUCCESS;
        for (uint8_t i = 0; i < 3; ++i)
            next.valid &= isfinite(next.motion_n[i]) && isfinite(next.gyro_rad[i]) && isfinite(next.xn[i]);
        if (rigid_valid)
        {
            /* attitude_accel 已扣杆臂，这里只去重力，绝不能重复扣除。
             * 求导/低通仍在 INS 全采样率完成，底盘端仅消费同帧快照。 */
            float center_b_raw[3];
            for (uint8_t i=0; i<3; ++i) center_b_raw[i] = attitude_accel[i]-gravity_b[i];
            next.center_valid = IMURigidProjectFiltered(&rigid_state, INS.q,
                                                        center_b_raw, INS.AccelLPF, next.center_n) && next.valid;
        }
        else if (!lever_configured) rigid_state.trace.reject=IMU_RIGID_DISABLED;
        memcpy(next.gravity_b,gravity_b,sizeof(next.gravity_b));
        next.pitch_rad=INS.Pitch*0.01745329252f;
        next.roll_rad=INS.Roll*0.01745329252f;
        next.attitude_chi_ratio=QEKF_INS.ChiSquare_Data[0]/QEKF_INS.ChiSquareTestThreshold;
        next.attitude_flags=(QEKF_INS.ConvergeFlag ? 1u : 0u) |
            (QEKF_INS.StableFlag ? 2u : 0u) |
            (QEKF_INS.IMU_QuaternionEKF.SkipEq5 ? 4u : 0u) |
            (QEKF_INS.IMU_QuaternionEKF.FirstMatError!=ARM_MATH_SUCCESS ? 8u : 0u) |
            (QEKF_INS.GravityRecoveryAllowed ? 32u : 0u) |
            (QEKF_INS.GravityRecoveryActive ? 64u : 0u);
        const float *center=next.center_valid ? next.center_n : next.motion_n;
        /* This is recovery qualification, not a zero-speed detector or bias
         * learner. Ordinary dynamics remain allowed after recovery completes. */
        uint8_t settled=(next.attitude_flags&5u)==1u &&
            isfinite(next.attitude_chi_ratio) && next.attitude_chi_ratio>=0.0f && next.attitude_chi_ratio<=1.0f &&
            QEKF_INS.gyro_norm<0.05f && fabsf(QEKF_INS.accl_norm-9.81f)<0.3f &&
            hypotf(center[0],center[1])<0.10f;
        next.motion_ready=IMURecoveryUpdate(&motion_recovery,clock.generation,
            (uint32_t)(dt*1000000.0f),next.valid,settled);
        next.clock_generation=clock.generation;
        next.recovery_good_us=motion_recovery.good_us;
        /* 全 INS 采样率累计，低优先级任务每秒只打印摘要；不在这里 printf/开方。 */
        IMURigidStatsPush(&rigid_stats, sample_time_us, &rigid_state.trace,
            next.center_valid, INS.MotionAccel_n, next.center_n);
        uint32_t key = __get_PRIMASK();
        __disable_irq();
        motion_snapshot = next;
        __set_PRIMASK(key);
#if defined(WHEEL_IMU_CAPTURE) && WHEEL_IMU_CAPTURE
        CaptureIMUAttempt(sample_time_us, clock.generation, read_status, raw, attitude_accel);
#endif
    }

ins_temperature_control:
    if (fresh && update_temperature)
        IMU_Temperature_Ctrl();
}
/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[0] = c_11 * gyro_temp[0] +
              c_12 * gyro_temp[1] +
              c_13 * gyro_temp[2];
    gyro[1] = c_21 * gyro_temp[0] +
              c_22 * gyro_temp[1] +
              c_23 * gyro_temp[2];
    gyro[2] = c_31 * gyro_temp[0] +
              c_32 * gyro_temp[1] +
              c_33 * gyro_temp[2];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[0] = c_11 * accel_temp[0] +
               c_12 * accel_temp[1] +
               c_13 * accel_temp[2];
    accel[1] = c_21 * accel_temp[0] +
               c_22 * accel_temp[1] +
               c_23 * accel_temp[2];
    accel[2] = c_31 * accel_temp[0] +
               c_32 * accel_temp[1] +
               c_33 * accel_temp[2];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}
/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
