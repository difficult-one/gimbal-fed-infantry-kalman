#include "chassis_log.h"
#include "build_config.h"
#include "config.h"
#include "chassis_motion.h"
#include "bsp_dwt.h"
#include "ins.h"
#include "robot_cmd.h"
#include "gimbal.h"
#include "dji_motor.h"
#include "SEGGER_RTT.h"
#include "wheel_imu_capture.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static uint8_t motion_log_ready;
static uint32_t motion_transport_drop;

void ChassisLogInit(void)
{
    /* BSP configures RTT before the chassis enables its producer. */
    motion_log_ready = 1u;
}

_Static_assert(sizeof(ChassisMotionState) == 444u, "Update MCL1 decoder and version after changing state layout");

/* RTT 精简 printf 不支持浮点；按字段标注的单位转换为整数，不引入浮点格式化库。
 * 非有限值记为 INT32_MIN，超范围值饱和，避免异常估计值转整数时发生未定义行为。 */
static int32_t MotionLogScaled(float value, float scale)
{
    if (!isfinite(value)) return INT32_MIN;
    float scaled = value * scale;
    if (scaled >= 2147483520.0f) return INT32_MAX;
    if (scaled <= -2147483520.0f) return -INT32_MAX;
    return (int32_t)scaled;
}

/* 追加失败后把长度置为容量；整包撤销，禁止把截断文本作为有效诊断发送。 */
static void MotionDiagAppend(char *text, unsigned capacity, unsigned *used, const char *fmt, ...)
{
    if (*used >= capacity) return;
    va_list args;
    va_start(args, fmt);
    int n=vsnprintf(text+*used, capacity-*used, fmt, args);
    va_end(args);
    if (n < 0 || (unsigned)n >= capacity-*used) *used=capacity;
    else *used += (unsigned)n;
}

static void MotionDiagVector(char *text, unsigned capacity, unsigned *used, const char *name,
                             const float *values, unsigned count, uint32_t samples,
                             uint8_t rms, float scale)
{
    MotionDiagAppend(text, capacity, used, " %s=", name);
    for (unsigned i=0; i<count; ++i) {
        float x=samples ? values[i]/samples : NAN;
        if (rms) x=sqrtf(x);
        MotionDiagAppend(text, capacity, used, "%s%ld", i ? "," : "", (long)MotionLogScaled(x, scale));
    }
}

static inline float MotionSpinReference(float max_omega)
{
    float requested = g_chassis_config.legacy_spin_command * g_chassis_config.legacy_omega_scale;
    return fmaxf(-max_omega, fminf(requested, max_omega));
}

static void MotionDiagnosticLog(void)
{
    static IMURigidStats rigid;
    static IMUYawBiasDebug aid;
    static uint32_t last_window, dropped;
    static char text[2560];
    INS_GetCompensationDebug(&rigid, &aid);
    if (!rigid.seq || rigid.seq==last_window) return;
    if (last_window) dropped += (uint32_t)(rigid.seq-last_window)-1u;
    last_window=rigid.seq;
    unsigned used=0;
    /* 与控制日志独立，每个完成的约 1 s 窗只尝试发送一次。ddrop 包括漏读秒窗
     * 和本诊断包发送失败；RTT 仍为 NO_BLOCK_SKIP，禁止拖慢 INS/控制任务。 */
    MotionDiagAppend(text, sizeof(text), &used,
        "[RIGID] win=%lu dver=1 start_t_us=%lu end_t_us=%lu n=%lu good=%lu bad=%lu reject=0x%08lx dt_us=%lu,%lu ddrop=%lu",
        (unsigned long)rigid.seq, (unsigned long)rigid.start_us, (unsigned long)rigid.end_us,
        (unsigned long)rigid.samples, (unsigned long)rigid.good, (unsigned long)rigid.rejected,
        (unsigned long)rigid.reject_mask, (unsigned long)rigid.dt_min_us,
        (unsigned long)rigid.dt_max_us, (unsigned long)dropped);
    MotionDiagVector(text,sizeof(text),&used,"alpha_rms_mradps2",rigid.alpha2,3,rigid.good,1,1000);
    MotionDiagVector(text,sizeof(text),&used,"alpha_pk_mradps2",rigid.alpha_peak,3,1,0,1000);
    MotionDiagVector(text,sizeof(text),&used,"tangent_rms_mmps2",rigid.tangent2,3,rigid.good,1,1000);
    MotionDiagVector(text,sizeof(text),&used,"tangent_pk_mmps2",rigid.tangent_peak,3,1,0,1000);
    MotionDiagVector(text,sizeof(text),&used,"centrip_rms_mmps2",rigid.centripetal2,3,rigid.good,1,1000);
    MotionDiagVector(text,sizeof(text),&used,"centrip_pk_mmps2",rigid.centripetal_peak,3,1,0,1000);
    MotionDiagVector(text,sizeof(text),&used,"before_rms_mmps2",rigid.before2,2,rigid.good,1,1000);
    MotionDiagVector(text,sizeof(text),&used,"after_rms_mmps2",rigid.after2,2,rigid.good,1,1000);
    MotionDiagVector(text,sizeof(text),&used,"before_mean_mmps2",rigid.before_sum,2,rigid.good,0,1000);
    MotionDiagVector(text,sizeof(text),&used,"after_mean_mmps2",rigid.after_sum,2,rigid.good,0,1000);
    MotionDiagVector(text,sizeof(text),&used,"corr_rms_mmps2",rigid.correction2,2,rigid.good,1,1000);
    MotionDiagVector(text,sizeof(text),&used,"cross_um2ps4",rigid.cross,2,rigid.good,0,1000000);
    MotionDiagAppend(text, sizeof(text), &used,
        "\r\n[AID] win=%lu dver=1 t_us=%lu now=0x%08lx last=0x%08lx last_t_us=%lu resets=%lu settle_ms=%ld window_ms=%ld hits=",
        (unsigned long)rigid.seq, (unsigned long)aid.time_us, (unsigned long)aid.reason,
        (unsigned long)aid.last_reason, (unsigned long)aid.last_block_us, (unsigned long)aid.resets,
        (long)MotionLogScaled(aid.settle_s,1000), (long)MotionLogScaled(aid.window_s,1000));
    for (unsigned i=0; i<IMU_AID_CAUSE_COUNT; ++i)
        MotionDiagAppend(text, sizeof(text), &used, "%s%lu", i ? "," : "", (unsigned long)aid.hits[i]);
    MotionDiagAppend(text, sizeof(text), &used, "\r\n");
    DWT_ClockDebug clock;
    DWT_GetClockDebug(&clock);
    MotionDiagAppend(text,sizeof(text),&used,
        "[CLOCK] dver=1 t_us=%lu hal_ms=%lu cyccnt=%lu demcr=%lu ctrl=%lu gen=%lu flags=%lu last_t_us=%lu\r\n",
        (unsigned long)DWT_GetTimeline_us(),(unsigned long)clock.tick_ms,
        (unsigned long)clock.cyccnt,(unsigned long)clock.demcr,(unsigned long)clock.ctrl,
        (unsigned long)clock.generation,(unsigned long)clock.last_flags,(unsigned long)clock.last_event_us);
    if (used >= sizeof(text) || SEGGER_RTT_Write(g_chassis_log_config.text_rtt_channel,text,used)!=used) dropped++;
}

#if WHEEL_IMU_CAPTURE
static uint8_t PrepareCapture(ChassisMotionState *state, WheelIMUCaptureConfig *metadata)
{
    INS_MotionSnapshot imu;
    INS_GetMotionSnapshot(&imu);
    ChassisMotionGetState(state);
    const uint32_t now = (uint32_t)DWT_GetTimeline_us();
    DWT_ClockDebug clock;
    DWT_GetClockDebug(&clock);
    if (state->mode != MC_SHADOW || !imu.valid || !imu.motion_ready ||
        imu.clock_generation != clock.generation ||
        (uint32_t)(now - imu.time_us) > g_chassis_motion_config.sample_max_us)
        return 0u;

    ChassisMotionConfig config;
    ChassisMotionGetConfig(&config);
    if (config.wheels.type != CK_CUSTOM || config.algebra_y_sign != -1 || config.center_y_sign != -1)
        return 0u;

    *metadata = (WheelIMUCaptureConfig){
        .version = config.version,
        .frame_revision = g_chassis_log_config.frame_revision,
        .spin_ref_radps = MotionSpinReference(config.max_omega),
        .yaw_encoder_sign = g_chassis_imu_config.yaw_encoder_sign,
        .yaw_zero_deg = g_gimbal_config.yaw_align_ecd * ECD_ANGLE_COEF_DJI,
        .algebra_y_sign = config.algebra_y_sign,
        .center_y_sign = config.center_y_sign,
    };
    memcpy(metadata->radius_m, config.wheels.radius_m, sizeof(metadata->radius_m));
    memcpy(metadata->gear_ratio, config.wheels.gear_ratio, sizeof(metadata->gear_ratio));
    memcpy(metadata->motor_sign, config.wheels.motor_sign, sizeof(metadata->motor_sign));
    memcpy(metadata->rows, config.wheels.rows, sizeof(metadata->rows));
    memcpy(metadata->spin_trim_dps, config.spin_wheel_trim_dps, sizeof(metadata->spin_trim_dps));
    return 1u;
}

static void ServiceCapture(ChassisMotionState *state)
{
    static WheelIMUCaptureConfig metadata;
    uint8_t arm_allowed = wheel_imu_capture_request && !WheelIMUCaptureActive() &&
                          PrepareCapture(state, &metadata);
    WheelIMUCaptureService(arm_allowed ? &metadata : NULL,
                           g_chassis_log_config.capture_duration_us,
                           g_chassis_log_config.capture_rtt_channel);
}
#endif

static uint8_t MotionWritePacket(const char *text, unsigned capacity, unsigned used)
{
    if (used < capacity && SEGGER_RTT_Write(g_chassis_log_config.text_rtt_channel, text, used) == used)
        return 1u;
    motion_transport_drop++;
    return 0u;
}

static void MotionAuxiliaryLog(const ChassisMotionState *state, const ChassisMotionObservationState *observation,
                               char *text, unsigned capacity)
{
    static uint8_t sent;
    static uint32_t last_time_us;
    static ChassisMotionConfig config;
    static RobotCMDYawDebug yaw;
    static INS_CalibrationDebug calibration;
    static GimbalYawFeedback yaw_control;
    if (sent && (uint32_t)(state->time_us - last_time_us) < 1000000u) return;

#ifndef CHASSIS_BOARD
    RobotCMDGetYawDebug(&yaw);
    GimbalGetYawFeedback(&yaw_control);
#endif
    INS_GetCalibrationDebug(&calibration);
    INS_SampleDebug sample;
    INS_GetSampleDebug(&sample);
    const uint32_t read_time_us = (uint32_t)DWT_GetTimeline_us();
    ChassisMotionGetConfig(&config);

    unsigned used = 0u;
    /* DETAIL 跟随控制快照。YAW/HEALTH 是日志任务读取的快照，各自保留源时间戳。 */
    MotionDiagAppend(text, capacity, &used, "[DETAIL] seq=%lu", (unsigned long)state->seq);
    MotionDiagVector(text,capacity,&used,"q_1e6",state->imu_q,4,1,0,1000000);
    MotionDiagVector(text,capacity,&used,"gravity_mmps2",observation->gravity_b,3,1,0,1000);
    MotionDiagVector(text,capacity,&used,"innov_x1000",state->innovation,4,1,0,1000);
    MotionDiagVector(text,capacity,&used,"r_x1e6",observation->r_diag,4,1,0,1000000);
    MotionDiagAppend(text, capacity, &used,
        "\r\n[YAW] seq=%lu diag=3 cmd_t_us=%lu src=%u online=%u gmode=%u rc=%d used=%d ref_mdeg=%ld fb_mdeg=%ld enc_mdeg=%ld gz_mdps=%ld temp_mC=%ld cal_temp_mC=%ld cal=%u bias_z_urps=%ld trim_z_urps=%ld trim=%u trim_updates=%lu enc_t_us=%lu ctrl_t_us=%lu spd_p_raw=%ld spd_i_raw=%ld out_raw=%ld sent_raw=%ld i_limit_raw=%ld\r\n",
        (unsigned long)state->seq, (unsigned long)yaw.time_us,
        (unsigned)yaw.source, (unsigned)yaw.rc_online, (unsigned)yaw.gimbal_mode,
        (int)yaw.rc_yaw_raw, (int)yaw.rc_yaw_used,
        (long)MotionLogScaled(yaw.yaw_ref, 1000.0f),
        (long)MotionLogScaled(yaw.yaw_feedback, 1000.0f),
        (long)MotionLogScaled(calibration.yaw_encoder_deg, 1000.0f),
        (long)MotionLogScaled(yaw.gyro_z, 1000.0f),
        (long)MotionLogScaled(calibration.temperature, 1000.0f),
        (long)MotionLogScaled(calibration.calibration_temperature, 1000.0f),
        (unsigned)calibration.status,
        (long)MotionLogScaled(calibration.gyro_offset_z, 1000000.0f),
        (long)MotionLogScaled(calibration.yaw_trim_z, 1000000.0f),
        (unsigned)calibration.trim_status, (unsigned long)calibration.trim_updates,
        (unsigned long)calibration.encoder_time_us,
        /* GM6020 的 raw 驱动指令不能解读为实际电流或转矩。 */
        (unsigned long)yaw_control.control_time_us,
        (long)MotionLogScaled(yaw_control.speed_p_raw, 1.0f),
        (long)MotionLogScaled(yaw_control.speed_i_raw, 1.0f),
        (long)MotionLogScaled(yaw_control.output_raw, 1.0f),
        (long)MotionLogScaled(yaw_control.sent_raw, 1.0f),
        (long)g_gimbal_config.yaw.controller_param_init_config.speed_PID.IntegralLimit);
    MotionDiagAppend(text, capacity, &used,
        "[HEALTH] seq=%lu t_us=%lu read_t_us=%lu attempt_seq=%lu accepted_seq=%lu status=%u startup=%u timeout=%lu error=%lu busy=%lu not_ready=%lu gaps=%lu\r\n",
        (unsigned long)state->seq, (unsigned long)read_time_us, (unsigned long)sample.read_time_us,
        (unsigned long)sample.attempt_seq, (unsigned long)sample.accepted_seq,
        (unsigned)sample.status, (unsigned)sample.startup_status,
        (unsigned long)sample.timeout_count, (unsigned long)sample.error_count,
        (unsigned long)sample.busy_count, (unsigned long)sample.not_ready_count,
        (unsigned long)sample.gap_count);
    /* FreeRTOS 1 tick = 1 ms。用运行时配置保留平台层定制的车轮参数。 */
    MotionDiagAppend(text, capacity, &used,
        "[CONFIG] seq=%lu fw=%08lx frame=%u period_us=%lu decimation=%u direct=%u rotation_priority=%u imu_on_gimbal=%u imu_follows_pitch=%u drive=%u",
        (unsigned long)state->seq, (unsigned long)config.version,
        (unsigned)g_chassis_log_config.frame_revision, (unsigned long)g_task_config.robot.period_ticks*1000u,
        (unsigned)config.log_decimation, (unsigned)config.direct_translation, (unsigned)config.rotation_priority,
        (unsigned)config.imu_on_gimbal, (unsigned)config.imu_follows_pitch, (unsigned)config.wheels.type);
    MotionDiagVector(text,capacity,&used,"radius_um",config.wheels.radius_m,4,1,0,1000000);
    MotionDiagVector(text,capacity,&used,"gear_x1000",config.wheels.gear_ratio,4,1,0,1000);
    MotionDiagVector(text,capacity,&used,"motor_sign",config.wheels.motor_sign,4,1,0,1);
    float rows[12];
    memcpy(rows, config.wheels.rows, sizeof(rows));
    MotionDiagVector(text,capacity,&used,"rows_x1e6",rows,12,1,0,1000000);
    MotionDiagVector(text,capacity,&used,"trim_dps",config.spin_wheel_trim_dps,4,1,0,1);
    const float spin_ref = MotionSpinReference(config.max_omega);
    MotionDiagAppend(text, capacity, &used, " spin_ref_mradps=%ld", (long)MotionLogScaled(spin_ref,1000));
    MotionDiagVector(text,capacity,&used,"max_wheel_dps",config.wheels.max_motor_dps,4,1,0,1);
    MotionDiagAppend(text, capacity, &used,
        " current_limit_raw=%ld algebra_y_sign=%d center_y_sign=%d\r\n",
        (long)MotionLogScaled(config.current_limit,1), (int)config.algebra_y_sign, (int)config.center_y_sign);
    /* RTT 满时让下一控制快照重试，避免忙等和漏掉首条配置。 */
    if (MotionWritePacket(text, capacity, used)) {
        last_time_us = state->time_us;
        sent = 1u;
    }
}

void ChassisMotionLogTask(void)
{
    /* 缓冲区和状态快照放在静态区，不挤占日志任务的栈。只在低优先级任务格式化。 */
    static ChassisMotionState state;
    static ChassisMotionDirectionState direction;
    static ChassisMotionObservationState observation;
    static char text[3072];
    if (!motion_log_ready) return;
#if WHEEL_IMU_CAPTURE
    ServiceCapture(&state);
#endif
    while (ChassisMotionPopDiagnosticLog(&state, &direction, &observation))
    {
        /* 六行核心数据来自同一控制快照，三元组为xyz，四轮LF/RF/LB/RB。
         * mmps=mm/s，mmps2=mm/s²，mrad=毫弧度；电流保持 DJI 原始单位。
         * 整组只调用一次非阻塞 RTT_Write，空间不足则整组丢弃并累计 tdrop。 */
        int length = snprintf(text, sizeof(text),
            "[STATUS] seq=%lu t_us=%lu fw=%08lx frame=%u mode=%u valid=%u exit=%u accel=%u wheel_mask=%u obs=%u dt_us=%ld imu_age_us=%lu cmd_age_us=%lu qdrop=%lu tdrop=%lu imu_seq=%lu chassis_mode=%u control_mode=%u output_enabled=%u\r\n"
            "[WHEEL] seq=%lu dps=%ld,%ld,%ld,%ld req_dps=%ld,%ld,%ld,%ld req_i=%ld,%ld,%ld,%ld send_i=%ld,%ld,%ld,%ld fb_i=%ld,%ld,%ld,%ld vn_mmps=%ld,%ld omega_mradps=%ld omega_ref_mradps=%ld raw_dps=%ld,%ld,%ld,%ld age_us=%lu,%lu,%lu,%lu wheel_seq=%lu,%lu,%lu,%lu scale_x1000=%ld\r\n"
            "[IMU] seq=%lu corr=%u theta_mrad=%ld psi_bn_mrad=%ld psi_gn_mrad=%ld gyro_mradps=%ld,%ld,%ld raw_a_mmps2=%ld,%ld,%ld nav_a_mmps2=%ld,%ld,%ld center_a_mmps2=%ld,%ld\r\n"
            "[KF] seq=%lu v_n_mmps=%ld,%ld a_n_mmps2=%ld,%ld user_g_mmps=%ld,%ld ref_g_mmps=%ld,%ld hat_g_mmps=%ld,%ld err_g_mmps=%ld,%ld dv_g_mmps=%ld,%ld cmd_g_mmps=%ld,%ld kp_x1000=%ld flags=%u sat=%u p_x1e6=%ld,%ld,%ld,%ld\r\n"
            "[DIR] seq=%lu theta_out_mrad=%ld rel_omega_mradps=%ld lead_mrad=%ld rate_valid=%u\r\n",
            (unsigned long)state.seq, (unsigned long)state.time_us,
            (unsigned long)state.version,
            (unsigned)g_chassis_log_config.frame_revision,
            (unsigned)state.mode, (unsigned)state.estimate_valid, (unsigned)state.exit_reason,
            (unsigned)state.accel_used, (unsigned)state.wheel_valid_mask, (unsigned)state.observation_mask,
            (long)MotionLogScaled(state.dt_s, 1000000.0f),
            (unsigned long)state.imu_age_us, (unsigned long)state.cmd_age_us,
            (unsigned long)state.log_drop, (unsigned long)motion_transport_drop,
            (unsigned long)state.imu_seq, (unsigned)observation.chassis_mode,
            (unsigned)observation.control_mode, (unsigned)observation.output_enabled,
            (unsigned long)state.seq,
            (long)MotionLogScaled(state.motor_dps[0], 1.0f), (long)MotionLogScaled(state.motor_dps[1], 1.0f),
            (long)MotionLogScaled(state.motor_dps[2], 1.0f), (long)MotionLogScaled(state.motor_dps[3], 1.0f),
            (long)MotionLogScaled(state.wheel_eff_dps[0], 1.0f), (long)MotionLogScaled(state.wheel_eff_dps[1], 1.0f),
            (long)MotionLogScaled(state.wheel_eff_dps[2], 1.0f), (long)MotionLogScaled(state.wheel_eff_dps[3], 1.0f),
            (long)MotionLogScaled(state.current_request[0], 1.0f), (long)MotionLogScaled(state.current_request[1], 1.0f),
            (long)MotionLogScaled(state.current_request[2], 1.0f), (long)MotionLogScaled(state.current_request[3], 1.0f),
            (long)MotionLogScaled(state.current_send[0], 1.0f), (long)MotionLogScaled(state.current_send[1], 1.0f),
            (long)MotionLogScaled(state.current_send[2], 1.0f), (long)MotionLogScaled(state.current_send[3], 1.0f),
            (long)MotionLogScaled(state.current_fb[0], 1.0f), (long)MotionLogScaled(state.current_fb[1], 1.0f),
            (long)MotionLogScaled(state.current_fb[2], 1.0f), (long)MotionLogScaled(state.current_fb[3], 1.0f),
            (long)MotionLogScaled(state.wheel_n[0], 1000.0f), (long)MotionLogScaled(state.wheel_n[1], 1000.0f),
            (long)MotionLogScaled(state.omega_b, 1000.0f),
            (long)MotionLogScaled(state.omega_ref, 1000.0f),
            (long)MotionLogScaled(state.raw_motor_dps[0], 1.0f),
            (long)MotionLogScaled(state.raw_motor_dps[1], 1.0f),
            (long)MotionLogScaled(state.raw_motor_dps[2], 1.0f),
            (long)MotionLogScaled(state.raw_motor_dps[3], 1.0f),
            (unsigned long)state.wheel_age_us[0], (unsigned long)state.wheel_age_us[1],
            (unsigned long)state.wheel_age_us[2], (unsigned long)state.wheel_age_us[3],
            (unsigned long)state.wheel_seq[0], (unsigned long)state.wheel_seq[1],
            (unsigned long)state.wheel_seq[2], (unsigned long)state.wheel_seq[3],
            (long)MotionLogScaled(state.wheel_scale, 1000.0f),
            (unsigned long)state.seq, (unsigned)state.accel_center_valid,
            (long)MotionLogScaled(state.theta_bg, 1000.0f),
            (long)MotionLogScaled(state.psi_bn, 1000.0f), (long)MotionLogScaled(state.psi_gn, 1000.0f),
            (long)MotionLogScaled(state.imu_gyro_rad[0], 1000.0f),
            (long)MotionLogScaled(state.imu_gyro_rad[1], 1000.0f),
            (long)MotionLogScaled(state.imu_gyro_rad[2], 1000.0f),
            (long)MotionLogScaled(state.imu_accel_raw[0], 1000.0f),
            (long)MotionLogScaled(state.imu_accel_raw[1], 1000.0f),
            (long)MotionLogScaled(state.imu_accel_raw[2], 1000.0f),
            (long)MotionLogScaled(state.imu_motion_n[0], 1000.0f),
            (long)MotionLogScaled(state.imu_motion_n[1], 1000.0f),
            (long)MotionLogScaled(state.imu_motion_n[2], 1000.0f),
            (long)MotionLogScaled(state.accel_center_n[0], 1000.0f),
            (long)MotionLogScaled(state.accel_center_n[1], 1000.0f),
            (unsigned long)state.seq,
            (long)MotionLogScaled(state.x[0], 1000.0f), (long)MotionLogScaled(state.x[2], 1000.0f),
            (long)MotionLogScaled(state.x[1], 1000.0f), (long)MotionLogScaled(state.x[3], 1000.0f),
            (long)MotionLogScaled(state.user_g[0], 1000.0f), (long)MotionLogScaled(state.user_g[1], 1000.0f),
            (long)MotionLogScaled(state.ref_g[0], 1000.0f), (long)MotionLogScaled(state.ref_g[1], 1000.0f),
            (long)MotionLogScaled(state.hat_g[0], 1000.0f), (long)MotionLogScaled(state.hat_g[1], 1000.0f),
            (long)MotionLogScaled(state.error_g[0], 1000.0f), (long)MotionLogScaled(state.error_g[1], 1000.0f),
            (long)MotionLogScaled(state.dv_g[0], 1000.0f), (long)MotionLogScaled(state.dv_g[1], 1000.0f),
            (long)MotionLogScaled(state.cmd_g[0], 1000.0f), (long)MotionLogScaled(state.cmd_g[1], 1000.0f),
            (long)MotionLogScaled(state.kp, 1000.0f),
            (unsigned)state.limit_flags, (unsigned)state.current_saturated_mask,
            (long)MotionLogScaled(state.p_diag[0], 1000000.0f), (long)MotionLogScaled(state.p_diag[1], 1000000.0f),
            (long)MotionLogScaled(state.p_diag[2], 1000000.0f), (long)MotionLogScaled(state.p_diag[3], 1000000.0f),
            (unsigned long)state.seq,
            (long)MotionLogScaled(direction.theta_output_bg, 1000.0f),
            (long)MotionLogScaled(direction.theta_rate_bg, 1000.0f),
            (long)MotionLogScaled(direction.lead_rad, 1000.0f),
            (unsigned)direction.rate_valid);
        unsigned used=length>0 ? (unsigned)length : sizeof(text);
        MotionDiagAppend(text,sizeof(text),&used,
            "[OBS] seq=%lu imu_t_us=%lu clock_gen=%lu recovering=%u good_ms=%lu tilt_mrad=%ld,%ld chi_x1000=%ld att_flags=%u dv_res_mmps=%ld,%ld window_ms=%ld gate=%u rejects=%lu dv_now_mmps=%ld,%ld v_res_mmps=%ld,%ld reject_reason=%u control_ready=%u control_inhibit=%u control_good_ms=%ld control_blend_x1000=%ld control_v_err_mmps=%ld control_a_res_mmps2=%ld\r\n",
            (unsigned long)state.seq,(unsigned long)observation.imu_time_us,
            (unsigned long)observation.clock_generation,(unsigned)observation.imu_recovering,
            (unsigned long)(observation.recovery_good_us/1000u),
            (long)MotionLogScaled(observation.pitch_rad,1000),(long)MotionLogScaled(observation.roll_rad,1000),
            (long)MotionLogScaled(observation.attitude_chi_ratio,1000),(unsigned)observation.attitude_flags,
            (long)MotionLogScaled(observation.delta_v_residual[0],1000),
            (long)MotionLogScaled(observation.delta_v_residual[1],1000),
            (long)MotionLogScaled(observation.window_s,1000),(unsigned)observation.gate,
            (unsigned long)observation.rejected_windows,
            (long)MotionLogScaled(observation.delta_v_partial[0],1000),
            (long)MotionLogScaled(observation.delta_v_partial[1],1000),
            (long)MotionLogScaled(observation.velocity_residual[0],1000),
            (long)MotionLogScaled(observation.velocity_residual[1],1000),
            (unsigned)observation.reject_reason,
            (unsigned)observation.control_ready,(unsigned)observation.control_inhibit,
            (long)MotionLogScaled(observation.control_good_s,1000),
            (long)MotionLogScaled(observation.control_blend,1000),
            (long)MotionLogScaled(observation.control_velocity_error,1000),
            (long)MotionLogScaled(observation.control_accel_residual,1000));
        if (MotionWritePacket(text, sizeof(text), used))
            MotionAuxiliaryLog(&state, &observation, text, sizeof(text));
    }
    MotionDiagnosticLog();
}
