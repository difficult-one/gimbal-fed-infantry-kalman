/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_msg.h"
#include "config.h"
#include "chassis_force.h"
#include "chassis_log.h"
#include "chassis_setup.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include <string.h>
#include <math.h>
#include "power_control.h"
#include "super_cap.h"
#include "message_center.h"
#include "chassis_ui.h"

#include "math_units.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "math_utils.h"
#include "chassis_motion.h"
#include "wheel_imu_capture.h"

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
static uint32_t chassis_cmd_can_seq;
#endif // CHASSIS_BOARD
static attitude_t *Chassis_IMU_data;
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
static Publisher_t *chassis_force_pub;              // 独立力反馈话题
static Subscriber_t *chassis_force_sensor_sub;      // 独立轮端力传感器输入话题
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static PIDInstance buffer_PID;             // 用于底盘的缓冲能量PID
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float vt_lf, vt_rf, vt_lb, vt_rb;                  // 底盘速度解算后的临时输出,待进行限幅
static uint8_t chassis_cmd_stale_cycles;
static uint32_t chassis_cmd_time_us;
static uint8_t chassis_have_cmd;
_Static_assert((int)CHASSIS_ROTATE == (int)MC_ROTATE_CONTROL_KEY, "spin control key changed");

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

static void SetChassisOuterLoop(Closeloop_Type_e loop)
{
    static Closeloop_Type_e last_loop = SPEED_LOOP;
    if (last_loop == loop)
        return;

    DJIMotorOuterLoop(motor_lf, loop);
    DJIMotorOuterLoop(motor_rf, loop);
    DJIMotorOuterLoop(motor_lb, loop);
    DJIMotorOuterLoop(motor_rb, loop);
    /* M3508/C620 内部已经有电流环。力控模式直接发送 DJI 原始电流命令，
       避免在驱动器电流环外再串一个未标定的软件 current_PID。 */
    if (loop == CURRENT_LOOP)
    {
        motor_lf->motor_settings.close_loop_type &= ~CURRENT_LOOP;
        motor_rf->motor_settings.close_loop_type &= ~CURRENT_LOOP;
        motor_lb->motor_settings.close_loop_type &= ~CURRENT_LOOP;
        motor_rb->motor_settings.close_loop_type &= ~CURRENT_LOOP;
    }
    else
    {
        motor_lf->motor_settings.close_loop_type |= CURRENT_LOOP;
        motor_rf->motor_settings.close_loop_type |= CURRENT_LOOP;
        motor_lb->motor_settings.close_loop_type |= CURRENT_LOOP;
        motor_rb->motor_settings.close_loop_type |= CURRENT_LOOP;
    }
    ResetPIDState(&motor_lf->motor_controller.current_PID);
    ResetPIDState(&motor_rf->motor_controller.current_PID);
    ResetPIDState(&motor_lb->motor_controller.current_PID);
    ResetPIDState(&motor_rb->motor_controller.current_PID);
    ResetPIDState(&motor_lf->motor_controller.speed_PID);
    ResetPIDState(&motor_rf->motor_controller.speed_PID);
    ResetPIDState(&motor_lb->motor_controller.speed_PID);
    ResetPIDState(&motor_rb->motor_controller.speed_PID);
    last_loop = loop;
}

static uint8_t ChassisMotorsOnline(void)
{
    return DaemonIsOnline(motor_lf->daemon) &&
           DaemonIsOnline(motor_rf->daemon) &&
           DaemonIsOnline(motor_lb->daemon) &&
           DaemonIsOnline(motor_rb->daemon);
}

static uint8_t ChassisControlProtocolValid(void)
{
#ifdef CHASSIS_BOARD
    if (!CANCommIsOnline(chasiss_can_comm))
        return 0u;
#endif
    if (chassis_cmd_stale_cycles > g_chassis_config.command_timeout_cycles || !chassis_have_cmd ||
        (uint32_t)((uint32_t)DWT_GetTimeline_us() - chassis_cmd_time_us) > g_chassis_config.command_timeout_us)
        return 0u;
    /* 遥控摇杆正常范围 ±660，现有倍率 30 对应最大旧命令 ±19800。
     * 允许完整摇杆行程，避免原 ±10000 检查在大行程时误判为无效命令；
     * 实际速度仍由 车型配置的平移速度上限 和轮速/电流限幅约束。 */
    if (chassis_cmd_recv.control_mode == CHASSIS_CONTROL_VELOCITY)
        return isfinite(chassis_cmd_recv.vx) && fabsf(chassis_cmd_recv.vx) <= g_chassis_config.velocity_command_limit &&
               isfinite(chassis_cmd_recv.vy) && fabsf(chassis_cmd_recv.vy) <= g_chassis_config.velocity_command_limit &&
               isfinite(chassis_cmd_recv.wz) && isfinite(chassis_cmd_recv.offset_angle) &&
               fabsf(chassis_cmd_recv.offset_angle) <= g_chassis_config.offset_angle_limit_deg;
#ifndef CHASSIS_DRIVE_OMNI_X
    /* Legacy force allocator is specific to this vehicle's omni geometry. */
    return 0u;
#endif
    if (chassis_cmd_recv.control_mode < CHASSIS_CONTROL_VELOCITY ||
        chassis_cmd_recv.control_mode > CHASSIS_CONTROL_ADMITTANCE)
        return 0u;
    return chassis_cmd_recv.protocol_version == CHASSIS_CONTROL_PROTOCOL_VERSION &&
           ChassisForceCommandValid(&chassis_cmd_recv);
}

void ChassisInit()
{
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = BSPCANHandle(g_link_config.chassis_motor_can),
        .controller_param_init_config = {
            .speed_PID = g_chassis_config.speed_pid,
            .current_PID = g_chassis_config.current_pid,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    DJIMotorInstance **const motors[4] = {&motor_lf, &motor_rf, &motor_lb, &motor_rb};
    for (unsigned i = 0; i < 4; ++i) {
        chassis_motor_config.can_init_config.tx_id = g_chassis_config.motor_tx_id[i];
        chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
            g_chassis_config.motor_reversed[i] ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
        *motors[i] = DJIMotorInit(&chassis_motor_config);
    }
#if defined(WHEEL_IMU_CAPTURE) && WHEEL_IMU_CAPTURE
    WheelIMUCaptureRegisterMotor(WIMU_LF, motor_lf);
    WheelIMUCaptureRegisterMotor(WIMU_RF, motor_rf);
    WheelIMUCaptureRegisterMotor(WIMU_LB, motor_lb);
    WheelIMUCaptureRegisterMotor(WIMU_RB, motor_rb);
#endif

    referee_data = UITaskInit(BSPUSARTHandle(g_link_config.referee_uart), &ui_data); // 裁判系统初始化,会同时初始化UI

/* Buffer环暂未测试，逻辑是计算期望buffer与实际buffer的差值，转换为冗余的功率，todo：输入给功率控制部分，待完善 */
    PID_Init_Config_s Buffer_pid_conf = g_chassis_config.power_buffer_pid;
    PIDInit(&buffer_PID, &Buffer_pid_conf); // 缓冲能量PID初始化

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = BSPCANHandle(g_link_config.super_cap_can),
            .tx_id = g_link_config.super_cap_tx_id,
            .rx_id = g_link_config.super_cap_rx_id,
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    /* A板也需要在底盘应用中初始化 INS；INS_Task() 只负责更新已初始化的实例。 */
    Chassis_IMU_data = INS_Init(&g_imu_config);
    if (g_chassis_imu_config.on_gimbal && g_chassis_imu_config.follows_pitch)
        INS_SetLeverArm(g_chassis_imu_config.gimbal_lever_m);
    else if (!g_chassis_imu_config.on_gimbal)
        INS_SetLeverArm(g_chassis_imu_config.chassis_lever_m);
    ChassisMotionInit();
    ChassisLogInit();

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = BSPCANHandle(g_link_config.board_can),
            .tx_id = g_link_config.board_tx_id,
            .rx_id = g_link_config.board_rx_id,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
        .daemon_count = g_link_config.board_daemon_count,
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    chassis_force_pub = PubRegister(CHASSIS_FORCE_FEED_TOPIC, sizeof(Chassis_Force_Feed_s));
    chassis_force_sensor_sub = SubRegister(CHASSIS_FORCE_SENSOR_TOPIC, sizeof(Chassis_Force_Sensor_s));
#endif // ONE_BOARD
    chassis_cmd_stale_cycles = g_chassis_config.command_timeout_cycles + 1u;
    ChassisForceInit();
}

#define COS45 0.70710678f
/**
 * @brief 四轮全向轮运动学解算 (X 型排列, 轮子互成 90°)
 *
 * 轮子布局:  RF=45°, LF=135°, LB=225°, RB=315°
 * 公式: v_i = vx*cos(θ) + vy*sin(θ) - wz*L
 */
static void OmniCalculate(float chassis_vx, float chassis_vy)
{
    vt_rf =  chassis_vx * COS45 + chassis_vy * COS45 + chassis_cmd_recv.wz * g_chassis_config.legacy_omni_center;
    vt_lf = -chassis_vx * COS45 + chassis_vy * COS45 + chassis_cmd_recv.wz * g_chassis_config.legacy_omni_center;
    vt_lb = -chassis_vx * COS45 - chassis_vy * COS45 + chassis_cmd_recv.wz * g_chassis_config.legacy_omni_center;
    vt_rb =  chassis_vx * COS45 - chassis_vy * COS45 + chassis_cmd_recv.wz * g_chassis_config.legacy_omni_center;
}

__attribute__((weak)) uint8_t ChassisReadSteeringFeedback(float angle[4], uint32_t time[4], uint8_t *mask)
{
    (void)angle; (void)time; *mask=0u;
    return 0u;
}

__attribute__((weak)) uint8_t ChassisApplySteeringTargets(const float angle[4], uint8_t enabled)
{
    (void)angle; (void)enabled;
    return 0u;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
#ifdef CHASSIS_DRIVE_SWERVE
    ChassisWheelCommand command;
    ChassisMotionGetWheelCommand(&command);
    if (!ChassisApplySteeringTargets(command.steer_rad, 1u))
        vt_lf = vt_rf = vt_lb = vt_rb = 0.0f;
#endif
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    static ChassisMotionInput input;
    INS_MotionSnapshot imu;
    DJIMotorInstance *motors[4] = {motor_lf, motor_rf, motor_lb, motor_rb};
    memset(&input, 0, sizeof(input));
    /* 一次短临界区取得四轮+IMU 快照；矩阵、三角运算均在恢复中断后执行。 */
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    INS_GetMotionSnapshot(&imu);
    for (uint8_t i = 0; i < 4; ++i)
    {
        DJIMotorSnapshot wheel;
        DJIMotorGetSnapshot(motors[i], &wheel);
        input.wheel_seq[i] = wheel.seq;
        input.wheel_time_us[i] = wheel.time_us;
        input.motor_dps[i] = wheel.speed_dps;
        input.raw_motor_dps[i] = wheel.raw_speed_dps;
        input.current_fb[i] = wheel.current_fb;
        input.current_request[i] = wheel.current_request;
        input.current_send[i] = wheel.current_send;
        if (wheel.online) input.wheel_online_mask |= 1u << i;
    }
#ifdef CHASSIS_DRIVE_SWERVE
    if (!ChassisReadSteeringFeedback(input.steer_rad, input.steer_time_us, &input.steer_online_mask))
        input.steer_online_mask = 0u;
#endif
    input.time_us = (uint32_t)DWT_GetTimeline_us();
    DWT_ClockDebug clock;
    DWT_GetClockDebug(&clock);
    __set_PRIMASK(key);
    input.cmd_age_us = chassis_have_cmd ? (uint32_t)(input.time_us - chassis_cmd_time_us) : UINT32_MAX;
    input.imu_seq = imu.seq;
    input.imu_time_us = imu.time_us;
    memcpy(input.imu_q, imu.q, sizeof(input.imu_q));
    memcpy(input.imu_accel_raw, imu.accel_raw, sizeof(input.imu_accel_raw));
    memcpy(input.imu_motion_b, imu.motion_b, sizeof(input.imu_motion_b));
    memcpy(input.imu_motion_n, imu.motion_n, sizeof(input.imu_motion_n));
    memcpy(input.imu_gyro_rad, imu.gyro_rad, sizeof(input.imu_gyro_rad));
    memcpy(input.imu_center_n, imu.center_n, sizeof(input.imu_center_n));
    input.imu_center_valid = imu.center_valid;
    input.clock_generation=clock.generation;
    input.imu_recovering=!imu.motion_ready || imu.clock_generation!=clock.generation;
    input.imu_recovery_good_us=imu.recovery_good_us;
    memcpy(input.imu_gravity_b,imu.gravity_b,sizeof(input.imu_gravity_b));
    input.imu_pitch_rad=imu.pitch_rad; input.imu_roll_rad=imu.roll_rad;
    input.attitude_chi_ratio=imu.attitude_chi_ratio; input.attitude_flags=imu.attitude_flags;
    /* 仅“不跟随”速控零命令且四轮真实反馈静止时允许辅助标定。
     * 跟随/自旋/力控、过期命令、任一轮失联或转动立即撤销许可。
     * 使用原始速度，防止轮速低通尚未响应时误判静止。 */
    uint32_t still_reason=0;
    if (!chassis_have_cmd || input.cmd_age_us>g_chassis_motion_config.sample_max_us) still_reason |= IMU_AID_BIT(IMU_AID_CHASSIS_CMD_TIME);
    if (chassis_cmd_recv.chassis_mode!=CHASSIS_NO_FOLLOW ||
        chassis_cmd_recv.control_mode!=CHASSIS_CONTROL_VELOCITY) still_reason |= IMU_AID_BIT(IMU_AID_CHASSIS_MODE);
    if (!(fabsf(chassis_cmd_recv.vx)<g_chassis_imu_config.still_command_max && fabsf(chassis_cmd_recv.vy)<g_chassis_imu_config.still_command_max &&
          fabsf(chassis_cmd_recv.wz)<g_chassis_imu_config.still_command_max)) still_reason |= IMU_AID_BIT(IMU_AID_CHASSIS_CMD_MOTION);
    if (input.wheel_online_mask!=15u) still_reason |= IMU_AID_BIT(IMU_AID_WHEEL_OFFLINE);
    for (uint8_t i=0; i<4; ++i) {
        if ((uint32_t)(input.time_us-input.wheel_time_us[i])>g_chassis_motion_config.sample_max_us) still_reason |= IMU_AID_BIT(IMU_AID_WHEEL_TIME);
        if (!isfinite(input.raw_motor_dps[i]) || !(fabsf(input.raw_motor_dps[i])<g_chassis_imu_config.still_raw_dps)) still_reason |= IMU_AID_BIT(IMU_AID_WHEEL_RAW);
        if (!isfinite(input.motor_dps[i]) || !(fabsf(input.motor_dps[i])<g_chassis_imu_config.still_filtered_dps)) still_reason |= IMU_AID_BIT(IMU_AID_WHEEL_FILTERED);
    }
    INS_SetChassisStill(input.time_us, still_reason==0u, still_reason);
    /* 固定 N=(INS_n.x,-INS_n.y,-INS_n.z)，旧命令/J 航向由实车平移核对。
     * theta_BG 只含真实相对角；经验方向 trim 仅旋转用户意图，不污染姿态。 */
    input.theta_bg = g_chassis_imu_config.yaw_encoder_sign * chassis_cmd_recv.offset_angle * DEGREE_2_RAD;
    input.psi_imu_n = ChassisMotionHeading(imu.xn) + g_chassis_imu_config.heading_offset_rad;
    input.accel_n[0] = imu.motion_n[0];
    input.accel_n[1] = -imu.motion_n[1];
    /* gyro 投影到 N 竖轴，避免 IMU 随 pitch 倾斜时将传感器 z 当作 yaw。 */
    float q0 = imu.q[0], q1 = imu.q[1], q2 = imu.q[2], q3 = imu.q[3];
    input.omega_imu = -((2.0f * (q1 * q3 - q0 * q2)) * imu.gyro_rad[0] +
                        (2.0f * (q2 * q3 + q0 * q1)) * imu.gyro_rad[1] +
                        (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * imu.gyro_rad[2]);
    input.pose_valid = imu.valid && hypotf(imu.xn[0], imu.xn[1]) > g_chassis_imu_config.min_horizontal_heading;
    input.accel_valid = imu.valid;
    ChassisMotionUpdate(&input);
}

static void RunVelocityControl(void)
{
    float legacy_wz = 0.0f, ref[4], user_g[2];
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: break;
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        legacy_wz = float_constrain(g_chassis_config.follow_gain * theta_format(chassis_cmd_recv.offset_angle),
                                    -g_chassis_config.follow_limit, g_chassis_config.follow_limit);
        break;
    case CHASSIS_ROTATE: legacy_wz = g_chassis_config.legacy_spin_command; break;
    default: break;
    }
    /* 旧指令只在这里转 SI；跟随/小陀螺策略沿用原行为，不累改接收命令。 */
    float user_untrimmed[2] = {chassis_cmd_recv.vx * g_chassis_config.mps_per_motor_dps,
                              chassis_cmd_recv.vy * g_chassis_config.mps_per_motor_dps};
    ChassisMotionRotate(g_chassis_config.command_trim_deg * DEGREE_2_RAD, user_untrimmed, user_g);
#ifndef CHASSIS_DRIVE_OMNI_X
    user_g[1] = -user_g[1]; /* Legacy public command +Y right -> portable +Y left. */
#endif
    ChassisMotionControl(user_g, legacy_wz * g_chassis_config.legacy_omega_scale, 1u,
                         (uint8_t)chassis_cmd_recv.chassis_mode, ref);
    vt_lf = ref[0]; vt_rf = ref[1]; vt_lb = ref[2]; vt_rb = ref[3];
    LimitChassisOutput();
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    uint8_t motors_online;
    uint8_t force_measurement_valid;
    float sensor_fx, sensor_fy, sensor_mz;
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    if (SubGetMessage(chassis_sub, &chassis_cmd_recv))
    {
        chassis_cmd_stale_cycles = 0u;
        chassis_cmd_time_us = (uint32_t)DWT_GetTimeline_us();
        chassis_have_cmd = 1u;
    }
    else if (chassis_cmd_stale_cycles < 0xffu)
        chassis_cmd_stale_cycles++;
#endif
#ifdef CHASSIS_BOARD
    if (CANCommGetSnapshot(chasiss_can_comm, &chassis_cmd_recv, sizeof(chassis_cmd_recv),
                           &chassis_cmd_can_seq, &chassis_cmd_time_us))
    {
        chassis_have_cmd = 1u;
        chassis_cmd_stale_cycles = 0u;
    }
    else if (chassis_cmd_stale_cycles < 0xffu) chassis_cmd_stale_cycles++;
#endif // CHASSIS_BOARD

    /* 每个底盘周期都消费一次传感器话题，保证切回力控时不会复用过期样本。 */
    const Chassis_Force_Sensor_s *sample = NULL;
#ifdef ONE_BOARD
    Chassis_Force_Sensor_s sensor_message;
    if (SubGetMessage(chassis_force_sensor_sub, &sensor_message))
        sample = &sensor_message;
#endif
    force_measurement_valid = ChassisForceReadMeasurement(sample, &sensor_fx, &sensor_fy, &sensor_mz);

    if (referee_data != NULL)
        SetPowerLimit(referee_data->GameRobotState.chassis_power_limit);//设置功率限制
    motors_online = ChassisMotorsOnline();
    uint8_t mode_valid = chassis_cmd_recv.chassis_mode >= CHASSIS_ZERO_FORCE &&
                         chassis_cmd_recv.chassis_mode <= CHASSIS_FOLLOW_GIMBAL_YAW;
    uint8_t output_allowed = mode_valid && chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE &&
                             motors_online && ChassisControlProtocolValid();
    if (!output_allowed)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        SetChassisOuterLoop(SPEED_LOOP);
        DJIMotorSetRef(motor_lf, 0.0f);
        DJIMotorSetRef(motor_rf, 0.0f);
        DJIMotorSetRef(motor_lb, 0.0f);
        DJIMotorSetRef(motor_rb, 0.0f);
        ResetPIDState(&motor_lf->motor_controller.current_PID);
        ResetPIDState(&motor_rf->motor_controller.current_PID);
        ResetPIDState(&motor_lb->motor_controller.current_PID);
        ResetPIDState(&motor_rb->motor_controller.current_PID);
        ResetPIDState(&motor_lf->motor_controller.speed_PID);
        ResetPIDState(&motor_rf->motor_controller.speed_PID);
        ResetPIDState(&motor_lb->motor_controller.speed_PID);
        ResetPIDState(&motor_rb->motor_controller.speed_PID);
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        ChassisForceResetControl();
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    /* 停止输出判定先完成；估计在补偿之前执行，急停下也能观察真实滑行。 */
    EstimateSpeed();
#ifdef CHASSIS_DRIVE_SWERVE
    ChassisMotionState motion;
    ChassisMotionGetState(&motion);
    if (!output_allowed || motion.exit_reason==MC_EXIT_STEERING || motion.exit_reason==MC_EXIT_DT ||
        motion.exit_reason==MC_EXIT_GEOMETRY) {
        float hold[4] = {0};
        (void)ChassisApplySteeringTargets(hold, 0u);
        DJIMotorStop(motor_lf); DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb); DJIMotorStop(motor_rb);
        output_allowed=0u;
    }
#endif
    uint8_t velocity_enabled = output_allowed && chassis_cmd_recv.control_mode == CHASSIS_CONTROL_VELOCITY;
    float velocity_limit = velocity_enabled ? g_chassis_motion_config.current_limit : 0.0f;
    motor_lf->output_limit_raw = motor_rf->output_limit_raw = velocity_limit;
    motor_lb->output_limit_raw = motor_rb->output_limit_raw = velocity_limit;
    if (!velocity_enabled)
    {
        const float zero[2] = {0.0f, 0.0f};
        float unused[4];
        ChassisMotionControl(zero, 0.0f, 0u, 0u, unused);
    }

    DJIMotorInstance *const motors[4] = {motor_lf, motor_rf, motor_lb, motor_rb};
    if (output_allowed)
    {
        if (chassis_cmd_recv.control_mode == CHASSIS_CONTROL_TORQUE)
        {
            /* 电流/估算力矩模式：直接使用底盘合力与合力矩命令。 */
            SetChassisOuterLoop(CURRENT_LOOP);
            float gyro_z = Chassis_IMU_data ? Chassis_IMU_data->Gyro[2] : NAN;
            const float *current = ChassisForceTorque(chassis_cmd_recv.fx_ref, chassis_cmd_recv.fy_ref,
                                                      chassis_cmd_recv.mz_ref, gyro_z, motors);
            for (unsigned i = 0; i < 4; ++i)
                DJIMotorSetRef(motors[i], current[i]);
        }
        else if (chassis_cmd_recv.control_mode == CHASSIS_CONTROL_FORCE_TRACK)
        {
            if (force_measurement_valid)
            {
                SetChassisOuterLoop(CURRENT_LOOP);
                float gyro_z = Chassis_IMU_data ? Chassis_IMU_data->Gyro[2] : NAN;
                const float *current = ChassisForceTrack(sensor_fx, sensor_fy, sensor_mz,
                                                          &chassis_cmd_recv, gyro_z, motors);
                for (unsigned i = 0; i < 4; ++i)
                    DJIMotorSetRef(motors[i], current[i]);
            }
            else
            {
                DJIMotorStop(motor_lf);
                DJIMotorStop(motor_rf);
                DJIMotorStop(motor_lb);
                DJIMotorStop(motor_rb);
                ChassisForceResetControl();
            }
        }
        else if (chassis_cmd_recv.control_mode == CHASSIS_CONTROL_ADMITTANCE)
        {
            if (force_measurement_valid)
            {
                float velocity[3];
                ChassisForceAdmittance(sensor_fx, sensor_fy, sensor_mz, &chassis_cmd_recv, velocity);
                SetChassisOuterLoop(SPEED_LOOP);
                float theta = (chassis_cmd_recv.offset_angle + g_chassis_config.command_trim_deg) * DEGREE_2_RAD;
                float cos_theta = arm_cos_f32(theta), sin_theta = arm_sin_f32(theta);
                float vx = velocity[0] * cos_theta - velocity[1] * sin_theta;
                float vy = velocity[0] * sin_theta + velocity[1] * cos_theta;
                chassis_cmd_recv.wz = velocity[2];
                OmniCalculate(vx, vy);
                LimitChassisOutput();
            }
            else
            {
                DJIMotorStop(motor_lf);
                DJIMotorStop(motor_rf);
                DJIMotorStop(motor_lb);
                DJIMotorStop(motor_rb);
                ChassisForceResetControl();
            }
        }
        else
        {
            SetChassisOuterLoop(SPEED_LOOP);

            RunVelocityControl();
        }
    }

#if WHEEL_IMU_CAPTURE
    ChassisMotionVisitRecord(WheelIMUCaptureRecordControl);
#endif
    ChassisMotionRecord();
    const Chassis_Force_Feed_s *force_feedback =
        ChassisForceUpdateFeedback(ChassisMotorsOnline(), &chassis_cmd_recv, motors);
#ifdef ONE_BOARD
    PubPushMessage(chassis_force_pub, (void *)force_feedback);
#else
    (void)force_feedback;
#endif

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}
