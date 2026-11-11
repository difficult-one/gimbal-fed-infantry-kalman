#include "gimbal.h"
#include "build_config.h"
#if defined(WHEEL_IMU_CAPTURE) && WHEEL_IMU_CAPTURE
#include "wheel_imu_capture.h"
#endif
#include "robot_msg.h"
#include "config.h"
#include "dji_motor.h"
#include "ins.h"
#include "message_center.h"
#include "math_units.h"
#include "SEGGER_RTT.h"
#include "bsp_log.h"
#include "robot_cmd.h"
#include <math.h>


static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息


void GimbalInit()
{
    gimbal_IMU_data = INS_Init(&g_imu_config);
#ifdef GIMBAL_BOARD
    /* 双板云台没有 ChassisInit，由这里安装云台 IMU 杆臂。 */
    if (g_chassis_imu_config.on_gimbal && g_chassis_imu_config.follows_pitch)
        INS_SetLeverArm(g_chassis_imu_config.gimbal_lever_m);
#endif
    if (g_log_config.gimbal_jscope_init)
        g_log_config.gimbal_jscope_init(&g_log_config.gimbal);

    Motor_Init_Config_s yaw_config = g_gimbal_config.yaw;
    yaw_config.can_init_config.can_handle = BSPCANHandle(g_gimbal_config.motor_can);
    yaw_config.controller_param_init_config.other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle;
    yaw_config.controller_param_init_config.other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2];

    Motor_Init_Config_s pitch_config = g_gimbal_config.pitch;
    pitch_config.can_init_config.can_handle = BSPCANHandle(g_gimbal_config.motor_can);
    pitch_config.controller_param_init_config.other_angle_feedback_ptr = &gimbal_IMU_data->Pitch;
    pitch_config.controller_param_init_config.other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[1];

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
#if defined(WHEEL_IMU_CAPTURE) && WHEEL_IMU_CAPTURE
    WheelIMUCaptureRegisterMotor(WIMU_YAW, yaw_motor);
#endif
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

void GimbalGetYawFeedback(GimbalYawFeedback *feedback)
{
    memset(feedback, 0, sizeof(*feedback));
    if (yaw_motor == NULL) return;
    /* 编码器、PID输出由不同中断/任务更新，复制时使用同一临界区。 */
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    DJIMotorSnapshot sample;
    DJIMotorGetSnapshot(yaw_motor, &sample);
    feedback->time_us = sample.time_us;
    feedback->angle_deg = sample.angle_deg;
    feedback->online = sample.online;
    feedback->speed_p_raw = yaw_motor->motor_controller.speed_PID.Pout;
    feedback->speed_i_raw = yaw_motor->motor_controller.speed_PID.Iout;
    feedback->output_raw = yaw_motor->current_request_raw;
    feedback->sent_raw = yaw_motor->current_send_raw;
    feedback->control_time_us = (uint32_t)DWT_GetTimeline_us();
    __set_PRIMASK(key);
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    /* 以实际 CAN 编码器辅助低频航向零偏，不使用公共反馈的整度截断值。
     * 不改电机目标/PID；完整精度也留给通道 0，用于观察小于 1° 的慢漂移。 */
    DJIMotorSnapshot yaw_sample, pitch_sample;
    INS_MotionSnapshot imu;
    RobotCMDYawDebug command;
    DJIMotorGetSnapshot(yaw_motor, &yaw_sample);
    DJIMotorGetSnapshot(pitch_motor, &pitch_sample);
    INS_GetMotionSnapshot(&imu);
    RobotCMDGetYawDebug(&command);
    IMUYawBiasInput aid = {0};
    aid.time_us = (uint32_t)DWT_GetTimeline_us();
    aid.encoder_deg = yaw_sample.angle_deg;
    aid.pitch_encoder_deg = pitch_sample.angle_deg;
    aid.yaw_deg = imu.yaw_deg;
    aid.yaw_ref_deg = gimbal_cmd_recv.yaw;
    aid.pitch_ref_deg = gimbal_cmd_recv.pitch;
    for (uint8_t i=0; i<4; ++i) aid.q[i] = imu.q[i];
    for (uint8_t i=0; i<3; ++i)
    {
        aid.gyro[i] = imu.gyro_rad[i];
        aid.accel_norm += imu.accel_raw[i]*imu.accel_raw[i];
    }
    aid.accel_norm = sqrtf(aid.accel_norm);
    /* 门槛与原版本相同，仅拆开拒绝原因，供累计诊断定位间歇重置。 */
    if (!imu.valid || (uint32_t)(aid.time_us-imu.time_us)>g_gimbal_config.yaw_aid_max_age_us) aid.inhibit |= IMU_AID_BIT(IMU_AID_IMU);
    if (!yaw_sample.online || (uint32_t)(aid.time_us-yaw_sample.time_us)>g_gimbal_config.yaw_aid_max_age_us) aid.inhibit |= IMU_AID_BIT(IMU_AID_YAW_FEEDBACK);
    if (!pitch_sample.online || (uint32_t)(aid.time_us-pitch_sample.time_us)>g_gimbal_config.yaw_aid_max_age_us) aid.inhibit |= IMU_AID_BIT(IMU_AID_PITCH_FEEDBACK);
    if (!command.rc_online) aid.inhibit |= IMU_AID_BIT(IMU_AID_REMOTE);
    if (command.source!=1u) aid.inhibit |= IMU_AID_BIT(IMU_AID_SOURCE);
    if (command.rc_yaw_used!=0) aid.inhibit |= IMU_AID_BIT(IMU_AID_YAW_INPUT);
    if (gimbal_cmd_recv.gimbal_mode!=GIMBAL_GYRO_MODE) aid.inhibit |= IMU_AID_BIT(IMU_AID_GIMBAL_MODE);
    if ((uint32_t)(aid.time_us-command.time_us)>g_gimbal_config.yaw_aid_max_age_us) aid.inhibit |= IMU_AID_BIT(IMU_AID_GIMBAL_CMD_TIME);
    aid.valid = aid.inhibit == 0u;
    INS_UpdateYawBias(&aid, yaw_sample.time_us);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }
    //GimbalDebugSend();
    // SEGGER_RTT_printf(BUFFER_INDEX,"roll:%d,pitch:%d,yaw:%d\n",(int16_t)gimbal_IMU_data->Roll,
    //                                               (int16_t)gimbal_IMU_data->Pitch,
    //                                               (int16_t)gimbal_IMU_data->Yaw);
    if (g_log_config.gimbal_jscope_send)
        g_log_config.gimbal_jscope_send(gimbal_IMU_data->Roll, gimbal_IMU_data->Pitch, gimbal_IMU_data->Yaw,
                             gimbal_IMU_data->Gyro[0], gimbal_IMU_data->Gyro[1], gimbal_IMU_data->Gyro[2]);
    // if ((gyro_print_count++ % 100) == 0)
    // {
    //     SEGGER_RTT_printf(BUFFER_INDEX, "ang yaw:%d pitch:%d roll:%d total:%d (x1000)\r\n",
    //                       (int)(gimbal_IMU_data->Yaw * 1000.0f),
    //                       (int)(gimbal_IMU_data->Pitch * 1000.0f),
    //                       (int)(gimbal_IMU_data->Roll * 1000.0f),
    //                       (int)(gimbal_IMU_data->YawTotalAngle * 1000.0f));
    //     SEGGER_RTT_printf(BUFFER_INDEX, "gyro x:%d y:%d z:%d (x1e6)\r\n",
    //                       (int)(gimbal_IMU_data->Gyro[0] * 1000000.0f),
    //                       (int)(gimbal_IMU_data->Gyro[1] * 1000000.0f),
    //                       (int)(gimbal_IMU_data->Gyro[2] * 1000000.0f));
    // }
    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
