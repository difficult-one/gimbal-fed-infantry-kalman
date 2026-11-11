// app
#include "robot_msg.h"
#include "config.h"
#include "bsp_usart.h"
#include "robot_cmd.h"
#include "gimbal.h"
#include <math.h>
// module
#include "remote_control.h"
#include "vt02_control.h"
#include "ins.h"
#include "vision_link.h"
#include "message_center.h"
#include "math_units.h"
#include "dji_motor.h"
#include "math_utils.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vt02_Ctrl_t *vt02_data;          // 图传键鼠数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
static RobotCMDYawDebug yaw_debug;
static int16_t yaw_input_raw, yaw_input_used;
static uint8_t yaw_input_source;

void RobotCMDGetYawDebug(RobotCMDYawDebug *snapshot)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    *snapshot = yaw_debug;
    __set_PRIMASK(key);
}

static int16_t ChassisRCDeadBand(int16_t input)
{
    return (input > -g_input_config.chassis_dead_band && input < g_input_config.chassis_dead_band) ? 0 : input;
}

static int16_t GimbalRCDeadBand(int16_t input)
{
    return (input >= -g_input_config.gimbal_dead_band && input <= g_input_config.gimbal_dead_band) ? 0 : input;
}

void RobotCMDInit()
{
    rc_data = RemoteControlInit(BSPUSARTHandle(g_link_config.remote_uart));   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(BSPUSARTHandle(g_link_config.vision_uart)); // 视觉通信串口
    vt02_data = Vt02ControlInit(BSPUSARTHandle(g_link_config.vt02_uart));   // 图传键鼠通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = BSPCANHandle(g_link_config.board_can),
            .tx_id = g_link_config.board_tx_id,
            .rx_id = g_link_config.board_rx_id,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .daemon_count = g_link_config.board_daemon_count,
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    /* 力控命令默认为传统速度模式，需由上位机显式切换到 TORQUE。 */
    chassis_cmd_send.control_mode = CHASSIS_CONTROL_VELOCITY;
    chassis_cmd_send.fx_ref = 0.0f;
    chassis_cmd_send.fy_ref = 0.0f;
    chassis_cmd_send.mz_ref = 0.0f;
    chassis_cmd_send.protocol_version = CHASSIS_CONTROL_PROTOCOL_VERSION;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 云台相对底盘的夹角 = 当前yaw电机单圈角度 - 对齐零位角度, 归一化到 [-180, 180]
    /* 公共反馈保留原协议；方向转换直接读取最新CAN快照，避免上一机器人周期
     * 的消息延迟和uint16_t整度截断。离线/过期时沿用原回传值，不外推失效样本。 */
    GimbalYawFeedback yaw;
    GimbalGetYawFeedback(&yaw);
    float angle = (float)gimbal_fetch_data.yaw_motor_single_round_angle;
    if (yaw.online && isfinite(yaw.angle_deg) &&
        (uint32_t)(yaw.control_time_us - yaw.time_us) <= g_input_config.yaw_feedback_max_age_us)
        angle = yaw.angle_deg;
    chassis_cmd_send.offset_angle = theta_format(
        angle - g_gimbal_config.yaw_align_ecd * ECD_ANGLE_COEF_DJI);
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,由右侧三段开关选择:
    // [上] 不跟随: 底盘全向平移不旋转, 云台保持IMU稳像
    // [中] 随动:   底盘跟随云台YAW指向, 云台用IMU反馈(陀螺仪稳像)
    // [下] 小陀螺: 底盘恒定自旋, 云台用IMU反馈保持绝对指向
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }

    // 云台参数,确定云台控制数据
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中],视觉模式
    {
        // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
        // ...
    }
    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || vision_recv_data->target_state == NO_TARGET)
    { // 按照摇杆的输出大小进行角度增量,增益系数需调整
        /* 回中时不再累加角度，避免微小中位偏差造成云台持续缓慢转动。 */
        /* 保存实际参与本周期累加的同一个值，日志不能另读下一帧遥控数据。 */
        yaw_input_raw = rc_data[TEMP].rc.rocker_l_;
        yaw_input_used = GimbalRCDeadBand(yaw_input_raw);
        gimbal_cmd_send.yaw += g_input_config.rc_yaw_scale * (float)yaw_input_used;
        gimbal_cmd_send.pitch += g_input_config.rc_pitch_scale * (float)GimbalRCDeadBand(rc_data[TEMP].rc.rocker_l1);
    }
    // 云台软件限位

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    /* 保留现有手感/协议数值；SI 换算只在底盘速控入口进行，见 config.c。 */
    chassis_cmd_send.vx = g_input_config.rc_vx_scale * (float)ChassisRCDeadBand(rc_data[TEMP].rc.rocker_r1); // 右摇杆竖直方向：前进/后退
    chassis_cmd_send.vy = g_input_config.rc_vy_scale * (float)ChassisRCDeadBand(rc_data[TEMP].rc.rocker_r_);  // 右摇杆水平方向：左/右横移

    // 发射参数
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
        ;                                            // 弹舱舵机控制,待添加servo_motor模块,开启
    else
        ; // 弹舱舵机控制,待添加servo_motor模块,关闭

    // 摩擦轮/拨盘分两段控制,拨轮向上打为负,向下为正;拨轮量程约 -660~+660
    // 第一段: 向上拨过 -150, 只开摩擦轮(单独转); 第二段: 拨过 -350, 摩擦轮+拨盘一起动
    if (rc_data[TEMP].rc.dial < g_input_config.friction_dial) // 推一点,摩擦轮先单独转
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
        // 遥控器没有图传键盘的 Z 键，固定使用 30 m/s 档位。
        // 不赋值时会落入 shoot.c 的 default 分支，当前目标速度只有 1000 deg/s。
        shoot_cmd_send.bullet_speed = SMALL_AMU_30;
    }
    else
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.bullet_speed = BULLET_SPEED_NONE;
    }
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial < g_input_config.loader_dial) // 再往上拨,拨盘跟着一起动,不需要拨到底
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = g_input_config.rc_shoot_rate;
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].d * 300 - rc_data[TEMP].key[KEY_PRESS].a * 300;

    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / g_input_config.mouse_full_scale * g_input_config.mouse_angle_deg; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / g_input_config.mouse_full_scale * g_input_config.mouse_angle_deg;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }
}

/**
 * @brief 控制输入为图传的模式和控制量设置
 *
 */
static void VideoTransmissionControlSet()
{
    static uint8_t last_mouse_left = 0;
    static uint8_t video_force_mode = 0;
    uint8_t mouse_left_pressed;

    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    /* 标准 VT02 没有 L 键，使用标准键盘位域 G 键换挡。 */
    video_force_mode = (uint8_t)(vt02_data->key_l_count & 1u);

    if (video_force_mode == 0u)
    {
        chassis_cmd_send.control_mode = CHASSIS_CONTROL_VELOCITY;
        chassis_cmd_send.vx = -vt02_data->key[KEY_PRESS].w * g_input_config.video_speed_scale +
                               vt02_data->key[KEY_PRESS].s * g_input_config.video_speed_scale;
        chassis_cmd_send.vy = -vt02_data->key[KEY_PRESS].d * g_input_config.video_speed_scale +
                               vt02_data->key[KEY_PRESS].a * g_input_config.video_speed_scale;
        chassis_cmd_send.wz = 0.0f;
        chassis_cmd_send.fx_ref = 0.0f;
        chassis_cmd_send.fy_ref = 0.0f;
        chassis_cmd_send.mz_ref = 0.0f;
    }
    else
    {
        /* 当前图传力控采用电机电流估算轮端力矩，不依赖轮端力传感器。 */
        chassis_cmd_send.control_mode = CHASSIS_CONTROL_TORQUE;
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        chassis_cmd_send.wz = 0.0f;
        chassis_cmd_send.fx_ref = float_constrain(
            (-vt02_data->key[KEY_PRESS].w + vt02_data->key[KEY_PRESS].s) * g_input_config.video_force_x_step,
            -g_input_config.video_force_max, g_input_config.video_force_max);
        chassis_cmd_send.fy_ref = float_constrain(
            (-vt02_data->key[KEY_PRESS].d + vt02_data->key[KEY_PRESS].a) * g_input_config.video_force_y_step,
            -g_input_config.video_force_max, g_input_config.video_force_max);
        chassis_cmd_send.mz_ref = float_constrain(
            -(float)vt02_data->mouse.x * g_input_config.video_force_mz_scale,
            -g_input_config.video_mz_max, g_input_config.video_mz_max);
    }

    gimbal_cmd_send.yaw -= (float)vt02_data->mouse.x / g_input_config.mouse_full_scale * g_input_config.mouse_angle_deg;
    gimbal_cmd_send.pitch += (float)vt02_data->mouse.y / g_input_config.mouse_full_scale * g_input_config.mouse_angle_deg;

    mouse_left_pressed = vt02_data->mouse.press_l && !last_mouse_left;
    last_mouse_left = vt02_data->mouse.press_l;

    switch (vt02_data->key_count[KEY_PRESS][Key_Z] % 3) // 设置摩擦轮转速
    {
    case 0:
        shoot_cmd_send.bullet_speed = SMALL_AMU_15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = SMALL_AMU_18;
        break;
    default:
        shoot_cmd_send.bullet_speed = SMALL_AMU_30;
        break;
    }

    switch (vt02_data->key_count[KEY_PRESS][Key_E] % 4) // 设置拨弹盘转动角度——射击模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        // 单发只响应鼠标左键按下沿,避免松开鼠标中断发射
        shoot_cmd_send.load_mode = mouse_left_pressed ? LOAD_1_BULLET : LOAD_STOP;
        break;
    case 2:
        // 三连发只响应鼠标左键按下沿,避免松开鼠标中断发射
        shoot_cmd_send.load_mode = mouse_left_pressed ? LOAD_3_BULLET : LOAD_STOP;
        break;
    default:
        // 连射时左键按下运行,松开停止
        shoot_cmd_send.load_mode = vt02_data->mouse.press_l ? LOAD_BURSTFIRE : LOAD_STOP;
        break;
    }

    switch (vt02_data->key_count[KEY_PRESS][Key_R] % 2) // 开启/关闭弹仓
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }

    switch (vt02_data->key_count[KEY_PRESS][Key_F] % 2) // 开启摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }

    switch (vt02_data->key_count[KEY_PRESS][Key_C] % 4) // 设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }

    switch (vt02_data->key_count[KEY_PRESS][Key_X] % 4) // 设置底盘模式
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        break;
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    case 2:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;
    default:
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        break;
    }

    shoot_cmd_send.shoot_rate = g_input_config.video_shoot_rate;
}
/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (!RemoteControlIsOnline() || rc_data[TEMP].rc.dial > g_input_config.stop_dial) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        chassis_cmd_send.vx = 0;
        chassis_cmd_send.vy = 0;
        chassis_cmd_send.wz = 0;
        chassis_cmd_send.fx_ref = 0.0f;
        chassis_cmd_send.fy_ref = 0.0f;
        chassis_cmd_send.mz_ref = 0.0f;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (RemoteControlIsOnline() &&
        (switch_is_up(rc_data[TEMP].rc.switch_right) ||
         switch_is_mid(rc_data[TEMP].rc.switch_right) ||
         switch_is_down(rc_data[TEMP].rc.switch_right)))
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

static void GimbalCommandLimit()
{
    float yaw_feedback = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    float yaw_error = gimbal_cmd_send.yaw - yaw_feedback;
    yaw_error = float_constrain(yaw_error,-g_gimbal_config.yaw_max_track_error_deg,g_gimbal_config.yaw_max_track_error_deg);

    gimbal_cmd_send.yaw = yaw_feedback + yaw_error;
    // PITCH 必须限制自身目标，不能复用 YAW 误差，否则左右摇杆会串扰 PITCH。
    gimbal_cmd_send.pitch = float_constrain(
        gimbal_cmd_send.pitch, g_gimbal_config.pitch_min_deg, g_gimbal_config.pitch_max_deg);

}



/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    static uint32_t feedback_seq, feedback_time_us;
    (void)CANCommGetSnapshot(cmd_can_comm, &chassis_fetch_data, sizeof(chassis_fetch_data),
                             &feedback_seq, &feedback_time_us);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();

    /* 每个周期先回到速度模式；图传函数在 L 键切到力矩模式时覆盖这些字段。
       这样切回遥控器或图传速度模式时不会遗留上一帧的力控命令。 */
    chassis_cmd_send.control_mode = CHASSIS_CONTROL_VELOCITY;
    chassis_cmd_send.fx_ref = 0.0f;
    chassis_cmd_send.fy_ref = 0.0f;
    chassis_cmd_send.mz_ref = 0.0f;

    yaw_input_raw = rc_data[TEMP].rc.rocker_l_;
    yaw_input_used = 0;
    yaw_input_source = 0;
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是图传键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
    {
        yaw_input_source = 1;
        RemoteControlSet();
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],图传控制
    {
        yaw_input_source = 2;
        VideoTransmissionControlSet();
    }

    // pitch软件限位:反馈是绝对欧拉角,参考值必须限制在机械行程内,否则会一直累加导致云台顶死限位发热
    if (gimbal_cmd_send.pitch > g_gimbal_config.pitch_max_deg)
        gimbal_cmd_send.pitch = g_gimbal_config.pitch_max_deg;
    else if (gimbal_cmd_send.pitch < g_gimbal_config.pitch_min_deg)
        gimbal_cmd_send.pitch = g_gimbal_config.pitch_min_deg;

    GimbalCommandLimit();
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    /* 只发布诊断，不修改目标或反馈。feedback 来自上一轮云台上报，约滞后一个
     * 机器人周期；time_us 标注命令帧时间，不能将其误认为底盘日志的采样时间。 */
    RobotCMDYawDebug next = {
        .time_us = (uint32_t)DWT_GetTimeline_us(),
        .rc_yaw_raw = yaw_input_raw, .rc_yaw_used = yaw_input_used,
        .yaw_ref = gimbal_cmd_send.yaw,
        .yaw_feedback = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle,
        .yaw_encoder = gimbal_fetch_data.yaw_motor_single_round_angle,
        .gyro_z = gimbal_fetch_data.gimbal_imu_data.Gyro[2],
        .source = yaw_input_source, .rc_online = RemoteControlIsOnline(),
        .gimbal_mode = gimbal_cmd_send.gimbal_mode,
    };
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    yaw_debug = next;
    __set_PRIMASK(key);
}
