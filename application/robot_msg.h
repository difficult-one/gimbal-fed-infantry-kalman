/**
 * @file robot_msg.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#ifndef ROBOT_MSG_H
#define ROBOT_MSG_H

#include <stdint.h>
#include "build_config.h"
#include "ins.h"
#include "vision_link.h"

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

typedef enum
{
    CHASSIS_CONTROL_VELOCITY = 0, // 轮速闭环，现有默认模式
    CHASSIS_CONTROL_TORQUE,       // 电机电流/估算轮端力矩控制
    CHASSIS_CONTROL_FORCE_TRACK,  // 传感器力闭环跟踪
    CHASSIS_CONTROL_ADMITTANCE    // 力误差转换为速度修正

}Chassis_Control_Mode_e;

#define CHASSIS_CONTROL_PROTOCOL_VERSION 1u

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向旧命令单位；速控局部按 MC_LEGACY_V_SCALE 转 m/s
    float vy;           // 横移方向旧命令单位；不改变力控/导纳的既有语义
    float wz;           // 旧旋转命令单位，不能直接作为 °/s 或 rad/s
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    Chassis_Control_Mode_e control_mode;
    int chassis_speed_buff;
    // UI部分

    float fx_ref;           // N，底盘坐标系 +x
    float fy_ref;           // N，底盘坐标系 +y
    float mz_ref;           // N·m，绕 z 轴
    uint8_t protocol_version;

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;

    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red

} Chassis_Upload_Data_s;

/* 独立力控反馈话题：不并入 Chassis_Upload_Data_s，避免改变原有底盘反馈协议。 */
#define CHASSIS_FORCE_FEED_TOPIC "chassis_force_feed"
#define CHASSIS_FORCE_SENSOR_TOPIC "chassis_force_sensor"
#define CHASSIS_FORCE_FEED_VERSION 1u

typedef enum
{
    CHASSIS_FORCE_SOURCE_INVALID = 0,  // 没有可用力反馈
    CHASSIS_FORCE_SOURCE_ESTIMATE,     // 由电机电流估算
    CHASSIS_FORCE_SOURCE_SENSOR,       // 由轮端力传感器测量
    CHASSIS_FORCE_SOURCE_FAULT         // 传感器/估算链路故障
} Chassis_Force_Source_e;

/* 轮端力传感器输入帧。传感器驱动或外部采集应用发布此话题。 */
typedef struct
{
    uint8_t protocol_version;
    uint8_t force_valid;
    uint8_t force_source;     // 必须为 CHASSIS_FORCE_SOURCE_SENSOR
    uint8_t reserved;
    uint32_t sample_seq;
    float fx;
    float fy;
    float mz;
    float wheel_force[4];     // LF、RF、LB、RB，N
} Chassis_Force_Sensor_s;

typedef struct
{
    uint8_t protocol_version; // CHASSIS_FORCE_FEED_VERSION
    uint8_t force_valid;      // 1 表示下面的力数据可以用于闭环
    uint8_t force_source;     // Chassis_Force_Source_e，使用 uint8_t 保持协议宽度明确
    uint8_t reserved;
    uint32_t sample_seq;      // 每次发布递增，用于检测消息停滞

    float fx;                 // 底盘坐标系合力，N
    float fy;                 // 底盘坐标系合力，N
    float mz;                 // 绕 z 轴合力矩，N·m
    float wheel_force[4];     // LF、RF、LB、RB，N
    float current_ref[4];     // LF、RF、LB、RB，DJI CAN 原始电流量
    float current_feedback[4];// LF、RF、LB、RB，滤波后的 DJI CAN 实际电流量
} Chassis_Force_Feed_s;

/* 保留旧占位类型名，避免已有应用代码因类型名变化而无法编译。 */
typedef Chassis_Force_Feed_s chassis_force_feed;


typedef struct
{
    attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
    // code to go here
    // ...
} Shoot_Upload_Data_s;




#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_MSG_H
