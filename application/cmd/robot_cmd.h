#ifndef GIMBAL_CMD_H
#define GIMBAL_CMD_H

#include <stdint.h>

/* 私有诊断快照，不加入通信协议。角度/角速度仍为 °、°/s。
 * source: 0=本周期无新输入，1=遥控摇杆，2=图传键鼠。 */
typedef struct {
    uint32_t time_us;
    int16_t rc_yaw_raw, rc_yaw_used;
    float yaw_ref, yaw_feedback, yaw_encoder, gyro_z;
    uint8_t source, rc_online, gimbal_mode;
} RobotCMDYawDebug;

/* 只读复制本周期实际使用的输入与最终目标，供低优先级日志任务调用。 */
void RobotCMDGetYawDebug(RobotCMDYawDebug *snapshot);


/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

#endif // !GIMBAL_CMD_H
