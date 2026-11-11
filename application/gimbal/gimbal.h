#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

/* 私有单板读取接口，不改变 robot_msg.h 的既有通信结构。
 * 编码器保留小数；time_us 是CAN样本时间，control_time_us 是诊断读取时间。 */
typedef struct {
    uint32_t time_us, control_time_us;
    float angle_deg;
    float speed_p_raw, speed_i_raw, output_raw, sent_raw;
    uint8_t online;
} GimbalYawFeedback;

void GimbalGetYawFeedback(GimbalYawFeedback *feedback);

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H
