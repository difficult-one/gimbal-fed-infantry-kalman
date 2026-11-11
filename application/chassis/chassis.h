#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

/* SWERVE platform hooks. Order LF/RF/LB/RB; B.x forward, B.y left, CCW radians.
 * Feedback timestamps use local DWT microseconds. Read runs with IRQs masked:
 * only copy cached feedback; do not block, transmit or wait. Return 0 on failure.
 * Weak defaults reject output until actual steering hardware is connected. */
uint8_t ChassisReadSteeringFeedback(float angle_rad[4], uint32_t time_us[4], uint8_t *online_mask);
uint8_t ChassisApplySteeringTargets(const float angle_rad[4], uint8_t enabled);

#endif // CHASSIS_H
