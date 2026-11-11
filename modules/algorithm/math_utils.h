/**
 ******************************************************************************
 * @file	 math_utils.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>
#include "math_units.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

// 快速开方
float Sqrt(float x);

// 绝对值限制
float abs_limit(float num, float Limit);

// 判断符号位
float sign(float value);

// 浮点死区
float float_deadband(float Value, float minValue, float maxValue);

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue);

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue);

// 弧度格式化为-PI~PI

// 角度格式化为-180~180
float theta_format(float Ang);

int float_rounding(float raw);

float* Norm3d(float* v);

float NormOf3d(float* v);

void Cross3d(float* v1, float* v2, float* res);

float Dot3d(float* v1, float* v2);

float AverageFilter(float new_data, float *buf, uint8_t len);

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif
