/**
 * @file vt02_control.h
 * @author wbz
 * @brief  图传控制模块定义头文件
 * @version beta
 * @date 2026-09-13
 *
 * @copyright Copyright (c) 2026 NTU NSE EC all rights reserved
 *
 */

#ifndef VT02_CONTROL_H
#define VT02_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"
#include "remote_control.h"

#pragma pack(1)
typedef struct
{
    int16_t mouse_x;        // 鼠标X轴移动速度,负值向左
    int16_t mouse_y;        // 鼠标Y轴移动速度,负值向下
    int16_t mouse_z;        // 鼠标滚轮速度
    int8_t left_button;     // 鼠标左键状态
    int8_t right_button;    // 鼠标右键状态
    uint16_t keyboard_value; // 键盘按键位域
    uint16_t reserved;       // 扩展字段：bit0 为旧版扩展换挡键；标准协议使用 G 键
} Vt02_Raw_Data_t;
#pragma pack()

typedef struct
{
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3];
    uint8_t key_count[3][16];
    uint8_t key_l;             // 换挡键当前状态：标准协议为 G，兼容 reserved bit0
    uint8_t key_l_count;       // 换挡键上升沿计数
    uint8_t online;
} Vt02_Ctrl_t;

/**
 * @brief 初始化VT02图传控制模块,并注册接收串口
 *
 * @param vt02_usart_handle VT02使用的串口句柄
 * @retval Vt02_Ctrl_t* VT02键鼠控制数据指针
 */
Vt02_Ctrl_t *Vt02ControlInit(UART_HandleTypeDef *vt02_usart_handle);

/**
 * @brief 检查VT02图传链路是否在线,若尚未初始化也视为离线
 *
 * @retval uint8_t 1:在线 0:离线
 */
uint8_t Vt02ControlIsOnline(void);

#endif
