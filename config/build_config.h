#ifndef BUILD_CONFIG_H
#define BUILD_CONFIG_H

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#if !defined(ONE_BOARD) && !defined(CHASSIS_BOARD) && !defined(GIMBAL_BOARD)
#define ONE_BOARD // 默认单板；可从编译命令选择另外一种角色。
#endif

// 底盘运动学类型和命令方向修正。
//#define CHASSIS_DRIVE_MECANUM   // 麦克纳姆轮
#if !defined(CHASSIS_DRIVE_MECANUM) && !defined(CHASSIS_DRIVE_OMNI_X) && !defined(CHASSIS_DRIVE_SWERVE)
#define CHASSIS_DRIVE_OMNI_X   // X 型四全向轮
#endif
#if (defined(CHASSIS_DRIVE_MECANUM) + defined(CHASSIS_DRIVE_OMNI_X) + defined(CHASSIS_DRIVE_SWERVE)) != 1
#error "Select exactly one chassis drive type"
#endif
// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif


/* 采集缓冲只编入启用采集的固件。 */
#ifndef WHEEL_IMU_CAPTURE
#define WHEEL_IMU_CAPTURE 0
#endif
#if WHEEL_IMU_CAPTURE && (!defined(ONE_BOARD) || !defined(CHASSIS_DRIVE_OMNI_X))
#error "Wheel/IMU capture requires ONE_BOARD and CHASSIS_DRIVE_OMNI_X"
#endif

#endif
