#include "bsp_init.h"
#include "robot.h"
#include "config.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"

void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit(g_chassis_log_config.text_rtt_channel);
   
#ifndef CHASSIS_BOARD
    RobotCMDInit();
    GimbalInit(); /* Owns this board's INS on GIMBAL_BOARD. */
    ShootInit();
#endif
#ifndef GIMBAL_BOARD
    ChassisInit(); /* Dual board: local chassis INS, no gimbal drivers. */
#endif

    __enable_irq();
}

void RobotTask()
{
#ifndef CHASSIS_BOARD
    RobotCMDTask();
    GimbalTask();
    ShootTask();
#endif
#ifndef GIMBAL_BOARD
    ChassisTask();
#endif
}
