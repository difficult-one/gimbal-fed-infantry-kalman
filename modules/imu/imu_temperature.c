#include  "imu_temperature.h"
#include "bsp_pwm.h"
static PWMInstance *imu_pwm;
void IMU_Temperature_Init(float period_ms, float initial_duty)
{
    PWM_Init_Config_s imu_config = {
        .htim = &htim3,
        .channel = TIM_CHANNEL_2,
        .period = period_ms,
        .dutyratio = initial_duty,
    };
    imu_pwm = PWMRegister(&imu_config);
}


void IMU_Temperature_Update(float duty_ratio)
{
    PWMSetDutyRatio(imu_pwm, duty_ratio);
}
