#ifndef IMU_TEMPERATURE_H
#define IMU_TEMPERATURE_H

void IMU_Temperature_Init(float period_ms, float initial_duty);
void IMU_Temperature_Update(float duty_ratio);
#endif 