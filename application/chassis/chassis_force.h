#ifndef CHASSIS_FORCE_H
#define CHASSIS_FORCE_H

#include "dji_motor.h"
#include "robot_msg.h"

void ChassisForceInit(void);
void ChassisForceResetControl(void);
uint8_t ChassisForceCommandValid(const Chassis_Ctrl_Cmd_s *command);
uint8_t ChassisForceReadMeasurement(const Chassis_Force_Sensor_s *sample,
                                    float *fx, float *fy, float *mz);
/* Read wheel feedback in LF/RF/LB/RB order and return the published force estimate. */
const Chassis_Force_Feed_s *ChassisForceUpdateFeedback(uint8_t motors_online,
                                                      const Chassis_Ctrl_Cmd_s *command,
                                                      DJIMotorInstance *const motors[4]);
/* Return LF/RF/LB/RB references in DJI CAN current units. */
const float *ChassisForceTorque(float fx, float fy, float mz, float gyro_z_dps,
                                DJIMotorInstance *const motors[4]);
const float *ChassisForceTrack(float fx, float fy, float mz, const Chassis_Ctrl_Cmd_s *command,
                                float gyro_z_dps, DJIMotorInstance *const motors[4]);
/* Output the three legacy velocity axes before conversion to the chassis frame. */
void ChassisForceAdmittance(float fx, float fy, float mz, const Chassis_Ctrl_Cmd_s *command,
                           float velocity[3]);

#endif
