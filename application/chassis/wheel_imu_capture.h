#ifndef WHEEL_IMU_CAPTURE_H
#define WHEEL_IMU_CAPTURE_H

#include <stdint.h>

#if defined(WHEEL_IMU_CAPTURE) && WHEEL_IMU_CAPTURE

#include "chassis_motion.h"

/* Wire v1 uses the little-endian Cortex-M4 integer and float representations. */
typedef struct {
    uint32_t time_us, accepted_seq, attempt_seq, generation;
    int16_t accel_counts[3], gyro_counts[3];
    uint8_t read_status, valid_flags, attitude_flags, rigid_reject;
    float accel_b[3], gyro_b[3], q[4], gravity_b[3];
    float accel_at_intersection_b[3], motion_n[3], center_n[3];
} WheelIMUCaptureImu;

enum {
    WIMU_LF, WIMU_RF, WIMU_LB, WIMU_RB, WIMU_YAW, WIMU_MOTOR_COUNT
};

/* Write a nonzero token via live SWD; it is cleared when the capture arms. */
extern volatile uint32_t wheel_imu_capture_request;
uint8_t WheelIMUCaptureActive(void);
void WheelIMUCaptureRegisterMotor(unsigned index, const void *motor);
void WheelIMUCaptureRecordMotor(const void *motor, uint32_t time_us, uint32_t seq,
                                int16_t rotor_rpm, uint16_t encoder, float filtered_dps);
void WheelIMUCaptureRecordImu(const WheelIMUCaptureImu *sample);
void WheelIMUCaptureRecordControl(const ChassisMotionState *state,
                                  const ChassisMotionObservationState *observation);
/* Called by the low-priority logger. A pending request waits for a ready INS and SHADOW. */
void WheelIMUCaptureService(uint8_t arm_allowed, float yaw_zero_deg);

#endif
#endif
