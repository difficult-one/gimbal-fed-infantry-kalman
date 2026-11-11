#ifndef WHEEL_IMU_CAPTURE_H
#define WHEEL_IMU_CAPTURE_H

#include "build_config.h"
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

typedef struct {
    uint32_t version, frame_revision;
    float radius_m[4], gear_ratio[4], motor_sign[4], rows[4][3], spin_trim_dps[4];
    float spin_ref_radps, yaw_encoder_sign, yaw_zero_deg;
    int8_t algebra_y_sign, center_y_sign;
} WheelIMUCaptureConfig;

/* Write a nonzero token via live SWD; it is cleared when the capture arms. */
extern volatile uint32_t wheel_imu_capture_request;
uint8_t WheelIMUCaptureActive(void);
void WheelIMUCaptureRegisterMotor(unsigned index, const void *motor);
void WheelIMUCaptureRecordMotor(const void *motor, uint32_t time_us, uint32_t seq,
                                int16_t rotor_rpm, uint16_t encoder, float filtered_dps);
void WheelIMUCaptureRecordImu(const WheelIMUCaptureImu *sample);
void WheelIMUCaptureRecordControl(const ChassisMotionState *state,
                                  const ChassisMotionObservationState *observation);
/* Called by the low-priority logger. A null config defers arming but keeps exporting. */
void WheelIMUCaptureService(const WheelIMUCaptureConfig *config,
                            uint32_t duration_us, unsigned rtt_channel);

#endif
#endif
