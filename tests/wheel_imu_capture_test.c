#include "wheel_imu_capture.h"
#include "bsp_dwt.h"
#include "SEGGER_RTT.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

uint32_t host_primask;
static uint32_t now, generation;
static uint8_t transport_full;
static uint8_t transport[100000];
static unsigned written;
static int wheel;

uint64_t DWT_GetTimeline_us(void) { return now; }
void DWT_GetClockDebug(DWT_ClockDebug *out) { out->generation = generation; }

int SEGGER_RTT_ConfigUpBuffer(unsigned index, const char *name, void *buffer,
                             unsigned size, unsigned flags)
{
    assert(index == 1 && name && buffer && size > 512 && flags == SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    return 0;
}

unsigned SEGGER_RTT_Write(unsigned index, const void *buffer, unsigned size)
{
    assert(index == 1 && written + size <= sizeof(transport));
    if (transport_full) return 0;
    memcpy(transport + written, buffer, size);
    written += size;
    return size;
}

void ChassisMotionGetConfig(ChassisMotionConfig *config)
{
    static const float rows[4][3] = {
        {-.70710678f,.70710678f,.2298097f}, {.70710678f,.70710678f,.2298097f},
        {-.70710678f,-.70710678f,.2298097f}, {.70710678f,-.70710678f,.2298097f}
    };
    *config = (ChassisMotionConfig){
        .version = 0x20261110u, .wheels.type = CK_CUSTOM, .max_omega = 9.0f,
        .algebra_y_sign = -1, .center_y_sign = -1,
        .spin_wheel_trim_dps = {66,-107,29,12}
    };
    memcpy(config->wheels.rows, rows, sizeof(rows));
    for (unsigned i=0; i<4; ++i) {
        config->wheels.radius_m[i] = .076f;
        config->wheels.gear_ratio[i] = 19.0f;
        config->wheels.motor_sign[i] = 1.0f;
    }
}

static unsigned ReadU16(unsigned offset)
{
    return transport[offset] | (unsigned)transport[offset+1] << 8;
}

static void Drain(void)
{
    for (unsigned i=0; i<256; ++i) WheelIMUCaptureService(0, 16.25965f);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    WheelIMUCaptureRegisterMotor(WIMU_LF, &wheel);
    now = UINT32_MAX - 100;
    generation = 7;
    wheel_imu_capture_request = 42;
    WheelIMUCaptureService(0, 16.25965f);
    assert(!WheelIMUCaptureActive() && wheel_imu_capture_request == 42);
    WheelIMUCaptureService(1, 16.25965f);
    assert(WheelIMUCaptureActive() && !wheel_imu_capture_request);

    WheelIMUCaptureImu imu = {.time_us=30, .accepted_seq=8, .attempt_seq=10, .generation=7,
                              .accel_counts={2,-3,4}, .gyro_counts={-1,0,6}, .valid_flags=7};
    imu.q[0] = 1.0f;
    imu.accel_b[2] = 9.81f;
    WheelIMUCaptureRecordImu(&imu);
    imu.time_us = 80;
    imu.attempt_seq++;
    imu.read_status = 3;
    imu.valid_flags = 0;
    memset(imu.accel_counts, 0, sizeof(imu.accel_counts));
    memset(imu.gyro_counts, 0, sizeof(imu.gyro_counts));
    WheelIMUCaptureRecordImu(&imu);

    WheelIMUCaptureRecordMotor(NULL, 90, 99, 1234, 0, 42.0f);
    WheelIMUCaptureRecordMotor(&wheel, 90, UINT32_MAX, -50, 370, -296.5f);
    WheelIMUCaptureRecordMotor(&wheel, 140, 0, -51, 368, -300.0f);
    ChassisMotionState state = {.seq=31, .time_us=160, .imu_seq=8, .theta_bg=.25f,
                               .mode=MC_SHADOW, .estimate_valid=1, .wheel_eff_dps={123,234,345,456}};
    ChassisMotionObservationState obs = {.clock_generation=7, .control_inhibit=8, .reject_reason=2};
    WheelIMUCaptureRecordControl(&state, &obs);
    assert(written == 0 && WheelIMUCaptureActive());

    now += 350000u;
    transport_full = 1;
    WheelIMUCaptureService(1, 16.25965f);
    assert(!WheelIMUCaptureActive() && written == 0);
    transport_full = 0;
    Drain();
    assert(written == 184+2*120+2*20+116+4);
    assert(!memcmp(transport, "WIMU", 4));
    assert(ReadU16(6) == 184 && ReadU16(40) == 2 && ReadU16(42) == 2 && ReadU16(52) == 1);
    FILE *out = fopen(argv[1], "wb");
    assert(out && fwrite(transport, 1, written, out) == written && fclose(out) == 0);

    written = 0;
    wheel_imu_capture_request = 43;
    WheelIMUCaptureService(1, 16.25965f);
    assert(WheelIMUCaptureActive());
    for (unsigned i=0; i<369; ++i)
        WheelIMUCaptureRecordMotor(&wheel, now+i*10u, i, -12, 370, -70.0f);
    assert(!WheelIMUCaptureActive());
    Drain();
    assert(written == 184+368*20+4);
    assert(ReadU16(32) == 2 && ReadU16(36) == 2 && ReadU16(42) == 368);

    written = 0;
    wheel_imu_capture_request = 44;
    WheelIMUCaptureService(1, 16.25965f);
    assert(WheelIMUCaptureActive());
    generation++;
    WheelIMUCaptureService(1, 16.25965f);
    Drain();
    assert(written == 188 && ReadU16(32) == 3);

    written = 0;
    wheel_imu_capture_request = 45;
    WheelIMUCaptureService(1, 16.25965f);
    obs.clock_generation = generation;
    state.time_us = now;
    state.mode = MC_ACTIVE_FIXED;
    WheelIMUCaptureRecordControl(&state, &obs);
    Drain();
    assert(written == 188 && ReadU16(32) == 4);
    puts("PASS: capture arming, timer and sequence wrap, source filter, overflow, clock/mode freeze and RTT retry");
}
