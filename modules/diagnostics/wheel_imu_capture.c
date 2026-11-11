#include "wheel_imu_capture.h"

#if defined(WHEEL_IMU_CAPTURE) && WHEEL_IMU_CAPTURE

#include "bsp_dwt.h"
#include "SEGGER_RTT.h"
#include <float.h>
#include <string.h>

enum {
    IMU_CAPACITY = 368u, MOTOR_CAPACITY = 368u,
    CONTROL_CAPACITY = 80u, EXPORT_CHUNK = 512u
};
enum { IDLE, RECORDING, FROZEN };
enum { STOP_DURATION = 1, STOP_FULL, STOP_CLOCK, STOP_MODE };
enum { SRC_IMU, SRC_LF, SRC_CONTROL = SRC_LF + WIMU_MOTOR_COUNT, SOURCE_COUNT };

typedef struct {
    char magic[4];
    uint16_t version, header_size;
    uint32_t fw_id, frame, config_version, token, start_us, end_us, stop_reason, overflow_source;
    uint16_t counts[SOURCE_COUNT], reserved;
    float radius_m[4], gear_ratio[4], motor_sign[4], rows[4][3], spin_trim_dps[4];
    float spin_ref_radps, yaw_encoder_sign, yaw_zero_deg;
    int8_t algebra_y_sign, center_y_sign;
    uint8_t drive, reserved2;
} CaptureHeader;

typedef struct {
    uint32_t time_us, seq, generation;
    int16_t rotor_rpm;
    uint16_t encoder;
    float filtered_dps;
} CaptureMotor;

typedef struct {
    uint32_t time_us, seq, generation, imu_seq, wheel_seq[4];
    float theta_bg, psi_bn, wheel_n[2], accel_center_n[2], wheel_eff_dps[4];
    float omega_ref, ref_g[2], cmd_g[2], dv_now[2], dv_res[2];
    uint16_t control_inhibit;
    uint8_t reject_reason, exit_reason, wheel_valid_mask, observation_mask, flags, mode;
} CaptureControl;

_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ &&
               sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24,
               "WIMU needs little-endian binary32");
_Static_assert(sizeof(CaptureHeader) == 184 && sizeof(WheelIMUCaptureImu) == 120 &&
               sizeof(CaptureMotor) == 20 && sizeof(CaptureControl) == 116,
               "WIMU v1 wire size");

volatile uint32_t wheel_imu_capture_request;
static volatile uint8_t phase;
static CaptureHeader header;
static WheelIMUCaptureImu imu_records[IMU_CAPACITY];
static CaptureMotor motor_records[WIMU_MOTOR_COUNT][MOTOR_CAPACITY];
static CaptureControl control_records[CONTROL_CAPACITY];
static const void *motor_sources[WIMU_MOTOR_COUNT];
static uint32_t capture_generation, capture_duration, latest_elapsed;
static uint32_t crc, export_offset;
static uint8_t export_section;
static char rtt_buffer[2048];
static uint8_t rtt_ready;

static uint32_t Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}

static void Freeze(uint32_t elapsed, uint32_t reason, uint32_t overflow_source)
{
    if (elapsed < latest_elapsed || elapsed > INT32_MAX) elapsed = latest_elapsed;
    header.end_us = header.start_us + elapsed;
    header.stop_reason = reason;
    header.overflow_source = overflow_source;
    phase = FROZEN;
}

/* Called under the short append lock. Modular time allows a µs counter wrap
 * while rejecting a sample whose timestamp predates arming. */
static uint8_t InWindow(uint32_t time_us, uint32_t generation)
{
    if (phase != RECORDING) return 0;
    uint32_t elapsed = time_us - header.start_us;
    if (elapsed > INT32_MAX) return 0;
    if (generation != capture_generation) Freeze(elapsed, STOP_CLOCK, 0);
    else if (elapsed >= capture_duration) Freeze(capture_duration, STOP_DURATION, 0);
    else {
        if (elapsed > latest_elapsed) latest_elapsed = elapsed;
        return 1;
    }
    return 0;
}

static uint8_t CanAppend(unsigned source, unsigned capacity, uint32_t time_us, uint32_t generation)
{
    if (!InWindow(time_us, generation)) return 0;
    if (header.counts[source] < capacity) return 1;
    Freeze(time_us - header.start_us, STOP_FULL, source + 1u);
    return 0;
}

uint8_t WheelIMUCaptureActive(void)
{
    return phase == RECORDING;
}

void WheelIMUCaptureRegisterMotor(unsigned index, const void *motor)
{
    if (index < WIMU_MOTOR_COUNT) motor_sources[index] = motor;
}

void WheelIMUCaptureRecordMotor(const void *motor, uint32_t time_us, uint32_t seq,
                                int16_t rotor_rpm, uint16_t encoder, float filtered_dps)
{
    if (phase != RECORDING || !motor) return;
    for (unsigned i = 0; i < WIMU_MOTOR_COUNT; ++i) {
        if (motor_sources[i] != motor) continue;
        uint32_t key = Lock();
        DWT_ClockDebug clock;
        DWT_GetClockDebug(&clock);
        unsigned source = SRC_LF + i;
        if (CanAppend(source, MOTOR_CAPACITY, time_us, clock.generation)) {
            motor_records[i][header.counts[source]++] =
                (CaptureMotor){time_us, seq, clock.generation, rotor_rpm, encoder, filtered_dps};
        }
        __set_PRIMASK(key);
        return;
    }
}

void WheelIMUCaptureRecordImu(const WheelIMUCaptureImu *sample)
{
    uint32_t key = Lock();
    if (CanAppend(SRC_IMU, IMU_CAPACITY, sample->time_us, sample->generation))
        imu_records[header.counts[SRC_IMU]++] = *sample;
    __set_PRIMASK(key);
}

void WheelIMUCaptureRecordControl(const ChassisMotionState *s,
                                  const ChassisMotionObservationState *obs)
{
    if (phase != RECORDING) return;
    CaptureControl record = {
        .time_us=s->time_us, .seq=s->seq, .generation=obs->clock_generation, .imu_seq=s->imu_seq,
        .theta_bg=s->theta_bg, .psi_bn=s->psi_bn, .omega_ref=s->omega_ref,
        .control_inhibit=obs->control_inhibit, .reject_reason=obs->reject_reason,
        .exit_reason=s->exit_reason, .wheel_valid_mask=s->wheel_valid_mask,
        .observation_mask=s->observation_mask, .mode=s->mode,
        .flags=(uint8_t)((!!s->estimate_valid) | ((!!s->accel_used)<<1) |
                        ((!!obs->control_ready)<<2) | ((!!s->accel_center_valid)<<3))
    };
    memcpy(record.wheel_seq, s->wheel_seq, sizeof(record.wheel_seq));
    memcpy(record.wheel_n, s->wheel_n, sizeof(record.wheel_n));
    memcpy(record.accel_center_n, s->accel_center_n, sizeof(record.accel_center_n));
    memcpy(record.wheel_eff_dps, s->wheel_eff_dps, sizeof(record.wheel_eff_dps));
    memcpy(record.ref_g, s->ref_g, sizeof(record.ref_g));
    memcpy(record.cmd_g, s->cmd_g, sizeof(record.cmd_g));
    memcpy(record.dv_now, obs->delta_v_partial, sizeof(record.dv_now));
    memcpy(record.dv_res, obs->delta_v_residual, sizeof(record.dv_res));

    uint32_t key = Lock();
    if (InWindow(record.time_us, record.generation)) {
        if (s->mode != MC_SHADOW) Freeze(record.time_us-header.start_us, STOP_MODE, 0);
        else if (CanAppend(SRC_CONTROL, CONTROL_CAPACITY, record.time_us, record.generation))
            control_records[header.counts[SRC_CONTROL]++] = record;
    }
    __set_PRIMASK(key);
}

static void Arm(uint32_t token, const WheelIMUCaptureConfig *config, uint32_t duration_us)
{
    header = (CaptureHeader){
        .magic={'W','I','M','U'}, .version=1, .header_size=sizeof(header),
        .fw_id=config->version, .frame=config->frame_revision, .config_version=config->version,
        .token=token, .spin_ref_radps=config->spin_ref_radps,
        .yaw_encoder_sign=config->yaw_encoder_sign, .yaw_zero_deg=config->yaw_zero_deg,
        .algebra_y_sign=config->algebra_y_sign, .center_y_sign=config->center_y_sign, .drive=1
    };
    memcpy(header.radius_m, config->radius_m, sizeof(header.radius_m));
    memcpy(header.gear_ratio, config->gear_ratio, sizeof(header.gear_ratio));
    memcpy(header.motor_sign, config->motor_sign, sizeof(header.motor_sign));
    memcpy(header.rows, config->rows, sizeof(header.rows));
    memcpy(header.spin_trim_dps, config->spin_trim_dps, sizeof(header.spin_trim_dps));
    capture_duration = duration_us;
    crc = UINT32_MAX;
    export_offset = export_section = 0;
    latest_elapsed = 0;

    uint32_t key = Lock();
    DWT_ClockDebug clock;
    header.start_us = (uint32_t)DWT_GetTimeline_us();
    DWT_GetClockDebug(&clock);
    capture_generation = clock.generation;
    wheel_imu_capture_request = 0;
    phase = RECORDING;
    __set_PRIMASK(key);
}

/* Only the low-priority logger reads frozen buffers. Advance only after an
 * entire chunk fits in the nonblocking RTT buffer. */
static void Export(unsigned rtt_channel)
{
    enum { TRAILER_SECTION = SOURCE_COUNT + 1 };
    const uint8_t *bytes;
    unsigned length;
    uint32_t trailer = crc ^ UINT32_MAX;
    for (;;) {
        if (export_section == 0) {
            bytes = (const uint8_t *)&header;
            length = sizeof(header);
        } else if (export_section == SRC_IMU + 1u) {
            bytes = (const uint8_t *)imu_records;
            length = header.counts[SRC_IMU] * sizeof(imu_records[0]);
        } else if (export_section <= SRC_LF + WIMU_MOTOR_COUNT) {
            unsigned source = export_section - 1u;
            bytes = (const uint8_t *)motor_records[source-SRC_LF];
            length = header.counts[source] * sizeof(motor_records[0][0]);
        } else if (export_section == SRC_CONTROL + 1u) {
            bytes = (const uint8_t *)control_records;
            length = header.counts[SRC_CONTROL] * sizeof(control_records[0]);
        } else {
            bytes = (const uint8_t *)&trailer;
            length = sizeof(trailer);
        }
        if (export_offset < length) break;
        ++export_section;
        export_offset = 0;
    }

    unsigned remaining = length - export_offset;
    unsigned chunk = remaining < EXPORT_CHUNK ? remaining : EXPORT_CHUNK;
    bytes += export_offset;
    if (SEGGER_RTT_Write(rtt_channel, bytes, chunk) != chunk) return;
    if (export_section == TRAILER_SECTION) {
        phase = IDLE;
        return;
    }
    for (unsigned i = 0; i < chunk; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0);
    }
    export_offset += chunk;
}

void WheelIMUCaptureService(const WheelIMUCaptureConfig *config,
                            uint32_t duration_us, unsigned rtt_channel)
{
    if (!rtt_ready) {
        if (SEGGER_RTT_ConfigUpBuffer(rtt_channel, "WheelIMU_Capture", rtt_buffer,
                                      sizeof(rtt_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP) < 0) return;
        rtt_ready = 1;
    }
    if (phase == IDLE) {
        uint32_t token = wheel_imu_capture_request;
        if (!token || !config) return;
        Arm(token, config, duration_us);
        return;
    }
    if (phase == RECORDING) {
        uint32_t key = Lock();
        DWT_ClockDebug clock;
        uint32_t now = (uint32_t)DWT_GetTimeline_us();
        DWT_GetClockDebug(&clock);
        (void)InWindow(now, clock.generation);
        __set_PRIMASK(key);
    }
    if (phase == FROZEN) Export(rtt_channel);
}

#endif
