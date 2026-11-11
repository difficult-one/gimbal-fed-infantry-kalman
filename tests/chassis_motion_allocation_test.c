/* Quantify high-spin translation from independent wheel algebra, without KF aid. */
#include "chassis_setup.h"
#include "config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t tick;
static float motor[4];
static ChassisMotionState state;

static void Step(const float user[2], float spin, float theta)
{
    ChassisMotionInput in={0};
    in.time_us=++tick*5000u; in.imu_time_us=in.time_us; in.imu_seq=tick;
    in.theta_bg=theta; in.wheel_online_mask=15;
    /* An unavailable IMU withdraws active fusion, but base allocation still runs. */
    for (unsigned i=0;i<4;++i) {
        in.wheel_time_us[i]=in.time_us; in.wheel_seq[i]=tick;
    }
    ChassisMotionUpdate(&in);
    ChassisMotionControl(user,spin,1,MC_ROTATE_CONTROL_KEY,motor);
    ChassisMotionGetState(&state);
    assert(!state.estimate_valid && state.mode==MC_SHADOW);
    for (unsigned i=0;i<4;++i) assert(fabsf(motor[i])<=g_chassis_motion_config.wheels.max_motor_dps[0]+.02f);
}

static void Decode(const float motor_dps[4], float theta, float g[2], float *omega)
{
    float w[4];
    for (unsigned i=0;i<4;++i) w[i]=motor_dps[i]*g_chassis_config.mps_per_motor_dps;
    float bx=(-w[0]+w[1]-w[2]+w[3])/(4*.7071067812f);
    float by=(w[0]+w[1]-w[2]-w[3])/(4*.7071067812f);
    g[0]=cosf(theta)*bx-sinf(theta)*by;
    g[1]=sinf(theta)*bx+cosf(theta)*by;
    *omega=(w[0]+w[1]+w[2]+w[3])/(4*g_chassis_motion_config.wheels.rows[0][2]);
}

int main(void)
{
    ChassisMotionInit();
    ChassisMotionConfig config;
    ChassisMotionGetConfig(&config);
    assert(config.rotation_priority==g_chassis_motion_config.rotation_priority);
    const float requested_spin=fminf(g_chassis_config.legacy_spin_command*g_chassis_config.legacy_omega_scale,g_chassis_motion_config.max_omega);
    const float zero[2]={0};
    float minimum[2]={10,10}, maximum[2]={0,0};
    /* The same motor ceiling, request and independent wheel inverse compare policies. */
    for (unsigned priority=0;priority<2;++priority) {
        config.rotation_priority=priority;
        assert(ChassisMotionConfigure(&config));
        for (unsigned sign=0;sign<2;++sign) {
            float spin=(sign ? -1:1)*requested_spin;
            for (unsigned angle=0;angle<32;++angle) {
                float theta=angle*(2*3.14159265359f/32);
                const float user[2]={1,0};
                /* Two seconds also settle the +spin -> -spin slew transition. */
                for (unsigned k=0;k<400;++k) Step(user,spin,theta);
                float g[2], omega;
                Decode(motor,theta,g,&omega);
                assert(fabsf(g[1])<2e-6f && g[0]>0);
                assert(fabsf(g[0]-state.cmd_g[0])<2e-6f);
                assert(fabsf(omega-state.omega_ref)<2e-5f);
                minimum[priority]=fminf(minimum[priority],g[0]);
                maximum[priority]=fmaxf(maximum[priority],g[0]);
                /* Independent four-wheel envelope: a unit X reference has
                 * max wheel projection (|cos theta|+|sin theta|)/sqrt(2).
                 * Compare exact policy predictions for the calibrated wheel
                 * radius and spin, rather than historical 60 mm/6000 ranges. */
                const float projection=(fabsf(cosf(theta))+fabsf(sinf(theta)))*.7071067812f;
                const float capacity=g_chassis_motion_config.wheels.max_motor_dps[0]*g_chassis_config.mps_per_motor_dps;
                const float rotation=g_chassis_motion_config.wheels.rows[0][2]*fabsf(spin);
                if (priority) {
                    const float ideal=fminf(1,(capacity-rotation)/projection);
                    const float expected=ideal<1 ? ideal*.999f : ideal; // Production quantization margin.
                    assert(fabsf(g[0]-expected)<3e-6f);
                    assert(fabsf(omega-spin)<2e-5f);
                } else {
                    const float expected=fminf(1,capacity/(rotation+projection));
                    assert(fabsf(g[0]-expected)<3e-6f);
                    assert(fabsf(omega/spin-g[0])<3e-6f);
                }
            }
            /* The deployed pure-spin command adds a signed wheel trim after allocation. */
            for (unsigned k=0;k<300;++k) Step(zero,spin,0);
            float nominal[4], g[2], omega;
            for (unsigned i=0;i<4;++i)
                nominal[i]=motor[i]-copysignf(1.0f,spin)*config.spin_wheel_trim_dps[i];
            Decode(nominal,0,g,&omega);
            assert(hypotf(g[0],g[1])<2e-6f && fabsf(omega-spin)<2e-5f);
        }
    }
    assert(minimum[0]>minimum[1]+.001f);
    ChassisMotionControl(zero,requested_spin,0,MC_ROTATE_CONTROL_KEY,motor);
    for (unsigned i=0;i<4;++i) assert(motor[i]==0);
    printf("PASS: high-spin translation at 32 relative headings and both spins: rotation priority %.6f..%.6f m/s -> proportional %.6f..%.6f m/s; wheel ceiling, direction, pure-spin restoration and immediate stop\n",
           minimum[1],maximum[1],minimum[0],maximum[0]);
    return 0;
}
