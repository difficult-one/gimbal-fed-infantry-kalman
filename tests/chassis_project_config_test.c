#include "chassis_setup.h"
#include "config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static unsigned customized;
void ChassisMotionCustomizeConfig(ChassisMotionConfig *config)
{
    customized++; config->max_velocity=.75f;
}
#if !defined(CHASSIS_DRIVE_SWERVE) && !defined(CHASSIS_DRIVE_MECANUM)
static void AdvanceSample(ChassisMotionInput *input)
{
    input->time_us+=5000u;
    input->imu_time_us=input->time_us;
    input->imu_seq++;
    for (unsigned i=0;i<4;++i) {
        input->wheel_time_us[i]=input->time_us;
        input->wheel_seq[i]++;
    }
    ChassisMotionUpdate(input);
}

static void TestDirectTranslation(const ChassisMotionConfig *config)
{
    assert(ChassisMotionConfigure(config));
    ChassisMotionInput input={0};
    input.pose_valid=1u;
    input.wheel_online_mask=15u;
    float motors[4], zero[2]={0};
    AdvanceSample(&input);
    ChassisMotionControl(zero,0.0f,1u,0u,motors);

    /* Both directions exceed the historical chassis limit, while remaining
     * feasible for the calibrated wheels. Each request must apply in one tick. */
    const float requests[3][2]={{1.4f,.6f},{-1.4f,-.6f},{0.0f,0.0f}};
    for (unsigned step=0;step<3;++step) {
        AdvanceSample(&input);
        ChassisMotionControl(requests[step],0.0f,1u,0u,motors);
        ChassisMotionState state;
        ChassisMotionGetState(&state);
        for (unsigned axis=0;axis<2;++axis) {
            assert(fabsf(state.ref_g[axis]-requests[step][axis])<1e-6f);
            assert(fabsf(state.cmd_g[axis]-requests[step][axis])<1e-6f);
        }
        /* Independently decode the four calibrated X-omni wheel velocities. */
        float wheel[4];
        for (unsigned i=0;i<4;++i)
            wheel[i]=motors[i]*g_chassis_config.mps_per_motor_dps;
        float vx=(-wheel[0]+wheel[1]-wheel[2]+wheel[3])/(4.0f*.7071067812f);
        float vy=(wheel[0]+wheel[1]-wheel[2]-wheel[3])/(4.0f*.7071067812f);
        assert(fabsf(vx-requests[step][0])<1e-6f);
        assert(fabsf(vy-requests[step][1])<1e-6f);
    }

    AdvanceSample(&input);
    ChassisMotionControl(zero,1.0f,1u,0u,motors);
    ChassisMotionState state;
    ChassisMotionGetState(&state);
    assert(fabsf(state.omega_ref-config->omega_slew*.005f)<1e-6f);

    const float excessive[2]={20.0f,10.0f};
    AdvanceSample(&input);
    ChassisMotionControl(excessive,0.0f,1u,0u,motors);
    ChassisMotionGetState(&state);
    assert(state.limit_flags&MC_LIMIT_REF);
    assert(state.ref_g[0]>0.0f && state.ref_g[0]<excessive[0]);
    assert(fabsf(state.ref_g[0]/state.ref_g[1]-2.0f)<1e-6f);
    for (unsigned i=0;i<4;++i)
        assert(fabsf(motors[i])<=config->wheels.max_motor_dps[i]+.02f);

    ChassisMotionControl(excessive,0.0f,0u,0u,motors);
    for (unsigned i=0;i<4;++i) assert(motors[i]==0.0f);
    AdvanceSample(&input);
    const float invalid[2]={NAN,0.0f};
    ChassisMotionControl(invalid,0.0f,1u,0u,motors);
    for (unsigned i=0;i<4;++i) assert(motors[i]==0.0f);

    /* Existing portable profiles may still request a bounded, ramped reference. */
    ChassisMotionConfig bounded=*config;
    bounded.direct_translation=0u;
    assert(ChassisMotionConfigure(&bounded));
    input=(ChassisMotionInput){0};
    input.pose_valid=1u;
    input.wheel_online_mask=15u;
    AdvanceSample(&input);
    ChassisMotionControl(zero,0.0f,1u,0u,motors);
    AdvanceSample(&input);
    ChassisMotionControl(requests[0],0.0f,1u,0u,motors);
    ChassisMotionGetState(&state);
    assert(fabsf(hypotf(state.ref_g[0],state.ref_g[1])-bounded.velocity_slew*.005f)<1e-6f);
    for (unsigned n=0;n<300;++n) {
        AdvanceSample(&input);
        ChassisMotionControl(requests[0],0.0f,1u,0u,motors);
    }
    ChassisMotionGetState(&state);
    assert(fabsf(hypotf(state.ref_g[0],state.ref_g[1])-bounded.max_velocity)<1e-6f);
}
#endif
int main(void)
{
    ChassisMotionInit();
    ChassisMotionConfig c; ChassisMotionGetConfig(&c);
    assert(customized==1 && c.max_velocity==.75f && c.initial_mode==MC_SHADOW);
    assert(c.direct_translation==1u);
    ChassisMotionConfig portable;
    ChassisMotionConfigDefault(&portable,CK_OMNI_X);
    assert(portable.direct_translation==0u);
#ifdef CHASSIS_BOARD
    assert(!c.imu_on_gimbal && !c.imu_follows_pitch && c.use_center_accel);
#else
    assert(c.imu_on_gimbal && c.imu_follows_pitch && c.use_center_accel);
#endif
#if defined(CHASSIS_DRIVE_SWERVE)
    assert(c.wheels.type==CK_SWERVE && c.algebra_y_sign==1);
#elif defined(CHASSIS_DRIVE_MECANUM)
    assert(c.wheels.type==CK_MECANUM_X && c.algebra_y_sign==1);
#else
    assert(c.wheels.type==CK_CUSTOM && c.algebra_y_sign==-1);
#endif
    assert(ChassisMotionConfigure(&c));
    ChassisKinematics model;
    assert(ChassisKinematicsInit(&model,&c.wheels));
    /* Independent physical reference: 1000 output-wheel deg/s on a 152 mm
     * diameter wheel; rotor feedback is 19000 deg/s with a 19:1 gearbox. */
    for (unsigned i=0;i<4;++i)
        assert(fabsf(19000.0f*ChassisKinematicsMpsPerDps(&model,i)-1.32645023f)<2e-6f);
#if !defined(CHASSIS_DRIVE_SWERVE) && !defined(CHASSIS_DRIVE_MECANUM)
    ChassisMotionInput in={0};
    in.pose_valid=1; in.wheel_online_mask=15;
    float refs[4], zero[2]={0};
    for (unsigned n=0;n<300;++n) {
        in.time_us+=5000; in.imu_time_us=in.time_us; in.imu_seq++;
        for (unsigned i=0;i<4;++i) { in.wheel_time_us[i]=in.time_us; in.wheel_seq[i]++; }
        ChassisMotionUpdate(&in);
        ChassisMotionControl(zero,4000.0f*g_chassis_config.legacy_omega_scale,1,MC_ROTATE_CONTROL_KEY,refs);
    }
    ChassisMotionState state; ChassisMotionGetState(&state);
    assert(fabsf(state.omega_ref-6.8927054f)<2e-5f);
    /* Legacy conversion and inverse motor conversion cancel the radius in
     * nominal motor speed. The one-board omni adapter adds its measured trim. */
    for (unsigned i=0;i<4;++i) {
        float expected=22689.2803f;
#if defined(ONE_BOARD) && defined(CHASSIS_DRIVE_OMNI_X)
        const float trim[4]={66.0f, -107.0f, 29.0f, 12.0f};
        expected+=trim[i];
#endif
        assert(fabsf(refs[i]-expected)<.03f);
    }
    const float translation[2]={.01f,0.0f};
    for (unsigned n=0;n<4;++n) {
        in.time_us+=5000; in.imu_time_us=in.time_us; in.imu_seq++;
        for (unsigned i=0;i<4;++i) { in.wheel_time_us[i]=in.time_us; in.wheel_seq[i]++; }
        ChassisMotionUpdate(&in);
        ChassisMotionControl(translation,4000.0f*g_chassis_config.legacy_omega_scale,1,MC_ROTATE_CONTROL_KEY,refs);
    }
    ChassisMotionGetState(&state);
    for (unsigned i=0;i<4;++i) assert(fabsf(refs[i]-state.wheel_req_dps[i])<1e-5f);
    for (unsigned n=0;n<300;++n) {
        in.time_us+=5000; in.imu_time_us=in.time_us; in.imu_seq++;
        for (unsigned i=0;i<4;++i) { in.wheel_time_us[i]=in.time_us; in.wheel_seq[i]++; }
        ChassisMotionUpdate(&in);
        ChassisMotionControl(zero,6000.0f*g_chassis_config.legacy_omega_scale,1,MC_ROTATE_CONTROL_KEY,refs);
    }
    ChassisMotionGetState(&state);
    assert(fabsf(state.omega_ref-9.0f)<2e-5f); // 6000 now exceeds the SI cap.
    TestDirectTranslation(&c);
#endif
    puts("PASS: project board/drive selection, 152 mm wheel units, spin trim, direct translation and bounded portable profile");
}
