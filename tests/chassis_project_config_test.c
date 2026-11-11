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
int main(void)
{
    ChassisMotionInit();
    ChassisMotionConfig c; ChassisMotionGetConfig(&c);
    assert(customized==1 && c.max_velocity==.75f && c.initial_mode==MC_SHADOW);
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
#endif
    puts("PASS: project board/drive selection, 152 mm wheel units, spin trim and user hook");
}
