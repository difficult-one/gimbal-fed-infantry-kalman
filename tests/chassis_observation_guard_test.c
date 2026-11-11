#include "chassis_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static ChassisMotionInput in;
static ChassisMotionState state;
static ChassisMotionObservationState obs;
static ChassisMotionConfig config;
static float command_v=.2f;
static unsigned wheel_period=1, imu_period=1, sample_counter;
static void step(float v,float ax,float ay)
{
    in.time_us+=5000; sample_counter++;
    if (sample_counter%imu_period==0) { in.imu_time_us=in.time_us; in.imu_seq++; }
    in.pose_valid=in.accel_valid=in.imu_center_valid=1; in.wheel_online_mask=15;
    in.imu_q[0]=1; in.imu_center_n[0]=ax; in.imu_center_n[1]=ay;
    float twist[3]={v,0,0}, wheels[4], output[4], user[2]={command_v,0};
    ChassisMotionForward(twist,wheels);
    for (unsigned i=0;i<4;++i) {
        if (sample_counter%wheel_period==0) {
            in.wheel_seq[i]++; in.wheel_time_us[i]=in.time_us;
            in.motor_dps[i]=wheels[i]/(.01745329252f*config.wheels.radius_m[i]/config.wheels.gear_ratio[i]);
        }
    }
    ChassisMotionUpdate(&in); ChassisMotionControl(user,0,1,0,output);
    ChassisMotionGetState(&state); ChassisMotionGetObservationState(&obs);
    assert(isfinite(state.x[0]) && isfinite(state.x[2]));
    if (!state.estimate_valid) assert(state.cmd_g[0]==state.ref_g[0] && state.cmd_g[1]==state.ref_g[1]);
}
int main(void)
{
    ChassisMotionConfigDefault(&config,CK_OMNI_X);
    config.initial_mode=MC_ACTIVE_FIXED;
    assert(ChassisMotionConfigure(&config));
    /* Replay the measured tail's constant acceleration bias with zero wheels. */
    for (unsigned i=0;i<300;++i) {
        step(0,.23758f,-.12992f);
        assert(!state.estimate_valid && !state.accel_used);
        assert(hypotf(state.x[0],state.x[2])<1e-6f);
    }
    assert(obs.gate==2 && obs.rejected_windows>=5 && state.exit_reason==MC_EXIT_OBSERVATION);
    for (unsigned i=0;i<500;++i) step(.1f,0,0);
    assert(state.estimate_valid && state.dv_g[0]>.005f);
    for (unsigned i=0;i<150;++i) step(.1f,.23758f,-.12992f);
    assert(!state.estimate_valid && !state.accel_used && obs.gate==2);
    /* Healthy data must refill an entire consistency window and KF recovery. */
    for (unsigned i=0;i<200;++i) step(.1f,0,0);
    assert(state.estimate_valid);
    /* A clock repair with an otherwise normal 5ms dt still invalidates history. */
    in.clock_generation++;
    step(.1f,0,0); assert(!state.estimate_valid && state.exit_reason==MC_EXIT_CLOCK);
    in.imu_recovering=1;
    for (unsigned i=0;i<200;++i) { step(.1f,0,0); assert(!state.estimate_valid && !state.accel_used); }
    in.imu_recovering=0;
    for (unsigned i=0;i<200;++i) step(.1f,0,0);
    assert(state.estimate_valid);
    /* Genuine matching acceleration at zero command is preserved, not zeroed. */
    assert(ChassisMotionConfigure(&config)); memset(&in,0,sizeof(in));
    command_v=0;
    for (unsigned i=0;i<400;++i) step(i*.005f,1,0);
    assert(state.estimate_valid && obs.gate==1 && fabsf(state.x[1]-1)<.01f);
    assert(fabsf(state.x[0]-1.995f)<.01f);
    /* Both source observations are ambiguous during slip: withdraw permission. */
    for (unsigned i=0;i<150;++i) step(1.995f,1,0);
    assert(!state.estimate_valid && obs.gate==2);
    for (unsigned asynchronous=0;asynchronous<2;++asynchronous) {
        assert(ChassisMotionConfigure(&config)); memset(&in,0,sizeof(in));
        wheel_period=asynchronous ? 1 : 2; imu_period=asynchronous ? 2 : 1;
        for (unsigned i=0;i<600;++i) step(.1f,0,0);
        assert(state.estimate_valid && obs.gate==1 && obs.control_ready);
    }
    config.log_decimation=1;
    assert(ChassisMotionConfigure(&config)); memset(&in,0,sizeof(in));
    wheel_period=imu_period=1;
    in.attitude_chi_ratio=.123f; in.imu_gravity_b[2]=9.81f;
    step(0,0,0); ChassisMotionRecord();
    in.attitude_chi_ratio=.987f; in.imu_gravity_b[2]=8;
    step(0,0,0);
    assert(ChassisMotionPopDiagnosticLog(&state,NULL,&obs));
    assert(state.seq==1 && fabsf(obs.attitude_chi_ratio-.123f)<1e-6f && obs.gravity_b[2]==9.81f);
    puts("PASS: measured bias rejection, active withdrawal/recovery, clock epochs, real acceleration, ambiguous slip");
    return 0;
}
