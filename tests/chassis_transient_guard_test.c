/* Independent velocity/acceleration trajectories: no user-command stop inference. */
#include "chassis_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static ChassisMotionConfig config;
static ChassisMotionInput in;
static ChassisMotionState state;
static ChassisMotionObservationState obs;
static float ux,uy;

static void step(float v,float a)
{
    in.time_us+=5000; in.imu_time_us=in.time_us; in.imu_seq++;
    in.pose_valid=in.accel_valid=in.imu_center_valid=1;
    in.wheel_online_mask=15; in.imu_q[0]=1;
    in.imu_center_n[0]=a*ux; in.imu_center_n[1]=a*uy;
    float twist[3]={v*ux,v*uy,0}, wheels[4], motor[4], zero[2]={0};
    ChassisMotionForward(twist,wheels);
    for(unsigned i=0;i<4;++i) {
        in.wheel_time_us[i]=in.time_us; in.wheel_seq[i]++;
        in.motor_dps[i]=wheels[i]/(.01745329252f*config.wheels.radius_m[i]/config.wheels.gear_ratio[i]);
    }
    ChassisMotionUpdate(&in);
    ChassisMotionControl(zero,0,1,0,motor);
    ChassisMotionGetState(&state); ChassisMotionGetObservationState(&obs);
    if(!state.estimate_valid) assert(state.dv_g[0]==0 && state.dv_g[1]==0);
}
static void reset(float angle,unsigned offset)
{
    assert(ChassisMotionConfigure(&config)); memset(&in,0,sizeof(in));
    ux=cosf(angle); uy=sinf(angle);
    for(unsigned k=0;k<200+offset;++k) step(.25f,0);
    assert(state.estimate_valid);
}
int main(void)
{
    ChassisMotionConfigDefault(&config,CK_OMNI_X);
    config.initial_mode=MC_ACTIVE_FIXED;
    for(unsigned axis=0;axis<4;++axis) for(unsigned phase=0;phase<5;++phase) {
        reset(axis*1.57079632679f,phase*9);
        unsigned first_reject=1000;
        for(unsigned k=1;k<=40;++k) {
            float t=k*.005f, v=fmaxf(0,.25f-t*(.25f/.15f));
            /* Braking observation overshoots, as in A; vehicle never reverses. */
            float a=t<=.1f ? -3.7f : t<=.15f ? .4f : 0;
            step(v,a);
            if(obs.gate==2 && first_reject==1000) first_reject=k;
            float error=hypotf(state.x[0]-v*ux,state.x[2]-v*uy);
            assert(!state.estimate_valid || error<=config.velocity_innovation_max+.0001f);
            assert(!state.estimate_valid || state.x[0]*ux+state.x[2]*uy>-.02f);
        }
        assert(first_reject<=10); /* Detect in <=50ms, independent of 250ms phase. */
        for(unsigned k=0;k<160;++k) step(.13f,0);
        assert(state.estimate_valid && fabsf(state.x[0]*ux+state.x[2]*uy-.13f)<.005f);
        /* Matching real dynamics, despite zero command, must retain fusion. */
        reset(axis*1.57079632679f,phase*9);
        for(unsigned k=1;k<=30;++k) {
            step(.25f-k*.005f*(.25f/.15f),-.25f/.15f);
            assert(state.accel_used && state.estimate_valid && obs.gate==1);
        }
    }
    /* Large periodic spin observation cannot be hidden by a later cancellation. */
    reset(0,1);
    for(unsigned k=1;k<=30;++k) step(.25f,.95f*sinf(k*.005f*8.16f));
    assert(obs.gate==2 && !state.estimate_valid);
    puts("PASS: braking mismatch withdrawn <=50ms in four directions/window phases; no valid reversal, real acceleration/coasting preserved, periodic residual rejected");
    return 0;
}
