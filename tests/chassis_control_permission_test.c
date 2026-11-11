/* Observation recovery and actuator permission are independent. Synthetic truth
 * drives the wheels and IMU separately; zero user input never edits that truth. */
#include "chassis_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static ChassisMotionConfig cfg;
static ChassisMotionInput in;
static ChassisMotionState state;
static ChassisMotionObservationState obs;
static float command[2], motor[4];
static uint8_t enabled=1, key;

static void control(void)
{
    ChassisMotionControl(command,0,enabled,key,motor);
    ChassisMotionGetState(&state);
    ChassisMotionGetObservationState(&obs);
    if (!obs.control_ready || state.mode<MC_ACTIVE_FIXED) {
        assert(state.cmd_g[0]==state.ref_g[0] && state.cmd_g[1]==state.ref_g[1]);
    }
    if (!obs.control_ready) assert(state.dv_g[0]==0 && state.dv_g[1]==0);
}

static void tick(float vx,float vy,float ax,float ay,float omega)
{
    in.time_us+=5000; in.imu_time_us=in.time_us; in.imu_seq++;
    in.imu_center_n[0]=ax; in.imu_center_n[1]=ay;
    float twist[3]={vx,vy,omega}, wheel[4];
    ChassisMotionForward(twist,wheel);
    for (unsigned i=0;i<4;++i) {
        in.wheel_seq[i]++; in.wheel_time_us[i]=in.time_us;
        in.motor_dps[i]=wheel[i]/(.01745329252f*cfg.wheels.radius_m[i]/cfg.wheels.gear_ratio[i]);
    }
    ChassisMotionUpdate(&in); control();
}

static void warm(float vx,float vy)
{
    for (unsigned k=0;k<600;++k) tick(vx,vy,0,0,0);
    assert(state.estimate_valid && obs.control_ready && obs.control_blend==1);
}

static void reset(void)
{
    ChassisMotionConfigDefault(&cfg,CK_OMNI_X);
    cfg.initial_mode=MC_ACTIVE_FIXED;
    assert(ChassisMotionConfigure(&cfg));
    memset(&in,0,sizeof(in)); in.time_us=UINT32_MAX-100000u;
    in.pose_valid=in.accel_valid=in.imu_center_valid=1; in.wheel_online_mask=15;
    in.imu_q[0]=1; enabled=1; key=0; command[0]=.2f; command[1]=0;
}

int main(void)
{
    reset(); warm(0,0);
    assert(state.cmd_g[0]>.2f); /* Check compensation reaches actual wheel references. */
    float baseline=-.7071067812f*state.ref_g[0]/(.01745329252f*cfg.wheels.radius_m[0]/cfg.wheels.gear_ratio[0]);
    assert(motor[0]<baseline);
    /* Spin disagreement, then the small persistent recovery bias from the log:
     * valid may return while the control error remains below the old .04 limit. */
    for (unsigned k=0;k<200;++k) tick(0,0,.95f,0,6.575f);
    assert(!state.estimate_valid && !obs.control_ready);
    unsigned valid_but_blocked=0;
    for (unsigned k=0;k<1200;++k) {
        tick(0,0,.08f,0,0);
        if (state.estimate_valid && obs.control_velocity_error>cfg.control_velocity_max) {
            assert(obs.control_inhibit & MC_CONTROL_VELOCITY);
            assert(!obs.control_ready);
            valid_but_blocked++;
        }
    }
    assert(valid_but_blocked>400); /* A fixed one-second timer alone would fail. */
    warm(0,0);
    /* Current saturation/mismatch/non-finite feedback withdraw on this cycle. */
    in.current_request[0]=cfg.current_limit; in.current_send[0]=cfg.current_limit;
    tick(0,0,0,0,0); assert(obs.control_inhibit & MC_CONTROL_ACTUATOR);
    assert(!obs.control_ready && state.dv_g[0]==0);
    in.current_request[0]=20; in.current_send[0]=0;
    tick(0,0,0,0,0); assert(obs.control_inhibit & MC_CONTROL_ACTUATOR);
    in.current_request[0]=.9f; /* Integer truncation alone is not a fault. */
    warm(0,0);
    in.current_fb[0]=NAN;
    tick(0,0,0,0,0); assert(!obs.control_ready);
    in.current_fb[0]=0; warm(0,0);
    /* Stop withdraws immediately; estimation continues to observe real coasting. */
    enabled=0; tick(.25f,0,0,0,0);
    for (unsigned i=0;i<4;++i) assert(motor[i]==0);
    for (unsigned k=0;k<200;++k) tick(.25f,0,0,0,0);
    assert(state.estimate_valid && fabsf(state.x[0]-.25f)<.001f && !obs.control_ready);
    enabled=1; tick(.25f,0,0,0,0);
    assert(!obs.control_ready && obs.control_good_s==0);
    float good=obs.control_good_s;
    for (unsigned k=0;k<500;++k) control();
    assert(obs.control_good_s==good); /* Repeated Control cannot accelerate qualification. */
    warm(.25f,0);
    /* Released stick, real braking then reversal; both directions remain observed. */
    command[0]=0;
    float v=.25f, last_a=0;
    for (unsigned k=0;k<100;++k) {
        float a=-1;
        v+=.5f*(last_a+a)*.005f; last_a=a;
        tick(v,0,a,0,0);
        assert(state.estimate_valid && obs.control_ready);
        assert(fabsf(state.x[0]-v)<.01f);
    }
    warm(-.25f,0);
    assert(state.x[0]<-.24f);
    /* IMU loss/recovery, wheel loss, clock discontinuity and motion-mode switch. */
    in.accel_valid=0; tick(-.25f,0,0,0,0); assert(!obs.control_ready);
    in.accel_valid=1;
    unsigned first_valid=0,first_ready=0;
    for (unsigned k=1;k<=600;++k) {
        tick(-.25f,0,0,0,0);
        if (state.estimate_valid && !first_valid) first_valid=k;
        if (obs.control_ready && !first_ready) {
            first_ready=k;
            assert(obs.control_blend>0 && obs.control_blend<.011f);
            assert(state.dv_g[0]>0 && state.dv_g[0]<.001f);
        }
    }
    assert(first_valid && first_ready-first_valid>=199);
    in.wheel_online_mask=7; tick(-.25f,0,0,0,0); assert(!obs.control_ready);
    in.wheel_online_mask=15; warm(-.25f,0);
    in.clock_generation++; tick(-.25f,0,0,0,0);
    assert(!obs.control_ready && obs.control_good_s==0);
    warm(-.25f,0);
    key=1; tick(-.25f,0,0,0,0); assert(!obs.control_ready);
    warm(-.25f,0);
    /* Held samples inside their freshness budget cannot advance qualification;
     * expired samples withdraw permission, also at slower asynchronous rates. */
    for (unsigned k=0;k<5;++k) {
        in.time_us+=5000; ChassisMotionUpdate(&in); control();
    }
    assert(!obs.control_ready && (obs.control_inhibit & MC_CONTROL_SOURCE));
    warm(-.25f,0);
    in.cmd_age_us=25000; tick(-.25f,0,0,0,0); assert(!obs.control_ready);
    in.cmd_age_us=0; warm(-.25f,0);
    ChassisMotionSetMode(MC_SHADOW); tick(-.25f,0,0,0,0); warm(-.25f,0);
    assert(state.cmd_g[0]==state.ref_g[0]);
    ChassisMotionSetMode(MC_ACTIVE_FIXED); tick(-.25f,0,0,0,0); assert(!obs.control_ready);
    /* Isolate acceleration qualification below the estimator's existing gate. */
    cfg.control_velocity_max=.039f; assert(ChassisMotionConfigure(&cfg));
    warm(0,0);
    unsigned accel_blocked=0;
    for (unsigned k=0;k<500;++k) {
        tick(0,0,.12f,0,0);
        if (state.estimate_valid && (obs.control_inhibit & MC_CONTROL_ACCEL)) {
            assert(!obs.control_ready); accel_blocked++;
        }
    }
    assert(accel_blocked>200);
    cfg.control_stable_s=NAN; assert(!ChassisMotionConfigure(&cfg));
    puts("PASS: independent stable control window, residual bias, gradual entry, immediate revocation, spin exit, coasting/reversal, source/clock/actuator faults and SHADOW isolation");
    return 0;
}
