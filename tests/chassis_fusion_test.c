/* 生产配置的融合机制回归：注入已知残差，检验观测权重/真实平移/掉线恢复。
 * 本文件只检验 KF 消费端；机械旋转与传感器采样由独立刚体用例覆盖。 */
#include "chassis_setup.h"
#include "config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static ChassisMotionInput in;
static ChassisMotionState state;
static void FrameWithYaw(float velocity,float accel,unsigned gyro_axis,float rate,float chassis_rate)
{
    in.time_us+=5000; in.imu_time_us=in.time_us; in.imu_seq++;
    in.pose_valid=in.accel_valid=in.imu_center_valid=1;
    in.wheel_online_mask=15; in.imu_q[0]=1;
    memset(in.imu_gyro_rad,0,sizeof(in.imu_gyro_rad));
    in.imu_gyro_rad[gyro_axis]=rate;
    in.imu_center_n[0]=accel;
    float twist[3]={velocity,0,chassis_rate}, wheels[4];
    ChassisMotionForward(twist,wheels);
    for(unsigned i=0;i<4;++i) {
        in.wheel_seq[i]++; in.wheel_time_us[i]=in.time_us;
        in.motor_dps[i]=wheels[i]/g_chassis_config.mps_per_motor_dps;
    }
}
static void Frame(float velocity,float accel,unsigned gyro_axis,float rate)
{
    FrameWithYaw(velocity,accel,gyro_axis,rate,0.0f);
}
static void Update(void)
{
    ChassisMotionUpdate(&in);
    const float user[2]={.2f,0}; float motor[4];
    ChassisMotionControl(user,0,1,1,motor);
    ChassisMotionGetState(&state);
    assert(isfinite(state.x[0]) && isfinite(state.x[1]));
    /* 此组用例显式选择估计模式，检查融合与电机输出解耦。 */
    assert(state.mode==MC_ESTIMATE_ONLY);
    assert(state.cmd_g[0]==state.ref_g[0] && state.dv_g[0]==0);
}
static void Reset(void)
{
    ChassisMotionSetMode(MC_BASELINE);
    Frame(0,0,2,0); ChassisMotionUpdate(&in);
    ChassisMotionSetMode(MC_ESTIMATE_ONLY);
    for(unsigned k=0;k<100;++k) { Frame(0,0,2,0); Update(); }
}
static float Residual(unsigned axis,float rate)
{
    Reset(); double error2=0;
    for(unsigned k=0;k<2000;++k) {
        float t=k*.005f;
        /* .24/sqrt(2)≈.17 m/s²，与第十二次 YAW 残差 RMS 同量级。 */
        Frame(0,.24f*sinf(2*t)+.03f,axis,rate); Update();
        assert(state.accel_used && state.observation_mask==15 && state.estimate_valid);
        if(k>500) error2+=state.x[0]*state.x[0];
    }
    return (float)sqrt(error2/1499);
}
static float SpinVelocityRms(float chassis_rate)
{
    Reset(); double error2=0;
    for(unsigned k=0;k<1200;++k) {
        float t=k*.005f;
        /* 模拟云台朝向稳定、底盘自旋时残留的低频中心加速度误差。 */
        FrameWithYaw(.15f,.24f*sinf(2*t)+.03f,2,0,chassis_rate);
        ChassisMotionUpdate(&in);
        ChassisMotionGetState(&state);
        assert(state.estimate_valid && state.accel_used);
        if(k>200) error2+=(state.x[0]-.15f)*(state.x[0]-.15f);
    }
    return (float)sqrt(error2/999);
}
static void DefaultShadowAndExplicitActive(void)
{
    const float user[2]={.2f,0}; float motor[4];
    /* 上电保持影子模式：即使估计恢复，四轮也只能执行基础参考。 */
    const float signs[4]={-1,1,-1,1};
    for(unsigned k=0;k<300;++k) {
        Frame(.15f,0,2,0); ChassisMotionUpdate(&in);
        ChassisMotionControl(user,0,1,1,motor); ChassisMotionGetState(&state);
        assert(state.mode==MC_SHADOW && state.cmd_g[0]==state.ref_g[0]);
        assert(state.cmd_g[1]==state.ref_g[1]);
    }
    assert(state.estimate_valid);
    for(unsigned i=0;i<4;++i)
        assert(fabsf(motor[i]-signs[i]*.7071067812f*.2f/g_chassis_config.mps_per_motor_dps)<.01f);
    /* 主动模式仅由用例显式开启，验证其独立门控/退出机制。 */
    ChassisMotionSetMode(MC_ACTIVE_FIXED);
    /* 显式主动模式；恢复完成前不追加，之后 .20-.15 的误差应追加约 .005。 */
    for(unsigned k=0;k<400;++k) {
        Frame(.15f,0,2,0); ChassisMotionUpdate(&in);
        ChassisMotionControl(user,0,1,1,motor);
        ChassisMotionGetState(&state);
        if(!state.estimate_valid)
            assert(state.cmd_g[0]==state.ref_g[0] && state.dv_g[0]==0);
    }
    assert(state.mode==MC_ACTIVE_FIXED && state.estimate_valid && state.accel_used);
    assert(fabsf(state.kp-.1f)<1e-6f && state.dv_g[0]>.004f && state.dv_g[0]<.006f);
    /* 独立旧 J 手算，直接检查四轮确实含补偿，避免只验证状态字段。 */
    for(unsigned i=0;i<4;++i)
        assert(fabsf(motor[i]-signs[i]*.7071067812f*.205f/g_chassis_config.mps_per_motor_dps)<.01f);
    assert(fabsf(state.cmd_g[0]-.205f)<1e-6f);
    /* 加速度失效立即撤销补偿；联合有效观测未恢复满20次前不得追加。 */
    Frame(.15f,0,2,0); in.accel_valid=0; ChassisMotionUpdate(&in);
    ChassisMotionControl(user,0,1,1,motor); ChassisMotionGetState(&state);
    assert(!state.estimate_valid && state.dv_g[0]==0 && state.cmd_g[0]==state.ref_g[0]);
    const unsigned recover=g_chassis_motion_config.recover_samples+(unsigned)ceilf(g_chassis_motion_config.accel_consistency_window_s/.005f)+1u;
    for(unsigned k=0;k<recover;++k) {
        Frame(.15f,0,2,0); ChassisMotionUpdate(&in);
        ChassisMotionControl(user,0,1,1,motor); ChassisMotionGetState(&state);
        if(k<g_chassis_motion_config.recover_samples-1u)
            assert(!state.estimate_valid && state.cmd_g[0]==state.ref_g[0]);
    }
    assert(state.estimate_valid && state.dv_g[0]==0); /* Estimation precedes control permission. */
    for(unsigned k=0;k<400;++k) {
        Frame(.15f,0,2,0); ChassisMotionUpdate(&in);
        ChassisMotionControl(user,0,1,1,motor); ChassisMotionGetState(&state);
    }
    assert(state.dv_g[0]>0);
    /* 影子与纯估计都立即去掉旧补偿；切回主动时按斜坡重新建立。 */
    for(unsigned i=0;i<2;++i) {
        ChassisMotionSetMode(i ? MC_SHADOW : MC_ESTIMATE_ONLY);
        Frame(.15f,0,2,0); ChassisMotionUpdate(&in);
        ChassisMotionControl(user,0,1,1,motor); ChassisMotionGetState(&state);
        assert(state.cmd_g[0]==state.ref_g[0] && state.cmd_g[1]==state.ref_g[1]);
        for(unsigned j=0;j<4;++j)
            assert(fabsf(motor[j]-signs[j]*.7071067812f*.2f/g_chassis_config.mps_per_motor_dps)<.01f);
    }
    ChassisMotionSetMode(MC_ACTIVE_FIXED);
    Frame(.15f,0,2,0); ChassisMotionUpdate(&in);
    ChassisMotionControl(user,0,1,1,motor); ChassisMotionGetState(&state);
    assert(state.dv_g[0]==0); /* Mode change requires a new stable control window. */
    /* 输出禁用优先于所有融合/参考/模式，四轮即刻归零。 */
    ChassisMotionControl(user,0,0,1,motor); ChassisMotionGetState(&state);
    for(unsigned i=0;i<4;++i) assert(motor[i]==0);
    assert(state.cmd_g[0]==0 && state.cmd_g[1]==0 && state.dv_g[0]==0 && state.dv_g[1]==0);
}
static void FasterSpinRequest(void)
{
    Reset();
    const float zero[2]={0,0}; float motor[4];
    const float request=g_chassis_config.legacy_spin_command*g_chassis_config.legacy_omega_scale;
    const float target=fminf(request,g_chassis_motion_config.max_omega);
    /* Independent legacy rotation column, then the physical SI speed cap.
     * The deployed comparison setting is 4000; 6000 no longer fits under 9 rad/s. */
    const float legacy_dps=g_chassis_config.legacy_spin_command*(g_chassis_config.wheel_base_mm+g_chassis_config.track_width_mm)*.5f*(3.14159265359f/180.0f);
    const float expected_dps=fminf(legacy_dps,g_chassis_motion_config.max_omega*g_chassis_motion_config.wheels.rows[0][2]/g_chassis_config.mps_per_motor_dps);
    for(unsigned direction=0;direction<2;++direction) {
        float sign=direction ? -1.0f : 1.0f;
        for(unsigned k=0;k<400;++k) {
            Frame(0,0,2,0); ChassisMotionUpdate(&in);
            ChassisMotionControl(zero,sign*request,1,2,motor);
        }
        ChassisMotionGetState(&state);
        assert(fabsf(state.omega_ref-sign*target)<1e-5f && state.limit_flags==0);
        for(unsigned i=0;i<4;++i)
            assert(fabsf(motor[i]-sign*expected_dps)<.05f && fabsf(motor[i])<=g_chassis_motion_config.wheels.max_motor_dps[0]);
    }
    /* 更高自旋仍需限制混合运动的四轮请求；用户加速不应绕过既有可行性处理。 */
    const float mixed[2]={.8f,.6f};
    for(unsigned k=0;k<400;++k) {
        Frame(0,0,2,0); ChassisMotionUpdate(&in);
        ChassisMotionControl(mixed,request,1,2,motor);
    }
    ChassisMotionGetState(&state);
    assert(state.limit_flags&MC_LIMIT_REF);
    for(unsigned i=0;i<4;++i) assert(fabsf(motor[i])<=g_chassis_motion_config.wheels.max_motor_dps[0]+.01f);
    ChassisMotionControl(zero,0,0,0,motor);
    for(unsigned i=0;i<4;++i) assert(motor[i]==0);
    printf("PASS: configured spin reaches %.6f rad/s and %.2f motor deg/s in both directions; mixed-motion cap and immediate stop\n",target,expected_dps);
}
int main(void)
{
    assert(g_chassis_motion_config.accel_enabled == 1u);
    assert(g_chassis_motion_config.initial_mode == MC_SHADOW);
    ChassisMotionInit();
    DefaultShadowAndExplicitActive();
    FasterSpinRequest();
    /* These cases isolate KF weighting with intentionally contradictory input.
     * Default conflict rejection is exercised by chassis_observation_guard_test. */
    ChassisMotionConfig weighting;
    ChassisMotionGetConfig(&weighting); weighting.accel_consistency_max_dv=1.0f;
    weighting.velocity_innovation_max=1.0f; /* Isolate weights; guard has independent tests. */
    assert(ChassisMotionConfigure(&weighting));
    float base=Residual(2,0), yaw=Residual(2,2), pitch=Residual(1,2);
    assert(yaw<base*.8f && fabsf(yaw-pitch)<1e-6f && yaw<.02f);
    float stationary_spin=SpinVelocityRms(0.0f);
    float chassis_spin=SpinVelocityRms(g_chassis_config.legacy_spin_command*g_chassis_config.legacy_omega_scale);
    assert(chassis_spin<stationary_spin*.5f && chassis_spin<.02f);
    Reset(); float max_error=0, a_peak=0;
    for(unsigned k=0;k<4000;++k) {
        float t=k*.005f;
        Frame(.3f*sinf(.5f*t),.15f*cosf(.5f*t),2,0); Update();
        if(k>500) {
            max_error=fmaxf(max_error,fabsf(state.x[0]-.3f*sinf(.5f*t)));
            a_peak=fmaxf(a_peak,fabsf(state.x[1]));
        }
    }
    assert(max_error<.01f && a_peak>.12f);
    Frame(0,0,2,0); in.imu_gyro_rad[1]=NAN; Update();
    assert(!state.accel_used && !state.accel_center_valid && !state.estimate_valid);
    const unsigned recover=g_chassis_motion_config.recover_samples+(unsigned)ceilf(g_chassis_motion_config.accel_consistency_window_s/.005f)+2u;
    for(unsigned k=0;k<recover;++k) { Frame(0,0,2,0); Update(); }
    assert(state.estimate_valid);
    Frame(0,0,2,0); in.accel_valid=0; Update();
    assert(!state.accel_used && !state.estimate_valid);
    for(unsigned k=0;k<g_chassis_motion_config.recover_samples-1u;++k) { Frame(0,0,2,0); Update(); assert(!state.estimate_valid); }
    for(unsigned k=0;k<recover;++k) { Frame(0,0,2,0); Update(); }
    Frame(0,0,2,0); Update(); assert(state.estimate_valid);
    printf("PASS: default shadow preserves base motor references; explicit active mode, startup/dropout recovery, mode switching/slew and disabled output; rotation-weighted residual velocity RMS %.6f -> %.6f m/s; pitch %.6f; translation max error %.6f; accel peak %.6f; NaN/dropout recovery; estimate-only motor references\n",
        base,yaw,pitch,max_error,a_peak);
    printf("PASS: chassis spin rate lowers accelerometer weight; translation velocity RMS %.6f -> %.6f m/s\n",
        stationary_spin,chassis_spin);
    return 0;
}
