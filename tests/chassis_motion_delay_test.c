/* 生产控制器 + 独立的一阶轮速响应，不用理想瞬时轮速证明方向正确。
 * 时间常数只覆盖实测相位所支持的机制范围，不是已辨识的实车完整模型。 */
#include "chassis_setup.h"
#include "config.h"
#include "math_units.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static unsigned tick;
static float Wrap(float x) { return atan2f(sinf(x),cosf(x)); }

static void DirectionGuards(void)
{
    ChassisMotionInput in={0}; ChassisMotionState state;
    ChassisMotionDirectionState dir;
    const float user[2]={.05f,0}; float motor[4];
    in.wheel_online_mask=15;
    for(unsigned k=0;k<300;++k) {
        in.time_us=++tick*5000u; in.imu_time_us=in.time_us; in.imu_seq=tick;
        in.theta_bg=Wrap(3.1f-k*.05f); // 多次跨角度边界，真实相对速率-10rad/s。
        for(unsigned j=0;j<4;++j) { in.wheel_time_us[j]=in.time_us; in.wheel_seq[j]=tick; }
        /* INS故障不影响这条基于相对编码器的输出校正，主动融合仍不允许。 */
        in.psi_imu_n=NAN; in.pose_valid=0;
        ChassisMotionUpdate(&in);
        ChassisMotionControl(user,8,1,MC_ROTATE_CONTROL_KEY,motor);
        ChassisMotionGetState(&state); ChassisMotionGetDirectionState(&dir);
        assert(!state.estimate_valid && state.cmd_g[0]==state.ref_g[0]);
        if(k>100) {
            assert(dir.rate_valid && fabsf(dir.theta_rate_bg+10)<.0003f);
            assert(fabsf(dir.lead_rad+10*g_chassis_motion_config.phase_lead_s)<.00001f);
        }
    }
    /* 非小陀螺模式/停止/坏周期不应用提前量。 */
    ChassisMotionControl(user,8,1,2,motor); ChassisMotionGetDirectionState(&dir);
    assert(dir.lead_rad==0 && dir.theta_output_bg==in.theta_bg);
    ChassisMotionControl(user,8,0,MC_ROTATE_CONTROL_KEY,motor);
    ChassisMotionGetDirectionState(&dir); assert(dir.lead_rad==0);
    for(unsigned j=0;j<4;++j) assert(motor[j]==0);
    /* 不增加time：周期为零，控制输出和提前量必须同时撤销。 */
    ChassisMotionUpdate(&in); ChassisMotionControl(user,8,1,MC_ROTATE_CONTROL_KEY,motor);
    ChassisMotionGetDirectionState(&dir); assert(!dir.rate_valid && dir.lead_rad==0);
    for(unsigned j=0;j<4;++j) assert(motor[j]==0);
    /* 过期命令、离线、编码器跳变均不允许外推。 */
    for(unsigned k=0;k<3;++k) {
        in.time_us=++tick*5000u;
        for(unsigned j=0;j<4;++j) in.wheel_time_us[j]=in.time_us;
        in.cmd_age_us=k==0 ? 30000u:0u;
        in.wheel_online_mask=k==1 ? 7u:15u;
        if(k==2) in.theta_bg+=1;
        ChassisMotionUpdate(&in); ChassisMotionControl(user,8,1,MC_ROTATE_CONTROL_KEY,motor);
        ChassisMotionGetDirectionState(&dir); assert(!dir.rate_valid && dir.lead_rad==0);
    }
    puts("PASS: direction lead gates, wrapped encoder, invalid INS isolation, zero dt, stale/offline inputs and stop");
}

static float Run(float spin, unsigned axis, float sign, float tau)
{
    const float gamma=.43f, dt=.005f, c=.7071067812f;
    float user[2]={0}; user[axis]=sign*.05f;
    float expected[2]={cosf(gamma)*user[0]+sinf(gamma)*user[1],
                      sinf(gamma)*user[0]-cosf(gamma)*user[1]};
    float feedback[4]={0},motor[4],error_sum=0;
    unsigned samples=0;
    for(unsigned k=0;k<2200;++k) {
        float beta=spin*k*dt;
        ChassisMotionInput in={0}; ChassisMotionState state;
        in.time_us=++tick*5000u; in.imu_time_us=in.time_us; in.imu_seq=tick;
        in.pose_valid=in.accel_valid=in.imu_center_valid=1; in.wheel_online_mask=15;
        in.imu_q[0]=1; in.psi_imu_n=gamma; in.theta_bg=Wrap(gamma-beta);
        for(unsigned j=0;j<4;++j) {
            in.wheel_time_us[j]=in.time_us; in.wheel_seq[j]=tick;
            in.motor_dps[j]=feedback[j];
        }
        ChassisMotionUpdate(&in);
        /* 物理相对角速率和轮速模型相差25%，对应本组实测的比例差。
         * 控制提前角必须来自编码器，而不是把轮速omega当作相对角速度。 */
        ChassisMotionControl(user,spin/1.25f,1,MC_ROTATE_CONTROL_KEY,motor);
        ChassisMotionGetState(&state);
        assert(state.mode==MC_SHADOW && state.cmd_g[0]==state.ref_g[0]);
        for(unsigned j=0;j<4;++j) assert(fabsf(motor[j])<=g_chassis_motion_config.wheels.max_motor_dps[0]+.01f);
        float a=1-expf(-.001f/tau);
        for(unsigned sub=0;sub<5;++sub) {
            for(unsigned j=0;j<4;++j) feedback[j]+=a*(motor[j]-feedback[j]);
            if(k<400) continue;
            float w[4]; for(unsigned j=0;j<4;++j) w[j]=feedback[j]*g_chassis_config.mps_per_motor_dps;
            float bx=(-w[0]+w[1]-w[2]+w[3])/(4*c);
            float by=-(w[0]+w[1]-w[2]-w[3])/(4*c);
            float angle=beta+spin*(sub+1)*.001f;
            float nx=cosf(angle)*bx-sinf(angle)*by;
            float ny=sinf(angle)*bx+cosf(angle)*by;
            float e=atan2f(expected[0]*ny-expected[1]*nx,expected[0]*nx+expected[1]*ny);
            error_sum+=fabsf(e); samples++;
        }
    }
    return error_sum/samples;
}

int main(void)
{
    ChassisMotionInit(); ChassisMotionSetMode(MC_SHADOW);
    DirectionGuards();
    float worst=0;
    for(unsigned run=0;run<24;++run) {
        float spin=(run&1 ? -1.f:1.f)*(run&2 ? 10.18f:2.f);
        unsigned axis=(run>>2)&1u;
        float sign=run&8 ? -1.f:1.f;
        float tau=run<16 ? .0135f:.009f;
        float error=Run(spin,axis,sign,tau);
        worst=fmaxf(worst,error);
        if(g_chassis_motion_config.phase_lead_s>0) {
            /* 修复前高速约9°；覆盖响应变化，不能只匹配一个时间常数。 */
            assert(error<.065f);
        } else if(fabsf(spin)>10 && tau==.0135f) assert(error>.12f);
    }
    printf("PASS: independent delayed-wheel plant, both spins, forward/reverse/lateral, encoder/model rate mismatch; lead=%.4fs worst mean direction error=%.3f deg\n",
        (double)g_chassis_motion_config.phase_lead_s,(double)(worst/DEGREE_2_RAD));
    return 0;
}
