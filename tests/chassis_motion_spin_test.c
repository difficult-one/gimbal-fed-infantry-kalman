/* 小陀螺独立物理约束：云台惯性朝向不变时，摇杆平移的 N 系方向不能随底盘转。
 * 不调用生产正逆解生成期望值；由轮速的独立代数还原，再投影到固定世界系。 */
#include "chassis_setup.h"
#include "config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static float Wrap(float angle)
{
    return atan2f(sinf(angle),cosf(angle));
}

int main(void)
{
    const float gamma=.43f;
    const float cg=cosf(gamma), sg=sinf(gamma);
    unsigned tick=0;
    float worst=0,hat_worst=0;
    const ChassisMotionMode modes[]={MC_ESTIMATE_ONLY,MC_SHADOW,MC_ACTIVE_FIXED};
    ChassisMotionInit();
    for(unsigned run=0;run<12;++run) {
        const uint8_t fast=run%4>=2;
        const float magnitude=fast ? g_chassis_config.legacy_spin_command*g_chassis_config.legacy_omega_scale : 2.0f;
        const float spin=run%2 ? -magnitude : magnitude;
        /* 高速方向用例选取不触发统一限幅的小平移；混合限幅由fusion用例覆盖。 */
        const float user[2]={fast ? .05f : .2f,fast ? -.025f : -.1f};
        /* 旧命令 G 的 Y 与右手姿态 G 相反：vN=R(gamma)*(user.x,-user.y)。 */
        const float expected[2]={cg*user[0]+sg*user[1],sg*user[0]-cg*user[1]};
        const ChassisMotionMode mode=modes[run/4];
        ChassisMotionSetMode(mode);
        for(unsigned k=0;k<3000;++k) {
            float beta=spin*k*.005f, cb=cosf(beta), sb=sinf(beta);
            float bx=cb*expected[0]+sb*expected[1];
            float by=sb*expected[0]-cb*expected[1]; // 反射后的旧代数 B.y
            float wheels[4]={(-bx+by)*.7071067812f+g_chassis_motion_config.wheels.rows[0][2]*spin,
                             ( bx+by)*.7071067812f+g_chassis_motion_config.wheels.rows[0][2]*spin,
                             (-bx-by)*.7071067812f+g_chassis_motion_config.wheels.rows[0][2]*spin,
                             ( bx-by)*.7071067812f+g_chassis_motion_config.wheels.rows[0][2]*spin};
            ChassisMotionInput in={0}; ChassisMotionState state; float motor[4];
            in.time_us=++tick*5000u; in.imu_time_us=in.time_us; in.imu_seq=tick;
            in.pose_valid=in.accel_valid=in.imu_center_valid=1; in.wheel_online_mask=15;
            in.imu_q[0]=1; in.psi_imu_n=gamma;
            /* 经编码器映射后的真实相对角，跨 ±pi 仍应连续控制。 */
            in.theta_bg=Wrap(gamma-beta);
            for(unsigned i=0;i<4;++i) {
                in.wheel_time_us[i]=in.time_us; in.wheel_seq[i]=tick;
                in.motor_dps[i]=wheels[i]/g_chassis_config.mps_per_motor_dps;
            }
            ChassisMotionUpdate(&in);
            ChassisMotionControl(user,spin,1,MC_ROTATE_CONTROL_KEY,motor); ChassisMotionGetState(&state);
            /* 高/低速用例间平移速度也有阶跃；先等待参考和KF过渡收敛，
             * 再用原有严格阈值核对稳态方向，不把瞬时速度阶跃当成方向错误。 */
            if(k<1000) continue;
            float w[4];
            for(unsigned i=0;i<4;++i) w[i]=motor[i]*g_chassis_config.mps_per_motor_dps;
            /* 从四轮消去旋转共同项，独立还原实际要求的右手 B 平移。 */
            float actual_bx=(-w[0]+w[1]-w[2]+w[3])/(4*.7071067812f);
            float actual_by=-(w[0]+w[1]-w[2]-w[3])/(4*.7071067812f);
            float actual_nx=cb*actual_bx-sb*actual_by;
            float actual_ny=sb*actual_bx+cb*actual_by;
            float error=hypotf(actual_nx-expected[0],actual_ny-expected[1]);
            float hat_error=hypotf(state.hat_g[0]-user[0],state.hat_g[1]-user[1]);
            worst=fmaxf(worst,error); hat_worst=fmaxf(hat_worst,hat_error);
            if(error>1e-4f || hat_error>1e-4f)
                fprintf(stderr,"mode=%u beta=%.4f theta=%.4f actual N=(%.4f,%.4f) expected=(%.4f,%.4f) hat error=%.4f\n",
                        (unsigned)mode,beta,in.theta_bg,actual_nx,actual_ny,expected[0],expected[1],hat_error);
            assert(state.estimate_valid && error<1e-4f && hat_error<1e-4f);
            assert(fabsf((w[0]+w[1]+w[2]+w[3])/(4*g_chassis_motion_config.wheels.rows[0][2])-spin)<1e-5f);
            if(mode==MC_SHADOW)
                assert(state.cmd_g[0]==state.ref_g[0] && state.cmd_g[1]==state.ref_g[1]);
        }
    }
    printf("PASS: fixed gimbal heading + spinning chassis, 2 rad/s and 1.5x target, both spin directions, 3 modes and repeated angle wraps; world translation error %.8f m/s, returned command velocity error %.8f m/s\n",worst,hat_worst);
    return 0;
}
