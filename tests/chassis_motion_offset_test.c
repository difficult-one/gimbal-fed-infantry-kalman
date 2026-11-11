/* 已知中心不动，IMU 因两级转动件偏置而有加速度；校正后应恢复零平移。 */
#include "chassis_setup.h"
#include "config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    ChassisMotionInit();
    ChassisMotionInput in={0};
    ChassisMotionState s;
    const float ri_local[2]={g_chassis_motion_config.imu_offset_m[0],g_chassis_motion_config.imu_offset_m[1]};
    const float rc_local[2]={g_chassis_motion_config.yaw_center_m[0],g_chassis_motion_config.yaw_center_m[1]};
    for(unsigned step=1;step<1000;++step)
    {
        float t=step*.005f, wb=1+.5f*t, wi=.8f+.3f*t;
        float pb=t+.25f*t*t, pg=.8f*t+.15f*t*t, ri[2],rc[2];
        in.time_us=step*5000u; in.imu_seq=step; in.imu_time_us=in.time_us;
        in.pose_valid=in.accel_valid=1; in.wheel_online_mask=15;
        in.theta_bg=pg-pb;
        in.psi_imu_n=g_chassis_imu_config.on_gimbal?pg:pb;
        in.omega_imu=g_chassis_imu_config.on_gimbal?wi:wb;
        ChassisMotionRotate(g_chassis_imu_config.on_gimbal?pg:pb,ri_local,ri);
        ChassisMotionRotate(pb,rc_local,rc);
        float w=g_chassis_imu_config.on_gimbal?wi:wb, alpha=g_chassis_imu_config.on_gimbal?.3f:.5f;
        in.accel_n[0]=-alpha*ri[1]-w*w*ri[0];
        in.accel_n[1]= alpha*ri[0]-w*w*ri[1];
        if(g_chassis_imu_config.on_gimbal)
        {
            in.accel_n[0]+=-.5f*rc[1]-wb*wb*rc[0];
            in.accel_n[1]+= .5f*rc[0]-wb*wb*rc[1];
        }
        float twist[3]={0,0,wb}, wheels[4]; ChassisMotionForward(twist,wheels);
        for(unsigned i=0;i<4;++i)
        { in.wheel_seq[i]=step; in.wheel_time_us[i]=in.time_us; in.motor_dps[i]=wheels[i]/g_chassis_config.mps_per_motor_dps; }
        ChassisMotionUpdate(&in); ChassisMotionGetState(&s);
        if(step>100)
        {
            assert(s.estimate_valid == (g_chassis_motion_config.accel_enabled ? 1 : 0));
            assert(s.accel_used == (g_chassis_motion_config.accel_enabled ? 1 : 0));
            if (!g_chassis_motion_config.accel_enabled) assert(s.observation_mask == 5u);
            assert(fabsf(s.accel_center_n[0])<.0002f && fabsf(s.accel_center_n[1])<.0002f);
            assert(fabsf(s.x[0])<.00002f && fabsf(s.x[2])<.00002f);
        }
    }
    /* 新 IMU 帧含 NaN 时撤销补偿，但不以 NaN 污染滤波器。 */
    in.time_us+=5000; in.imu_time_us=in.time_us; in.imu_seq++; in.accel_n[0]=NAN;
    ChassisMotionUpdate(&in); ChassisMotionGetState(&s);
    assert(!s.estimate_valid && s.exit_reason==MC_EXIT_IMU);
    assert(isfinite(s.x[0]) && isfinite(s.x[2]));
    printf("PASS: offset correction with fusion=%d, on_gimbal=%d; invalid IMU rejection\n",
           g_chassis_motion_config.accel_enabled, g_chassis_imu_config.on_gimbal);
    return 0;
}
