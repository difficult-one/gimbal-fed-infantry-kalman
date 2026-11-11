/* 独立 Euler 解析轨迹：固定轴交点，IMU 随 yaw/pitch 转动。
 * 参考端使用解析角加速度，生产端使用真实时间差分，允许 O(dt) 误差。 */
#include "chassis_setup.h"
#include "config.h"
#include "imu_compensation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void Cross(const double a[3], const double b[3], double c[3])
{
    c[0]=a[1]*b[2]-a[2]*b[1];
    c[1]=a[2]*b[0]-a[0]*b[2];
    c[2]=a[0]*b[1]-a[1]*b[0];
}

int main(void)
{
    ChassisMotionInit();
    ChassisMotionInput in={0};
    ChassisMotionState s;
    /* 独立于生产配置写出实测安装参考：物理前方 8.4 cm、上方 3.7 cm，
     * 第四次“上抬枪口”日志确认传感器 +X 向后，故传感器系杆臂为负 X。
     * 不从待测宏生成参考值，避免错误的安装符号让参考端和生产端一起错。 */
    const double rx=-0.084, ry=0.0, rz=0.037;
    unsigned tick=0;
    IMURigidState rigid={0};
    const float lever[3]={-0.084f,0.0f,0.037f};
    for (unsigned scenario=0;scenario<4;++scenario)
        for (unsigned step=0;step<2000;++step)
        {
            double t=step*.001, yaw=0, yd=0, ydd=0, pitch=0, pd=0, pdd=0;
            if(scenario!=1) { yaw=.4*t+.05*t*t; yd=.4+.1*t; ydd=.1; }
            if(scenario!=0) { pitch=.3*sin(.9*t); pd=.27*cos(.9*t); pdd=-.243*sin(.9*t); }
            if(scenario==3) { yaw+=1.6*t; yd+=1.6; }
            double cy=cos(yaw), sy=sin(yaw), cp=cos(pitch), sp=sin(pitch);
            double r[3]={cy*cp*rx-sy*ry+cy*sp*rz,
                         sy*cp*rx+cy*ry+sy*sp*rz, -sp*rx+cp*rz};
            double omega[3]={-pd*sy,pd*cy,yd};
            double alpha[3]={-pdd*sy-pd*yd*cy,pdd*cy-pd*yd*sy,ydd};
            double tangent[3], wr[3], centripetal[3];
            Cross(alpha,r,tangent); Cross(omega,r,wr); Cross(omega,wr,centripetal);
            tick++;
            in.time_us=tick*1000u; in.imu_time_us=in.time_us; in.imu_seq=tick;
            in.pose_valid=in.accel_valid=1; in.wheel_online_mask=15;
            in.psi_imu_n=in.theta_bg=(float)-yaw; in.omega_imu=(float)-yd;
            in.imu_q[0]=(float)(cos(yaw/2)*cos(pitch/2));
            in.imu_q[1]=(float)(-sin(yaw/2)*sin(pitch/2));
            in.imu_q[2]=(float)(cos(yaw/2)*sin(pitch/2));
            in.imu_q[3]=(float)(sin(yaw/2)*cos(pitch/2));
            /* R^T omega 的独立解析式，单位 rad/s，和生产层 INS 快照一致。 */
            in.imu_gyro_rad[0]=(float)(-yd*sp);
            in.imu_gyro_rad[1]=(float)pd;
            in.imu_gyro_rad[2]=(float)(yd*cp);
            /* 第四场景还叠加已知轴交点平移加速度，不能被杆臂校正吞掉。 */
            double ax=scenario==3?.12:0, ay=scenario==3?-.04:0;
            in.accel_n[0]=(float)(ax+tangent[0]+centripetal[0]);
            in.accel_n[1]=(float)(ay-tangent[1]-centripetal[1]);
            /* 独立解析 R^T 将物理加速度送入 INS 数学层，1 kHz 校正，
             * 每 5 帧发布给真实 ChassisMotionUpdate，覆盖采样抽取边界。 */
            double ai[3]={ax+tangent[0]+centripetal[0],-ay+tangent[1]+centripetal[1],tangent[2]+centripetal[2]};
            float body[3]={(float)(cy*cp*ai[0]+sy*cp*ai[1]-sp*ai[2]),
                           (float)(-sy*ai[0]+cy*ai[1]),
                           (float)(cy*sp*ai[0]+sy*sp*ai[1]+cp*ai[2])};
            in.imu_center_valid=IMURigidUpdate(&rigid,in.time_us,in.imu_q,in.imu_gyro_rad,body,lever,in.imu_center_n);
            if(tick%5u) continue;
            for(unsigned i=0;i<4;++i) { in.wheel_seq[i]=tick; in.wheel_time_us[i]=in.time_us; }
            ChassisMotionUpdate(&in); ChassisMotionGetState(&s);
            if(step>500) /* Initial consistency window plus KF recovery. */
            {
                assert(s.accel_center_valid);
                if(fabs(s.accel_center_n[0]-ax)>=.004 || fabs(s.accel_center_n[1]-ay)>=.004)
                    fprintf(stderr,"3D mismatch scenario=%u step=%u actual=%.7f,%.7f expected=%.7f,%.7f\n",
                        scenario,step,s.accel_center_n[0],s.accel_center_n[1],ax,ay);
                assert(fabs(s.accel_center_n[0]-ax)<.004);
                assert(fabs(s.accel_center_n[1]-ay)<.004);
                assert(s.accel_used==(g_chassis_motion_config.accel_enabled?1:0));
                assert(s.estimate_valid==(g_chassis_motion_config.accel_enabled?1:0));
                if(!g_chassis_motion_config.accel_enabled) assert(s.observation_mask==5u);
            }
        }
    /* 无效姿态和陀螺必须撤销三维候选，不让非法数值污染轮速估计。 */
    in.time_us+=5000; in.imu_time_us=in.time_us; in.imu_seq++;
    memset(in.imu_q,0,sizeof(in.imu_q));
    const float zero[3]={0};
    in.imu_center_valid=IMURigidUpdate(&rigid,in.time_us,in.imu_q,in.imu_gyro_rad,zero,lever,in.imu_center_n);
    ChassisMotionUpdate(&in); ChassisMotionGetState(&s);
    assert(!s.accel_center_valid && !s.accel_used && !s.estimate_valid);
    in.imu_q[0]=1; in.imu_gyro_rad[1]=NAN;
    in.time_us+=5000; in.imu_time_us=in.time_us; in.imu_seq++;
    in.imu_center_valid=IMURigidUpdate(&rigid,in.time_us,in.imu_q,in.imu_gyro_rad,zero,lever,in.imu_center_n);
    ChassisMotionUpdate(&in); ChassisMotionGetState(&s);
    assert(!s.accel_center_valid && !s.accel_used && !s.estimate_valid);
    assert(isfinite(s.x[0]) && isfinite(s.x[2]));
    printf("PASS: analytic yaw/pitch 3D lever, height and coupling, translation preserved, invalid samples (fusion=%d)\n",g_chassis_motion_config.accel_enabled);
    return 0;
}
