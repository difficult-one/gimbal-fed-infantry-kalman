#include "chassis_setup.h"
#include "config.h"
#include "kalman_filter.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void Near(float actual, float expected, float tolerance)
{
    if (!isfinite(actual) || fabsf(actual-expected)>tolerance)
    { fprintf(stderr,"actual %.9g expected %.9g tolerance %.9g\n",actual,expected,tolerance); assert(0); }
}

static void GoldenKF(void)
{
    static KalmanFilter_t k;
    Kalman_Filter_Init(&k,4,0,4);
    k.UseAutoAdjustment=0;
    for(unsigned i=0;i<4;++i)
    {
        k.F_data[5*i]=k.H_data[5*i]=1;
        k.P_data[5*i]=(i&1)?0.25f:0.04f;
        k.R_data[5*i]=(i&1)?0.04f:0.0025f;
    }
    k.F_data[1]=k.F_data[11]=0.005f;
    for(unsigned b=0;b<=10;b+=10)
    {
        k.Q_data[b]=4.166666667e-9f;
        k.Q_data[b+1]=k.Q_data[b+4]=1.25e-6f;
        k.Q_data[b+5]=0.0005f;
    }
    k.MeasuredVector[0]=0.1f; k.MeasuredVector[1]=0.5f;
    Kalman_Filter_Update(&k);
    assert(k.FirstMatError==0);
    Near(k.xhat_data[0],0.09424445f,2e-7f);
    Near(k.xhat_data[1],0.43154983f,2e-7f);
    Near(k.xhat_data[2],0,1e-9f); // 合法零观测。
    memset(k.H_data,0,16*sizeof(float));
    k.xhat_data[0]=1; k.xhat_data[1]=0.5f;
    Kalman_Filter_Update(&k);
    Near(k.xhat_data[0],1.0025f,2e-7f); // 全缺测只预测，不能强迫零速。
    /* 求逆失败后还有成功矩阵操作，首错必须保留。 */
    memset(k.R_data,0,16*sizeof(float));
    Kalman_Filter_Update(&k);
    assert(k.FirstMatError==ARM_MATH_SINGULAR);
}

/* 独立双精度参考：每轴逐个标量观测，协方差使用 Joseph 形式。
 * 与生产代码的四维批量矩阵/简式 P 更新走不同计算路径。 */
static void JosephAxis(double x[2], double p[2][2], double dt, double q,
                       const float z[2], unsigned mask)
{
    x[0]+=dt*x[1];
    p[0][0]+=dt*(p[0][1]+p[1][0])+dt*dt*p[1][1]+q*dt*dt*dt/3;
    p[0][1]+=dt*p[1][1]+q*dt*dt/2; p[1][0]=p[0][1]; p[1][1]+=q*dt;
    const double r[2]={.0025,.04};
    for(unsigned h=0;h<2;++h) if(mask&(1u<<h))
    {
        double k[2]={p[0][h]/(p[h][h]+r[h]),p[1][h]/(p[h][h]+r[h])};
        double y=z[h]-x[h]; x[0]+=k[0]*y; x[1]+=k[1]*y;
        double e[2][2]={{1,0},{0,1}}, out[2][2]={{0}};
        e[0][h]-=k[0]; e[1][h]-=k[1];
        for(unsigned i=0;i<2;++i) for(unsigned j=0;j<2;++j)
        {
            for(unsigned a=0;a<2;++a) for(unsigned b=0;b<2;++b) out[i][j]+=e[i][a]*p[a][b]*e[j][b];
            out[i][j]+=k[i]*r[h]*k[j];
        }
        memcpy(p,out,sizeof(out));
    }
}
static void CompareJoseph(void)
{
    static KalmanFilter_t k;
    double x[2][2]={{0}}, p[2][2][2]={{{.04,0},{0,.25}},{{.04,0},{0,.25}}};
    Kalman_Filter_Init(&k,4,0,4); k.UseAutoAdjustment=0;
    for(unsigned i=0;i<4;++i) { k.P_data[5*i]=(i&1)?g_chassis_motion_config.p_accel:g_chassis_motion_config.p_velocity; k.R_data[5*i]=(i&1)?g_chassis_motion_config.r_accel[0]:g_chassis_motion_config.r_velocity[0]; }
    for(unsigned step=0;step<5000;++step)
    {
        float dt=.001f+(step%11)*.0009f, z[4]={.2f*sinf(step*.01f),.5f*cosf(step*.013f),-.1f*cosf(step*.003f),.3f*sinf(step*.007f)};
        unsigned mask=(step%19==0)?0:(step%3==0?5:(step%7==0?10:15));
        if(step%17==0) memset(z,0,sizeof(z));
        memset(k.F_data,0,16*sizeof(float)); memset(k.H_data,0,16*sizeof(float)); memset(k.Q_data,0,16*sizeof(float));
        for(unsigned i=0;i<4;++i)
        {
            k.F_data[5*i]=1; k.H_data[5*i]=(mask>>i)&1;
            k.MeasuredVector[i]=((mask>>i)&1)?z[i]:0;
        }
        k.F_data[1]=k.F_data[11]=dt;
        for(unsigned a=0;a<2;++a)
        {
            unsigned b=a*10; float q=.1f;
            k.Q_data[b]=q*dt*dt*dt/3; k.Q_data[b+1]=k.Q_data[b+4]=q*dt*dt/2; k.Q_data[b+5]=q*dt;
            JosephAxis(x[a],p[a],dt,q,z+2*a,(mask>>(2*a))&3);
        }
        Kalman_Filter_Update(&k); assert(k.FirstMatError==0);
        for(unsigned a=0;a<2;++a) for(unsigned i=0;i<2;++i)
        {
            Near(k.xhat_data[2*a+i],(float)x[a][i],2e-5f);
            for(unsigned j=0;j<2;++j) Near(k.P_data[(2*a+i)*4+2*a+j],(float)p[a][i][j],1e-6f);
        }
    }
}

static ChassisMotionInput in;
static ChassisMotionState state;
static float refs[4];

static void Frame(float vx_n, float vy_n, float omega, float psi_g, float theta, float dt)
{
    in.time_us+=(uint32_t)lroundf(dt*1e6f);
    in.imu_seq++; in.imu_time_us=in.time_us;
    in.pose_valid=in.accel_valid=1; in.wheel_online_mask=15;
    in.psi_imu_n=psi_g; in.theta_bg=theta; in.omega_imu=0;
    in.accel_n[0]=in.accel_n[1]=0;
    float v_n[2]={vx_n,vy_n}, b[2], twist[3], wheels[4];
    ChassisMotionRotate(-(psi_g-theta),v_n,b);
    /* 实际右手 B 平移先转成旧轮代数，不能用 J 正逆解的自洽代替此符号。 */
    twist[0]=b[0]; twist[1]=-b[1]; twist[2]=omega;
    ChassisMotionForward(twist,wheels);
    for(unsigned i=0;i<4;++i)
    {
        in.wheel_seq[i]++; in.wheel_time_us[i]=in.time_us;
        in.motor_dps[i]=wheels[i]/g_chassis_config.mps_per_motor_dps;
    }
    ChassisMotionUpdate(&in);
}
static void Control(float x,float y,float omega)
{
    const float user[2]={x,y};
    ChassisMotionControl(user,omega,1,1,refs);
    ChassisMotionGetState(&state);
}

static void TestWheelLimitPolicy(void)
{
    /* Exercise a high legacy request after the physical SI angular-speed cap. */
    const float requested_spin=6000.0f*g_chassis_config.legacy_omega_scale;
    const float spin=fminf(requested_spin,g_chassis_motion_config.max_omega);
    ChassisMotionInit();
    ChassisMotionSetMode(MC_ESTIMATE_ONLY);
    for(unsigned i=0;i<300;++i)
    {
        Frame(0,0,0,0,0,.005f);
        Control(.75f,0,requested_spin);
    }
    float wheel_mps[4],twist[3];
    for(unsigned i=0;i<4;++i)
    {
        assert(fabsf(refs[i])<=g_chassis_motion_config.wheels.max_motor_dps[0]+.01f);
        wheel_mps[i]=refs[i]*g_chassis_config.mps_per_motor_dps;
    }
    assert(ChassisMotionInverse(wheel_mps,twist));
    if (g_chassis_motion_config.rotation_priority) {
        Near(twist[2],spin,0.01f);
        assert(state.ref_g[0]>0.0f && state.ref_g[0]<.75f);
        assert(state.limit_flags&MC_LIMIT_REF);
    } else {
        Near(twist[0]/twist[2],.75f/spin,2e-3f);
    }

    if (g_chassis_motion_config.accel_enabled && g_chassis_motion_config.rotation_priority) {
        ChassisMotionInit();
        ChassisMotionSetMode(MC_ACTIVE_FIXED);
        for(unsigned i=0;i<300;++i)
        {
            Frame(0,0,0,0,0,.005f);
            Control(.75f,0,requested_spin);
        }
        assert(state.limit_flags&MC_LIMIT_WHEEL);
        Near(state.kp,0,0);
        Near(state.dv_g[0],0,0); Near(state.dv_g[1],0,0);
        Near(state.cmd_g[0],state.ref_g[0],1e-7f);
        Near(state.cmd_g[1],state.ref_g[1],1e-7f);
        for(unsigned i=0;i<4;++i) assert(fabsf(refs[i])<=g_chassis_motion_config.wheels.max_motor_dps[0]+.01f);
    }
}

int main(void)
{
    TestWheelLimitPolicy();
    GoldenKF();
    CompareJoseph();
    ChassisMotionInit();
    ChassisMotionSetMode(MC_ESTIMATE_ONLY); // 通用基线显式选择模式，上电默认另有生产用例。
    /* 四轮均新鲜，但姿态从一开始就非法：不得误报成 WHEEL。 */
    Frame(0,0,0,0,0,.005f);
    in.time_us+=5000; in.imu_time_us=in.time_us; in.imu_seq++;
    for(unsigned i=0;i<4;++i) { in.wheel_seq[i]++; in.wheel_time_us[i]=in.time_us; }
    in.psi_imu_n=NAN;
    ChassisMotionUpdate(&in);
    ChassisMotionGetState(&state);
    assert(state.wheel_valid_mask==15 && !state.estimate_valid);
    assert(state.exit_reason==MC_EXIT_POSE && state.observation_mask==0);
    Near(6000*g_chassis_config.mps_per_motor_dps,0.41887902f,1e-7f); // 152 mm wheel, 19:1 rotor feedback.
    /* 几何闭环覆盖三自由度和混合运动。 */
    for(unsigned i=0;i<100;++i)
    {
        float t[3]={sinf(i)*0.3f,cosf(i)*0.2f,(int)(i%7)-3.0f}, w[4],back[3];
        ChassisMotionForward(t,w); assert(ChassisMotionInverse(w,back));
        for(unsigned k=0;k<3;++k) Near(back[k],t[k],1e-6f);
        float v[2]={0.3f,-0.7f},r[2],rr[2];
        ChassisMotionRotate(i*0.1f,v,r); ChassisMotionRotate(-(i*0.1f),r,rr);
        Near(rr[0],v[0],2e-7f); Near(rr[1],v[1],2e-7f);
    }
    /* 一开始就跨过 32 位微秒时间戳回绕，并在运动中初始化。 */
    in.time_us=UINT32_MAX-20000u;
    for(unsigned i=0;i<300;++i) { Frame(.15f,.04f,0,0,0,.005f); Control(.2f,0,0); }
    Near(state.hat_g[0],.15f,1e-5f); Near(state.hat_g[1],-.04f,1e-5f);
    assert(state.estimate_valid==(g_chassis_motion_config.accel_enabled?1:0));
    float baseline[4]; memcpy(baseline,refs,sizeof(refs));
    ChassisMotionSetMode(MC_SHADOW);
    for(unsigned i=0;i<100;++i) { Frame(.15f,.04f,0,0,0,.005f); Control(.2f,0,0); }
    for(unsigned i=0;i<4;++i) Near(refs[i],baseline[i],1e-3f);
    if (g_chassis_motion_config.accel_enabled) {
        Near(state.dv_raw_g[0],.005f,2e-6f); Near(state.dv_raw_g[1],.004f,2e-6f);
        ChassisMotionSetMode(MC_ACTIVE_FIXED);
        for(unsigned i=0;i<400;++i) { Frame(.15f,.04f,0,0,0,.005f); Control(.2f,0,0); }
        Near(state.cmd_g[0],.205f,2e-6f); Near(state.cmd_g[1],.004f,2e-6f);
        ChassisMotionSetMode(MC_ACTIVE_SCHEDULED);
        Frame(.15f,.04f,0,0,0,.005f); Control(.2f,0,0); Near(state.kp,0,1e-8f);
        Frame(.15f,.04f,2,0,0,.005f); Control(.2f,0,2); Near(state.kp,.1f,1e-6f);
        in.current_request[0]=g_chassis_motion_config.current_limit;
        Frame(.15f,.04f,2,0,0,.005f); Control(.2f,0,2);
        assert(state.current_saturated_mask==1); Near(state.kp,0,0);
        in.current_request[0]=0;
    }
    /* G/B 转动时，固定 N 系速度不应该跟随坐标旋转产生假加速度。 */
    ChassisMotionSetMode(MC_ESTIMATE_ONLY);
    for(unsigned i=0;i<1000;++i)
    {
        float dt=.002f+(i%9)*.0007f, psi=i*.007f, theta=sinf(i*.013f);
        Frame(.15f,.04f,1,psi,theta,dt); Control(.2f,0,0);
        Near(state.x[0],.15f,2e-5f); Near(state.x[2],.04f,2e-5f);
        assert(state.p_diag[0]>0 && state.p_diag[2]>0);
    }
    /* 重复样本必须屏蔽观测；同一帧只可融合一次。 */
    in.time_us+=5000; ChassisMotionUpdate(&in); ChassisMotionGetState(&state);
    assert(state.observation_mask==0);
    in.time_us+=5000; in.wheel_online_mask=0; in.accel_valid=0;
    ChassisMotionUpdate(&in); Control(.2f,0,0);
    assert(!state.estimate_valid && state.observation_mask==0);
    assert(fabsf(state.x[0])>.1f); // 缺测不伪造静止。
    for(unsigned i=0;i<40;++i) Frame(0,0,0,0,0,.005f);
    Frame(0,0,0,0,0,.1f); Control(.2f,0,0);
    assert(!state.estimate_valid && state.exit_reason==MC_EXIT_DT);
    for(unsigned i=0;i<100;++i) { Frame(0,0,0,0,0,.005f); Control(100,100,100); }
    assert(state.limit_flags&MC_LIMIT_REF);
    for(unsigned i=0;i<4;++i) assert(fabsf(refs[i])<=g_chassis_motion_config.wheels.max_motor_dps[0]+.01f);
    const float zero[2]={0}; ChassisMotionControl(zero,0,0,0,refs);
    for(unsigned i=0;i<4;++i) Near(refs[i],0,0);
    /* 实车只动 yaw 的三组观测（原 offset 和 IMU 航向，rad）。
     * 底盘航向应保持约 0.11 rad；编码器未反向会出现约 -1.164/3.290 rad 的假转动。 */
    const float yaw_samples[3][2]={{-.004f,.107f},{-1.575f,1.715f},{.641f,-.523f}};
    for(unsigned i=0;i<3;++i)
    {
        Frame(0,0,0,yaw_samples[i][1],g_chassis_imu_config.yaw_encoder_sign*yaw_samples[i][0],.005f);
        ChassisMotionGetState(&state);
        Near(state.psi_bn,.11f,.075f);
    }
    /* 按实际日志抽取频率生成足够样本，保证队列溢出检查不依赖 10/50 Hz 配置。 */
    enum { expected_log_depth = 8u };
    for(unsigned i=0;i<(expected_log_depth+2u)*g_chassis_motion_config.log_decimation;++i)
    { Frame(0,0,0,0,0,.005f); ChassisMotionRecord(); }
    ChassisMotionGetState(&state); assert(state.log_drop>0);
    unsigned count=0; while(ChassisMotionPopLog(&state,NULL)) count++;
    assert(count==expected_log_depth);
    /* 生成真实 C 结构体帧，供 Python 解码器检查字段布局和流重同步。 */
    ChassisMotionLogFrame packet={MC_LOG_MAGIC,sizeof(state),7,state};
    packet.state.version=g_chassis_motion_config.version; packet.state.seq=123456u;
    packet.state.imu_q[2]=0.125f; packet.state.kp=0.0625f; packet.state.current_saturated_mask=9u;
    FILE *file=fopen(g_chassis_motion_config.accel_enabled?"build/host_motion/sample_1.bin":"build/host_motion/sample_0.bin","wb");
    assert(file); assert(fwrite(&packet,1,sizeof(packet),file)==sizeof(packet)); fclose(file);
    printf("PASS: golden KF, 5000 Joseph reference steps, zero/missing observations, first matrix error, kinematics, frames, wrap, compensation, timeouts, limits, log overflow (IMU=%d)\n",g_chassis_motion_config.accel_enabled);
    printf("Motion state bytes: %u; log frame bytes: %u\n",(unsigned)sizeof(state),(unsigned)sizeof(ChassisMotionLogFrame));
    return 0;
}
