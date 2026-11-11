/* 独立物理参考：有噪静止、解析正弦运动、编码器闭环慢漂与真实缓慢转动。 */
#include "imu_compensation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void RigidNoiseAndTiming(void)
{
    IMURigidState s={0};
    const float q[4]={1,0,0,0}, lever[3]={-0.084f,0,0.037f};
    float output[3], old=0.0f, new2=0.0f, old2=0.0f;
    /* 同一静止传感器的高频陀螺噪声不应未经低通直接进入中心加速度。 */
    for(unsigned k=1;k<=10000;++k)
    {
        float t=k*.001f, gyro[3]={0,0,.002f*sinf(2*3.14159265f*80*t)}, a[3]={0};
        uint8_t valid=IMURigidUpdate(&s,k*1000u,q,gyro,a,lever,output);
        float derivative=(gyro[2]-old)/.001f;
        old=gyro[2];
        if(k>100 && valid) { new2+=output[1]*output[1]; old2+=(.084f*derivative)*(.084f*derivative); }
    }
    assert(new2 < old2*.1f);
    printf("PASS: stationary gyro-noise RMS %.5f -> %.5f m/s2\n",sqrtf(old2/9900),sqrtf(new2/9900));
    memset(&s,0,sizeof(s));
    float legacy_filtered[2]={0}, legacy2=0.0f, corrected2=0.0f;
    unsigned count=0;
    /* 固定轴交点、水平 yaw 正弦轨迹：a_b=(-w² rx, alpha rx, 0)。
     * 原路径先低通 a，再减未滤波刚体项；新路径先相减再统一低通。 */
    for(unsigned k=1;k<=10000;++k)
    {
        double t=k*.001, yaw=.5*sin(2*t), w=cos(2*t), alpha=-2*sin(2*t);
        float quat[4]={(float)cos(yaw/2),0,0,(float)sin(yaw/2)};
        float gyro[3]={0,0,(float)w};
        float a[3]={(float)(.084*w*w),(float)(-.084*alpha),0};
        float gain=.001f/(IMU_MOTION_LPF_S+.001f);
        for(unsigned j=0;j<2;++j) legacy_filtered[j]+=gain*(a[j]-legacy_filtered[j]);
        uint8_t valid=IMURigidUpdate(&s,k*1000u,quat,gyro,a,lever,output);
        if(k>200 && valid)
        {
            legacy2+=(legacy_filtered[0]-a[0])*(legacy_filtered[0]-a[0])+
                     (legacy_filtered[1]-a[1])*(legacy_filtered[1]-a[1]);
            corrected2+=output[0]*output[0]+output[1]*output[1];
            count++;
        }
    }
    assert(corrected2 < legacy2*.02f);
    printf("PASS: analytic yaw timing residual %.6f -> %.6f m/s2\n",sqrtf(legacy2/count),sqrtf(corrected2/count));
    float gyro[3]={0}, a[3]={0};
    assert(!IMURigidUpdate(&s,10100000u,q,gyro,a,lever,output)); /* 丢帧不能跨段求导。 */
    assert(IMURigidUpdate(&s,10101000u,q,gyro,a,lever,output));
    gyro[0]=NAN;
    assert(!IMURigidUpdate(&s,10102000u,q,gyro,a,lever,output));
    gyro[0]=0;
    assert(!IMURigidUpdate(&s,10103000u,q,gyro,a,lever,output));
    assert(IMURigidUpdate(&s,10104000u,q,gyro,a,lever,output));
    assert(!IMURigidUpdate(&s,10104000u,q,gyro,a,lever,output)); /* 重复时间戳。 */
    float bad_q[4]={0};
    assert(!IMURigidUpdate(&s,10105000u,bad_q,gyro,a,lever,output));
    memset(&s,0,sizeof(s));
    assert(!IMURigidUpdate(&s,UINT32_MAX-500u,q,gyro,a,lever,output));
    assert(IMURigidUpdate(&s,499u,q,gyro,a,lever,output)); /* 时间戳回卷仍是 1 ms。 */
}

static float Encoder(double degrees)
{
    double x=fmod(degrees,360.0);
    if(x<0) x+=360.0;
    return (float)(floor(x*8192.0/360.0)*360.0/8192.0);
}

static IMUYawBiasInput Input(void)
{
    IMUYawBiasInput in={0};
    in.q[0]=1; in.accel_norm=9.81f; in.valid=1;
    return in;
}

static void YawClosedLoop(void)
{
    for(unsigned run=0;run<3;++run)
    {
        IMUYawBiasState s={0}; IMUYawBiasInput in=Input();
        double pitch=run==2?.45:0.0, bias=run==1?-.0016:.0016;
        double encoder=run==1?359.9:.1;
        in.q[0]=(float)cos(pitch/2); in.q[2]=(float)sin(pitch/2);
        /* 完美航向闭环让 IMU 航向保持零，但真实云台会为抵消测量零偏而转动。
         * 倾斜时的独立 Euler 关系 yaw_dot=gz/cos(pitch)。 */
        for(unsigned k=1;k<=12000;++k)
        {
            double real_rate=-(bias-s.trim_z)/cos(pitch);
            encoder+=real_rate*.005*180.0/3.141592653589793;
            in.time_us=k*5000u;
            in.encoder_deg=Encoder(encoder);
            in.gyro[0]=(float)(-real_rate*sin(pitch));
            in.gyro[2]=(float)(real_rate*cos(pitch)+bias-s.trim_z);
            IMUYawBiasUpdate(&s,&in);
        }
        assert(s.updates>10);
        assert(fabs(s.trim_z-bias)<.0003);
        printf("PASS: encoder-aided yaw trim true %.7f estimated %.7f rad/s (pitch %.2f)\n",bias,s.trim_z,pitch);
        float saved=s.trim_z; in.valid=0;
        for(unsigned k=0;k<2000;++k) { in.time_us+=5000; IMUYawBiasUpdate(&s,&in); }
        assert(s.status==0 && s.trim_z==saved);
    }
    /* 真实慢转同时被 IMU 和编码器观测，不得当作零偏吞掉。 */
    IMUYawBiasState s={0}; IMUYawBiasInput in=Input();
    for(unsigned k=1;k<=12000;++k)
    {
        in.time_us=k*5000u; in.yaw_deg=.3f*k*.005f;
        in.encoder_deg=Encoder(in.yaw_deg+359.7);
        in.gyro[2]=.3f*.01745329252f;
        IMUYawBiasUpdate(&s,&in);
    }
    assert(fabsf(s.trim_z)<.0003f);
    /* 持续目标变化、PITCH 运动、断流均不得积累学习窗口。 */
    memset(&s,0,sizeof(s)); in=Input();
    for(unsigned k=1;k<=4000;++k)
    {
        in.time_us=k*5000u; in.yaw_ref_deg=k*.01f;
        IMUYawBiasUpdate(&s,&in);
    }
    assert(s.updates==0 && s.trim_z==0);
    memset(&s,0,sizeof(s)); in=Input();
    for(unsigned k=1;k<=4000;++k)
    {
        in.time_us=k*5000u; in.pitch_encoder_deg=k*.005f*.1f;
        IMUYawBiasUpdate(&s,&in);
    }
    assert(s.updates==0);
    in.valid=0; IMUYawBiasUpdate(&s,&in); assert(s.status==0);
    memset(&s,0,sizeof(s)); in=Input();
    for(unsigned k=1;k<=2000;++k)
    {
        /* 每隔 100 ms 才到一帧，不能把失联段累计成完整静止窗口。 */
        in.time_us=k*100000u; in.yaw_deg=k*.01f;
        IMUYawBiasUpdate(&s,&in);
    }
    assert(s.updates==0 && s.trim_z==0);
    memset(&s,0,sizeof(s)); in=Input();
    for(unsigned k=1;k<=20000;++k)
    {
        in.time_us=k*5000u; in.yaw_deg=.2f*k*.005f;
        IMUYawBiasUpdate(&s,&in);
    }
    assert(s.status==3 && fabsf(s.trim_z-0.01f)<1e-7f);
    printf("PASS: true slow yaw preserved; command/pitch motion and invalid frames freeze learning\n");
}

static void Diagnostics(void)
{
    /* 一个 5 ms 的失败即可清空学习窗口；随后恢复正常时仍必须保留失败证据。 */
    IMUYawBiasState yaw={0}; IMUYawBiasInput in=Input();
    for(unsigned k=1;k<=590;++k) { in.time_us=k*5000u; IMUYawBiasUpdate(&yaw,&in); }
    assert(yaw.settle_s>2 && yaw.window_s>0.9f);
    in.time_us+=5000;
    in.inhibit=IMU_AID_BIT(IMU_AID_WHEEL_RAW)|IMU_AID_BIT(IMU_AID_PITCH_FEEDBACK);
    in.valid=0;
    IMUYawBiasUpdate(&yaw,&in);
    assert(yaw.debug.reason==in.inhibit && yaw.debug.resets==1);
    assert(yaw.debug.hits[IMU_AID_WHEEL_RAW]==1 && yaw.debug.hits[IMU_AID_PITCH_FEEDBACK]==1);
    assert(yaw.debug.hits[IMU_AID_CALLER]==0 && yaw.window_s==0);
    uint32_t blocked_time=in.time_us;
    in.inhibit=0; in.valid=1; in.time_us+=5000;
    IMUYawBiasUpdate(&yaw,&in);
    assert(yaw.debug.reason==0 && yaw.debug.last_block_us==blocked_time);
    assert(yaw.debug.last_reason==(IMU_AID_BIT(IMU_AID_WHEEL_RAW)|IMU_AID_BIT(IMU_AID_PITCH_FEEDBACK)));
    assert(yaw.updates==0 && yaw.debug.hits[IMU_AID_WHEEL_RAW]==1);
    for(unsigned k=0;k<1003;++k) { in.time_us+=5000; IMUYawBiasUpdate(&yaw,&in); }
    assert(yaw.updates==1); /* 门槛没有放宽：恢复后仍要等待完整约 5 s。 */
    in.time_us+=5000; in.pitch_ref_deg+=0.01f; IMUYawBiasUpdate(&yaw,&in);
    assert(yaw.debug.reason & IMU_AID_BIT(IMU_AID_PITCH_TARGET));
    assert(yaw.debug.hits[IMU_AID_PITCH_TARGET]==1);
    in.time_us+=5000; in.accel_norm=10.3f; IMUYawBiasUpdate(&yaw,&in);
    assert(yaw.debug.reason==IMU_AID_BIT(IMU_AID_GRAVITY));
    in.time_us+=5000; in.accel_norm=9.81f; in.gyro[0]=NAN; IMUYawBiasUpdate(&yaw,&in);
    assert(yaw.debug.reason & IMU_AID_BIT(IMU_AID_GYRO));

    /* 高频轨迹在 10 Hz 下恰好混叠为零；秒窗仍必须得到非零的准确 RMS。
     * alpha_z=2 rad/s²、r_x=-.084 m => tangent_y=-.168 m/s²，
     * omega_z=.5 rad/s => centripetal_x=.021 m/s²。 */
    IMURigidStatsState stats={0};
    IMURigidTrace trace={.alpha={0,0,2},.tangent={0,-.168f,0},
                         .centripetal={.021f,0,0},.dt_us=1000};
    for(unsigned k=0;k<=1000;++k) {
        float x=sinf(2*3.14159265358979323846f*20*k*.001f);
        float before[3]={x,2*x,0},after[3]={x/2,x,0};
        IMURigidStatsPush(&stats,1000u+k*1000u,&trace,1,before,after);
    }
    IMURigidStats first=stats.published;
    assert(first.seq==1 && first.samples==1001 && first.good==1001 && first.rejected==0);
    assert(first.end_us-first.start_us==1000000u && first.dt_min_us==1000 && first.dt_max_us==1000);
    float variance=500.0f/1001.0f;
    assert(fabsf(first.before2[0]/first.good-variance)<1e-5f);
    assert(fabsf(first.after2[0]/first.good-variance/4)<1e-5f);
    assert(fabsf(first.cross[0]/first.good-variance/2)<1e-5f);
    assert(fabsf(first.tangent2[1]/first.good-.168f*.168f)<1e-6f);
    assert(fabsf(first.centripetal2[0]/first.good-.021f*.021f)<1e-7f);
    assert(first.tangent_peak[1]==.168f && first.centripetal_peak[0]==.021f);
    assert(first.alpha_peak[2]==2 && fabsf(first.alpha2[2]/first.good-4)<1e-6f);
    float before[3]={0},after[3]={0};
    trace.reject=IMU_RIGID_TIME; trace.dt_us=30000;
    IMURigidStatsPush(&stats,1031000u,&trace,0,before,after);
    trace.reject=0; trace.dt_us=1000;
    before[0]=NAN;
    IMURigidStatsPush(&stats,1032000u,&trace,1,before,after);
    before[0]=0;
    IMURigidStatsPush(&stats,2031000u,&trace,1,before,after);
    assert(stats.published.seq==2 && stats.published.good==1 && stats.published.rejected==2);
    assert(stats.published.reject_mask==(IMU_RIGID_TIME|IMU_RIGID_INPUT));
    assert(stats.published.dt_max_us==30000 && stats.published.before2[0]==0);
    assert(first.good==1001); /* 后续窗不覆盖已经复制的快照。 */
    memset(&stats,0,sizeof(stats));
    IMURigidStatsPush(&stats,UINT32_MAX-500000u,&trace,1,before,after);
    IMURigidStatsPush(&stats,499999u,&trace,1,before,after);
    assert(stats.published.seq==1 && stats.published.end_us-stats.published.start_us==1000000u);
    printf("PASS: brief multi-cause aid interruption retained, full-rate RMS/cross/peaks, rejection and timer wrap\n");
}

int main(void)
{
    RigidNoiseAndTiming(); YawClosedLoop(); Diagnostics(); return 0;
}
