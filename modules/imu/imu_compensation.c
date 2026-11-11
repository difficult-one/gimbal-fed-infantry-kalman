#include "imu_compensation.h"
#include <math.h>
#include <string.h>

uint8_t IMURecoveryUpdate(IMURecoveryState *s, uint32_t generation,
                         uint32_t dt_us, uint8_t valid, uint8_t settled)
{
    if (generation!=s->clock_generation || !valid || !dt_us || dt_us>20000u) {
        s->clock_generation=generation; s->pending=1; s->good_us=0;
        return 0;
    }
    if (s->pending) {
        if (!settled) s->good_us=0;
        else {
            s->good_us+=dt_us;
            if (s->good_us>=500000u) { s->good_us=500000u; s->pending=0; }
        }
    }
    return !s->pending;
}
static float WrapDeg(float x)
{
    return remainderf(x, 360.0f);
}

static void Cross(const float a[3], const float b[3], float c[3])
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

static uint8_t Rotation(const float q[4], float r[3][3])
{
    float n = 0.0f;
    for (unsigned i = 0; i < 4; ++i) n += q[i]*q[i];
    if (!isfinite(n) || fabsf(n - 1.0f) > 0.05f) return 0;
    float inv = 1.0f/sqrtf(n);
    float w=q[0]*inv, x=q[1]*inv, y=q[2]*inv, z=q[3]*inv;
    r[0][0]=1-2*(y*y+z*z); r[0][1]=2*(x*y-w*z); r[0][2]=2*(x*z+w*y);
    r[1][0]=2*(x*y+w*z); r[1][1]=1-2*(x*x+z*z); r[1][2]=2*(y*z-w*x);
    r[2][0]=2*(x*z-w*y); r[2][1]=2*(y*z+w*x); r[2][2]=1-2*(x*x+y*y);
    return 1;
}

/* 对 omega(t)=b0+b1*u+b2*u² 作最小二乘，u=(t-t_now)/20ms。
 * 取当前端点导数 b1/20ms，而非窗中心导数；因此没有额外半窗延迟，
 * 不需要错开加速度与补偿的时间，也不对重力观测另加一个滞后低通。
 * 短窗抑制 1 ms 相邻差分的量化/重复读数噪声；它不修正机械安装误差。 */
static void AngularAcceleration(IMURigidState *s, uint32_t now,
                                const float gyro[3], float alpha[3])
{
    unsigned head=s->gyro_head;
    s->gyro_time[head]=now;
    memcpy(s->gyro_history[head],gyro,3*sizeof(float));
    s->gyro_head=(head+1u)%IMU_ALPHA_HISTORY;
    if(s->gyro_count<IMU_ALPHA_HISTORY) s->gyro_count++;
    float n=0, sx=0, sxx=0, sxxx=0, sxxxx=0;
    float sy[3]={0}, sxy[3]={0}, sxxy[3]={0};
    for(unsigned k=0;k<s->gyro_count;++k) {
        uint32_t age=(uint32_t)(now-s->gyro_time[k]);
        if(age>20000u) continue;
        float u=-(float)age/20000.0f, u2=u*u;
        n+=1; sx+=u; sxx+=u2; sxxx+=u2*u; sxxxx+=u2*u2;
        for(unsigned j=0;j<3;++j) {
            /* 去掉常数角速度，减小单精度消元的相消误差。 */
            float y=s->gyro_history[k][j]-gyro[j];
            sy[j]+=y; sxy[j]+=u*y; sxxy[j]+=u2*y;
        }
    }
    if(n<5) return; /* 启动短窗尚未建立时沿用相邻差分。 */
    float a=sxx-sx*sx/n, b=sxxx-sx*sxx/n, c=sxxxx-sxx*sxx/n;
    float det=a*c-b*b;
    if(det<=1e-9f) return;
    for(unsigned j=0;j<3;++j)
        alpha[j]=((sxy[j]-sx*sy[j]/n)*c-(sxxy[j]-sxx*sy[j]/n)*b)/(det*.02f);
}

uint8_t IMURigidPrepare(IMURigidState *s, uint32_t time_us,
                        const float gyro[3], const float lever_b[3])
{
    float alpha[3], tangent[3], wr[3], centripetal[3], a2=0.0f;
    float dt = (uint32_t)(time_us - s->time_us)*1e-6f;
    uint8_t continuous = s->ready && dt >= 0.0005f && dt <= 0.02f;
    uint8_t valid = 1u;
    for (unsigned i=0; i<3; ++i)
        valid &= isfinite(gyro[i]) && isfinite(lever_b[i]);
    if (!valid) {
        memset(s, 0, sizeof(*s));
        s->trace.reject = IMU_RIGID_INPUT;
        return 0;
    }
    memset(&s->trace, 0, sizeof(s->trace));
    s->trace.dt_us = (uint32_t)(time_us - s->time_us);
    uint8_t was_ready = s->ready;
    if(!continuous) s->gyro_count=s->gyro_head=0;
    for (unsigned i=0; i<3; ++i)
    {
        alpha[i] = continuous ? (gyro[i]-s->last_gyro[i])/dt : 0.0f;
        s->last_gyro[i] = gyro[i];
    }
    AngularAcceleration(s,time_us,gyro,alpha);
    for(unsigned i=0;i<3;++i) {
        a2+=alpha[i]*alpha[i];
        s->trace.alpha[i]=alpha[i];
    }
    s->time_us = time_us;
    s->ready = 1;
    if (!continuous || !isfinite(a2) || a2 > 100.0f*100.0f)
    {
        /* 只保留本帧角速度作为下一次差分起点，不跨越缺帧/异常继续滤波。 */
        memset(s->center_b, 0, sizeof(s->center_b));
        s->gyro_count=s->gyro_head=0;
        s->trace.reject = !was_ready ? IMU_RIGID_SEED :
            (!continuous ? IMU_RIGID_TIME : IMU_RIGID_ALPHA);
        return 0;
    }
    /* d(omega_b)/dt 就是 alpha_b：旋转坐标附加项 omega×omega 恒为零。
     * 避免先把 omega 旋到由 EKF 修正的导航系，再对它差分引入姿态修正噪声。 */
    Cross(alpha, lever_b, tangent);
    Cross(gyro, lever_b, wr);
    Cross(gyro, wr, centripetal);
    memcpy(s->trace.tangent, tangent, sizeof(tangent));
    memcpy(s->trace.centripetal, centripetal, sizeof(centripetal));
    return 1;
}

uint8_t IMURigidProject(IMURigidState *s, const float q[4],
                        const float center_b[3], float out_n[3])
{
    return IMURigidProjectFiltered(s, q, center_b, IMU_MOTION_LPF_S, out_n);
}

uint8_t IMURigidProjectFiltered(IMURigidState *s, const float q[4],
                                const float center_b[3], float lpf_s, float out_n[3])
{
    float r[3][3];
    uint8_t valid = Rotation(q, r);
    for (unsigned i=0; i<3; ++i) valid &= isfinite(center_b[i]);
    if (!valid) {
        memset(s, 0, sizeof(*s));
        s->trace.reject = IMU_RIGID_INPUT;
        return 0;
    }
    if (!s->ready || s->trace.reject) return 0;
    /* 与刚体求导使用同一采样间隔，不混用 SPI 读取之前的任务起点间隔。 */
    float dt = s->trace.dt_us*1e-6f;
    float k = dt/(lpf_s+dt);
    for (unsigned i=0; i<3; ++i)
        s->center_b[i] += k*(center_b[i]-s->center_b[i]);
    for (unsigned i=0; i<3; ++i)
    {
        out_n[i] = 0.0f;
        for (unsigned j=0; j<3; ++j) out_n[i] += r[i][j]*s->center_b[j];
    }
    return 1;
}

uint8_t IMURigidUpdate(IMURigidState *s, uint32_t time_us, const float q[4],
                       const float gyro[3], const float motion_b[3],
                       const float lever_b[3], float out_n[3])
{
    float r[3][3], center_b[3];
    uint8_t valid = Rotation(q, r);
    for (unsigned i=0; i<3; ++i) valid &= isfinite(motion_b[i]);
    if (!valid) {
        memset(s, 0, sizeof(*s));
        s->trace.reject = IMU_RIGID_INPUT;
        return 0;
    }
    if (!IMURigidPrepare(s, time_us, gyro, lever_b)) return 0;
    for (unsigned i=0; i<3; ++i)
        center_b[i] = motion_b[i]-s->trace.tangent[i]-s->trace.centripetal[i];
    return IMURigidProject(s, q, center_b, out_n);
}

void IMURigidStatsPush(IMURigidStatsState *s, uint32_t time_us,
                       const IMURigidTrace *trace, uint8_t valid,
                       const float before_n[3], const float after_n[3])
{
    IMURigidStats *w = &s->window;
    if (!w->samples) { w->start_us=time_us; w->dt_min_us=UINT32_MAX; }
    w->end_us=time_us;
    w->samples++;
    if (trace->dt_us && !(trace->reject & IMU_RIGID_SEED)) {
        if (trace->dt_us < w->dt_min_us) w->dt_min_us=trace->dt_us;
        if (trace->dt_us > w->dt_max_us) w->dt_max_us=trace->dt_us;
    }
    for (unsigned i=0; i<2; ++i)
        valid &= isfinite(before_n[i]) && isfinite(after_n[i]);
    if (!valid || trace->reject) {
        w->rejected++;
        w->reject_mask |= trace->reject ? trace->reject : IMU_RIGID_INPUT;
    } else {
        w->good++;
        for (unsigned i=0; i<3; ++i) {
            w->alpha2[i] += trace->alpha[i]*trace->alpha[i];
            w->alpha_peak[i] = fmaxf(w->alpha_peak[i], fabsf(trace->alpha[i]));
            w->tangent2[i] += trace->tangent[i]*trace->tangent[i];
            w->centripetal2[i] += trace->centripetal[i]*trace->centripetal[i];
            w->tangent_peak[i] = fmaxf(w->tangent_peak[i], fabsf(trace->tangent[i]));
            w->centripetal_peak[i] = fmaxf(w->centripetal_peak[i], fabsf(trace->centripetal[i]));
        }
        for (unsigned i=0; i<2; ++i) {
            float correction=before_n[i]-after_n[i];
            w->before2[i] += before_n[i]*before_n[i];
            w->after2[i] += after_n[i]*after_n[i];
            w->before_sum[i] += before_n[i]; w->after_sum[i] += after_n[i];
            w->correction2[i] += correction*correction;
            w->cross[i] += before_n[i]*correction;
        }
    }
    /* 双缓冲发布：不因 RTT 阻塞而暂停采样，也不把漏发的秒窗拼接成一个窗。 */
    if ((uint32_t)(time_us-w->start_us) >= 1000000u) {
        w->seq=s->published.seq+1u;
        if (w->dt_min_us==UINT32_MAX) w->dt_min_us=0;
        s->published=*w;
        memset(w, 0, sizeof(*w));
    }
}

static void PauseYaw(IMUYawBiasState *s, uint32_t reason)
{
    if (s->ready && (s->settle_s > 0.0f || s->window_s > 0.0f)) s->debug.resets++;
    if (reason) {
        s->debug.reason=s->debug.last_reason=reason;
        s->debug.last_block_us=s->debug.time_us;
        for (unsigned i=0; i<IMU_AID_CAUSE_COUNT; ++i)
            if (reason & IMU_AID_BIT(i)) s->debug.hits[i]++;
    }
    s->ready = s->status = 0;
    s->settle_s = s->window_s = 0.0f;
    s->yaw_delta = s->encoder_delta = s->jacobian_sum = 0.0f;
    /* 保留已学修正；禁止运动期间将它突然清零，造成速度反馈跳变。 */
}

void IMUYawBiasUpdate(IMUYawBiasState *s, const IMUYawBiasInput *in)
{
    float r[3][3], gyro2=0.0f;
    s->debug.time_us=in->time_us; s->debug.reason=0;
    uint32_t reason=in->inhibit;
    if (!in->valid && !reason) reason |= IMU_AID_BIT(IMU_AID_CALLER);
    uint8_t valid = Rotation(in->q, r) &&
        isfinite(in->yaw_deg) && isfinite(in->encoder_deg) &&
        isfinite(in->pitch_encoder_deg) && isfinite(in->yaw_ref_deg) &&
        isfinite(in->pitch_ref_deg) && isfinite(in->accel_norm);
    if (!valid) reason |= IMU_AID_BIT(IMU_AID_INPUT);
    if (isfinite(in->accel_norm) && fabsf(in->accel_norm-9.81f) >= 0.35f)
        reason |= IMU_AID_BIT(IMU_AID_GRAVITY);
    for (unsigned i=0; i<3; ++i) gyro2 += in->gyro[i]*in->gyro[i];
    if (!isfinite(gyro2) || gyro2 >= 0.05f*0.05f) reason |= IMU_AID_BIT(IMU_AID_GYRO);
    if (reason) { PauseYaw(s, reason); return; }
    /* ZYX 航向对机体系 gz 的灵敏度 J=cos(roll)/cos(pitch)。
     * 用完整四元数求 J，不能在 PITCH 倾斜时仍假定 yaw_dot=gz。 */
    float cp2 = 1.0f-r[2][0]*r[2][0];
    if (cp2 < 0.49f || r[2][2] < 0.5f) { PauseYaw(s, IMU_AID_BIT(IMU_AID_POSE)); return; }
    float jacobian = r[2][2]/cp2;
    float dt = (uint32_t)(in->time_us-s->time_us)*1e-6f;
    float de = WrapDeg(in->encoder_deg-s->last_encoder);
    float dy = WrapDeg(in->yaw_deg-s->last_yaw);
    if (s->ready) {
        if (dt < 0.001f || dt > 0.02f) reason |= IMU_AID_BIT(IMU_AID_TIMESTEP);
        if (fabsf(in->yaw_ref_deg-s->last_yaw_ref) >= 0.001f) reason |= IMU_AID_BIT(IMU_AID_YAW_TARGET);
        if (fabsf(in->pitch_ref_deg-s->last_pitch_ref) >= 0.001f) reason |= IMU_AID_BIT(IMU_AID_PITCH_TARGET);
        if (fabsf(WrapDeg(in->pitch_encoder_deg-s->last_pitch)) >= 0.06f) reason |= IMU_AID_BIT(IMU_AID_PITCH_STEP);
        if (fabsf(WrapDeg(in->pitch_encoder_deg-s->pitch_anchor)) >= 0.15f) reason |= IMU_AID_BIT(IMU_AID_PITCH_SPAN);
        if (fabsf(de) >= 0.2f) reason |= IMU_AID_BIT(IMU_AID_YAW_STEP);
        if (fabsf(dy) >= 0.2f) reason |= IMU_AID_BIT(IMU_AID_IMU_STEP);
    }
    uint8_t stable=s->ready && !reason;
    if (!stable) PauseYaw(s, reason);
    s->time_us=in->time_us; s->last_yaw=in->yaw_deg;
    s->last_encoder=in->encoder_deg; s->last_pitch=in->pitch_encoder_deg;
    s->last_yaw_ref=in->yaw_ref_deg; s->last_pitch_ref=in->pitch_ref_deg;
    s->ready=1;
    if (!stable) { s->pitch_anchor=in->pitch_encoder_deg; s->status=1; return; }
    s->settle_s += dt;
    if (s->settle_s < 2.0f) { s->status=1; return; }
    s->window_s += dt;
    s->yaw_delta += dy;
    s->encoder_delta += de;
    s->jacobian_sum += jacobian*dt;
    if (s->window_s < 3.0f) return;
    if (fabsf(s->encoder_delta/s->window_s) <= 0.75f)
    {
        /* yaw_delta-encoder_delta 是观测误差，实际缓慢旋转不会被强行置零。
         * 两者在当前安装下同号（已有手动 YAW 日志验证），均使用角度制。
         * 先转 rad/s，再除以 J，得到应从机体系 gz 中减掉的增量。 */
        float delta = 0.5f*(s->yaw_delta-s->encoder_delta)*0.01745329252f/s->jacobian_sum;
        delta = fmaxf(-0.001f, fminf(delta, 0.001f));
        float target = s->trim_z+delta;
        s->trim_z = fmaxf(-0.01f, fminf(target, 0.01f));
        s->status = fabsf(target) > 0.01f ? 3 : 2;
        s->updates++;
    }
    else { PauseYaw(s, IMU_AID_BIT(IMU_AID_WINDOW_SPEED)); return; }
    s->window_s=s->yaw_delta=s->encoder_delta=s->jacobian_sum=0.0f;
}
