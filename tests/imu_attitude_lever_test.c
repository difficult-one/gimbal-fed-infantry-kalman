/* 编译实际 INS_Task、姿态 EKF 和刚体数学层；仅替换硬件/调度入口。
 * 参考运动在导航系独立生成，不能用生产补偿结果生成加速度样本。 */
#include "ins.h"
#include "config.h"
#include "quaternion_ekf.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const INS_Init_Config *const ins_config = &g_imu_config;
static INS_t INS;
static INS_MotionSnapshot motion_snapshot;
static INS_SampleDebug sample_debug;
static uint32_t last_sample_time_us;
static uint8_t have_sample_time, read_interrupted;
static const float MPU6500_ACCEL_SEN = 0.002392578125f;
static const float MPU6500_GYRO_SEN = 5.3211256775067750677506775065818e-4f;
static uint8_t calibration_status;
static float calibration_temperature;
static IMURecoveryState motion_recovery={.pending=1};
HostDWT host_dwt;
HostCoreDebug host_core;
uint32_t host_primask;
static uint32_t clock_generation;
static uint32_t chassis_still_time;
static uint8_t chassis_still;
static IMURigidState rigid_state;
static IMURigidStatsState rigid_stats;
static IMUYawBiasState yaw_bias;
static IMU_Param_t IMU_Param;
static mpu_data_t mpu_data;
static float imu_lever_b[3]={-.084f,0,.037f};
static uint8_t lever_configured=1, INS_Init_flag=1;
static uint32_t INS_DWT_Count, sample_us;
static float task_step=.001f;
static const float xb[3]={1,0,0}, yb[3]={0,1,0}, zb[3]={0,0,1};
#define RAD_2_DEGREE 57.295779513f
float DWT_GetDeltaT(uint32_t *counter) { (void)counter; return task_step; }
uint64_t DWT_GetTimeline_us(void) { return sample_us; }
void DWT_GetClockDebug(DWT_ClockDebug *debug) { memset(debug,0,sizeof(*debug)); debug->generation=clock_generation; }
static INS_SampleStatus mock_read_status;
static unsigned alternate_fresh;
void DWT_Delay(float seconds) { sample_us += (uint32_t)(seconds*1e6f); }
static float Dot3d(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static INS_SampleStatus MpuReadRaw(int16_t raw[7])
{
    const int16_t stationary[7] = {0, 0, 4096, 0, 10, 0, 0};
    sample_debug.read_time_us = sample_us;
    sample_debug.attempt_seq++;
    INS_SampleStatus status = mock_read_status;
    if (alternate_fresh)
        status = (sample_debug.attempt_seq % 2u) ? INS_SAMPLE_FRESH : INS_SAMPLE_NOT_READY;
    if (status == INS_SAMPLE_FRESH) {
        sample_debug.accepted_seq++;
        memcpy(raw, stationary, sizeof(stationary));
    }
    return status;
}
static INS_SampleStatus mpu_get_data(int16_t raw[7]) { return MpuReadRaw(raw); }
static void IMU_Temperature_Ctrl(void) {}
static void IMU_Temperature_Update(float duty) { assert(duty == 0.0f); }
static void IMU_Param_Correction(IMU_Param_t *p,float g[3],float a[3]) { (void)p;(void)g;(void)a; }
static int legacy_order;
static void ReviewAttitudeUpdate(float gx,float gy,float gz,float ax,float ay,float az,float step)
{
    /* 重现旧顺序：EKF 观测原始加速度，后面仍完整扣除杆臂。
     * 新旧两种运行使用相同的解析轨迹、生产 EKF 与生产 INS 主体。 */
    if(legacy_order) { ax=INS.Accel[0]; ay=INS.Accel[1]; az=INS.Accel[2]; }
    IMU_QuaternionEKF_Update(gx,gy,gz,ax,ay,az,step);
}
void BodyFrameToEarthFrame(const float *,float *,float *);
void EarthFrameToBodyFrame(const float *,float *,float *);
#define IMU_QuaternionEKF_Update ReviewAttitudeUpdate
#include "ins_task_under_test.h"
#undef IMU_QuaternionEKF_Update

static void Cross(const double a[3],const double b[3],double c[3])
{
    c[0]=a[1]*b[2]-a[2]*b[1]; c[1]=a[2]*b[0]-a[0]*b[2]; c[2]=a[0]*b[1]-a[1]*b[0];
}

int main(int argc,char **argv)
{
    assert(argc==3);
    /* QuaternionEKF 的既有快平方根依赖 32 位 long；Windows/ARM 均满足。 */
    _Static_assert(sizeof(long)==sizeof(float),"test needs 32-bit long like target");
    legacy_order=atoi(argv[1]);
    int scenario=atoi(argv[2]);
    float initial[4]={1,0,0,0};
    IMU_QuaternionEKF_Init(initial,10,.001f,1000000,1,0);
    INS.AccelLPF=ins_config->motion_lpf_s;
    if (scenario == 8) {
        float q[4] = {0};
        mock_read_status = INS_SAMPLE_SPI_TIMEOUT;
        mpu_offset_call();
        assert(calibration_status == 2u);
        assert(sample_debug.accepted_seq == 0u && sample_us <= 15000500u);
        assert(mpu_data.GyroOffset[0] == ins_config->fallback_gyro_offset[0] && isfinite(mpu_data.AccelScale));
        assert(!InitQuaternion(q) && q[0] == 0.0f);

        alternate_fresh = 1u;
        mpu_data.Accel[2] = 9.81f;
        unsigned accepted_before = sample_debug.accepted_seq;
        assert(InitQuaternion(q) && q[0] == 1.0f && q[1] == 0.0f && q[2] == 0.0f);
        assert(sample_debug.accepted_seq == accepted_before + 100u);
        accepted_before = sample_debug.accepted_seq;
        mpu_offset_call();
        assert(calibration_status == 1u && sample_debug.accepted_seq == accepted_before + 6000u);
        assert(fabsf(mpu_data.GyroOffset[0] - 0.0053211f) < 1e-6f);
        puts("PASS: startup calibration and initial attitude count only fresh samples and finish on failures");
        return 0;
    }
    if (scenario == 7) {
        mpu_data.Accel[2] = 9.81f;
        for (unsigned k = 0; k < 1000; ++k) {
            sample_us += 1000u;
            INS_Task();
        }
        assert(motion_snapshot.valid && motion_snapshot.center_valid && motion_snapshot.motion_ready);
        const INS_MotionSnapshot accepted = motion_snapshot;
        const uint64_t updates = QEKF_INS.UpdateCount;

        mock_read_status = INS_SAMPLE_NOT_READY;  /* A normal poll with no new sensor sample. */
        sample_us += 1000u;
        INS_Task();
        assert(QEKF_INS.UpdateCount == updates);
        assert(motion_snapshot.valid && motion_snapshot.motion_ready);
        assert(motion_snapshot.seq == accepted.seq && motion_snapshot.time_us == accepted.time_us);

        mock_read_status = INS_SAMPLE_SPI_TIMEOUT;  /* A SPI timeout has no usable payload. */
        sample_us += 1000u;
        INS_Task();
        assert(QEKF_INS.UpdateCount == updates);
        assert(!motion_snapshot.valid && !motion_snapshot.center_valid && !motion_snapshot.motion_ready);
        assert(motion_snapshot.seq == accepted.seq && motion_snapshot.time_us == accepted.time_us);
        assert(memcmp(INS.q, accepted.q, sizeof(accepted.q)) == 0);
        assert(!rigid_state.ready && !rigid_state.gyro_count);

        mock_read_status = INS_SAMPLE_FRESH;  /* Identical values still form new samples. */
        for (unsigned k = 0; k < 101; ++k) {
            sample_us += 1000u;
            INS_Task();
        }
        assert(motion_snapshot.seq == accepted.seq + 101u);
        assert(motion_snapshot.valid && motion_snapshot.center_valid && !motion_snapshot.motion_ready);
        assert(QEKF_INS.UpdateCount > updates);
        for (unsigned k = 0; k < 500; ++k) {
            sample_us += 1000u;
            INS_Task();
        }
        assert(motion_snapshot.motion_ready);

        mock_read_status = INS_SAMPLE_NOT_READY;
        for (unsigned k = 0; k < 21; ++k) {
            sample_us += 1000u;
            INS_Task();
        }
        assert(!motion_snapshot.valid && !motion_snapshot.center_valid && !motion_snapshot.motion_ready);
        assert(!rigid_state.ready && !rigid_state.gyro_count);
        puts("PASS: production INS distinguishes a no-ready poll, SPI timeout, a fresh equal value and a persistent gap");
        return 0;
    }
    if(scenario==6) {
        mpu_data.Accel[2]=9.81f;
        for(unsigned k=0;k<2000;++k) { sample_us+=1000; INS_Task(); }
        /* Stale positive chassis permission cannot authorize forced correction. */
        chassis_still=1; chassis_still_time=sample_us-30000u;
        mpu_data.Accel[0]=1.5f;
        for(unsigned k=0;k<1000;++k) {
            sample_us+=1000; INS_Task();
            assert(!QEKF_INS.GravityRecoveryAllowed);
            assert(QEKF_INS.ConvergeFlag && QEKF_INS.IMU_QuaternionEKF.SkipEq5);
        }
        /* A fresh moving-platform report also withdraws that permission. */
        chassis_still=0;
        for(unsigned k=0;k<600;++k) {
            sample_us+=1000; chassis_still_time=sample_us; INS_Task();
            assert(!QEKF_INS.GravityRecoveryAllowed && QEKF_INS.ConvergeFlag);
        }
        chassis_still=1;
        for(unsigned k=0;k<400;++k) {
            sample_us+=1000; chassis_still_time=sample_us; INS_Task();
            assert(QEKF_INS.GravityRecoveryAllowed && QEKF_INS.ConvergeFlag);
        }
        chassis_still=0; sample_us+=1000; INS_Task();
        assert(QEKF_INS.GravityRecoveryTime==0);
        puts("PASS: production INS uses fresh independent chassis permission for gravity recovery; stale/moving permission rejected");
        return 0;
    }
    if(scenario==5) {
        mpu_data.Accel[2]=9.81f;
        for(unsigned k=0;k<1000;++k) { sample_us+=1000; INS_Task(); }
        assert(motion_snapshot.valid && motion_snapshot.center_valid && motion_snapshot.motion_ready);
        const float bad_steps[]={0,0,0,NAN,INFINITY,-.001f,.03f};
        for(unsigned i=0;i<sizeof(bad_steps)/sizeof(bad_steps[0]);++i) {
            uint64_t updates=QEKF_INS.UpdateCount;
            float last_q[4]; memcpy(last_q,INS.q,sizeof(last_q));
            task_step=bad_steps[i];
            if(task_step==.03f) sample_us+=30000;
            INS_Task();
            assert(!motion_snapshot.valid && !motion_snapshot.center_valid && !motion_snapshot.motion_ready);
            assert(motion_snapshot.attitude_flags==16u && isnan(motion_snapshot.attitude_chi_ratio));
            assert(QEKF_INS.UpdateCount==updates);
            assert(!memcmp(last_q,INS.q,sizeof(last_q)));
            assert(isfinite(INS.YawTotalAngle));
            for(unsigned j=0;j<3;++j) assert(isfinite(INS.MotionAccel_b[j]));
            /* 同时覆盖直接调用姿态 EKF 的入口，不能只靠任务层屏蔽。 */
            IMU_QuaternionEKF_Update(0,0,0,0,0,9.81f,task_step);
            assert(QEKF_INS.UpdateCount==updates);
        }
        task_step=.001f;
        for(unsigned k=0;k<100;++k) {
            sample_us+=1000; INS_Task();
            assert(motion_snapshot.valid && isfinite(INS.YawTotalAngle));
        }
        assert(motion_snapshot.center_valid);
        assert(!motion_snapshot.motion_ready); /* 100 ms is insufficient. */
        for(unsigned k=0;k<600;++k) { sample_us+=1000; INS_Task(); }
        assert(motion_snapshot.motion_ready);
        clock_generation++;
        sample_us+=1000; INS_Task();
        assert(!motion_snapshot.valid && !motion_snapshot.motion_ready);
        for(unsigned k=0;k<600;++k) { sample_us+=1000; INS_Task(); }
        assert(motion_snapshot.valid && motion_snapshot.motion_ready);
        puts("PASS: production INS clock faults preserve attitude, latch motion recovery, require settled samples after normal-dt clock repair");
        return 0;
    }
    double before2=0,after2=0;
    unsigned samples=0;
    for(unsigned k=1;k<=120000;++k) {
        double u=k*.001-40, yaw=0,yd=0,ydd=0,pitch=0,pd=0,pdd=0;
        if(u>0 && scenario==0) {
            yaw=.5*(1-cos(.1*u)); yd=.05*sin(.1*u); ydd=.005*cos(.1*u);
        }
        if(u>0 && scenario==1) {
            yaw=.6*(u-2*(1-exp(-u/2))); yd=.6*(1-exp(-u/2)); ydd=.3*exp(-u/2);
        }
        if(u>0 && scenario==2) {
            yaw=.4*sin(.6*u); yd=.24*cos(.6*u); ydd=-.144*sin(.6*u);
            pitch=.2*sin(.7*u); pd=.14*cos(.7*u); pdd=-.098*sin(.7*u);
        }
        double cy=cos(yaw),sy=sin(yaw),cp=cos(pitch),sp=sin(pitch);
        double r[3]={cy*cp*(-.084)+cy*sp*.037,
                     sy*cp*(-.084)+sy*sp*.037,-sp*(-.084)+cp*.037};
        double w[3]={-pd*sy,pd*cy,yd};
        double a[3]={-pdd*sy-pd*yd*cy,pdd*cy-pd*yd*sy,ydd};
        double tangent[3],wr[3],centripetal[3];
        Cross(a,r,tangent); Cross(w,r,wr); Cross(w,wr,centripetal);
        /* 无旋转的真实平移，比较新旧输出，不能因本次修复多扣平移量。
         * 此用例不宣称仅靠加速度计能区分长期平移与倾斜。 */
        double translation=scenario==3 && u>0?.12*sin(.8*u):0;
        double specific[3]={translation+tangent[0]+centripetal[0],
                            tangent[1]+centripetal[1],9.81+tangent[2]+centripetal[2]};
        mpu_data.Accel[0]=(float)(cy*cp*specific[0]+sy*cp*specific[1]-sp*specific[2]);
        mpu_data.Accel[1]=(float)(-sy*specific[0]+cy*specific[1]);
        mpu_data.Accel[2]=(float)(cy*sp*specific[0]+sy*sp*specific[1]+cp*specific[2]);
        mpu_data.Gyro[0]=(float)(-yd*sp);
        mpu_data.Gyro[1]=(float)pd;
        mpu_data.Gyro[2]=(float)(yd*cp);
        if(scenario==4) mpu_data.Gyro[2]=.002f*sinf((float)(2*3.141592653589793*80*k*.001));
        sample_us=k*1000u;
        INS_Task();
        assert(motion_snapshot.valid);
        float norm2=0;
        for(unsigned j=0;j<4;++j) norm2+=motion_snapshot.q[j]*motion_snapshot.q[j];
        assert(fabsf(norm2-1)<.000002f);
        assert(!memcmp(motion_snapshot.accel_raw,mpu_data.Accel,sizeof(mpu_data.Accel)));
        if(k>60000) {
            assert(motion_snapshot.center_valid);
            before2+=INS.MotionAccel_n[0]*INS.MotionAccel_n[0]+INS.MotionAccel_n[1]*INS.MotionAccel_n[1];
            after2+=motion_snapshot.center_n[0]*motion_snapshot.center_n[0]+motion_snapshot.center_n[1]*motion_snapshot.center_n[1];
            samples++;
        }
    }
    printf("%d %d %.9f %.9f\n",legacy_order,scenario,sqrt(before2/samples),sqrt(after2/samples));
    /* 检查生产任务失效路径：超时不跨段求导，首帧不发中心值，随后恢复。
     * 先停止模拟运动，再建立静态历史；这些探针不纳入上面的轨迹 RMS。 */
    memset(mpu_data.Gyro,0,sizeof(mpu_data.Gyro));
    mpu_data.Accel[0]=mpu_data.Accel[1]=0; mpu_data.Accel[2]=9.81f;
    task_step=.03f; sample_us+=30000;
    INS_Task();
    assert(!motion_snapshot.center_valid && rigid_state.trace.reject==IMU_RIGID_TIME);
    task_step=.001f; sample_us+=1000;
    INS_Task();
    assert(!motion_snapshot.center_valid && rigid_state.trace.reject==IMU_RIGID_SEED);
    sample_us+=1000;
    INS_Task();
    assert(motion_snapshot.center_valid);
    sample_us+=1000; lever_configured=0;
    INS_Task();
    assert(!motion_snapshot.center_valid && rigid_state.trace.reject==IMU_RIGID_DISABLED);
    return 0;
}
