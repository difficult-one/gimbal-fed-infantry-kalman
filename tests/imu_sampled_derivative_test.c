/* 独立采样模型：1 kHz 传感器保持、600~1400 us 读取抖动、实际量化步长、
 * accel/gyro 8.9/9.9 ms 不同硬件延迟。真实运动由解析轨迹给出。
 * 比较相邻差分基线，两个输出使用完全相同的运动低通，不能靠抹掉平移过关。 */
#include "imu_compensation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static double Quantize(double x,double step) { return nearbyint(x/step)*step; }
int main(void)
{
    const unsigned periods[]={600,1400,800,1200,1000};
    const float lever[3]={-.084f,0,.037f}, q[4]={1,0,0,0};
    for(unsigned moving=0;moving<3;++moving) {
        IMURigidState state={0};
        unsigned now=0, previous_time=0, used=0;
        double last_w=0, baseline[2]={0}, reference=0, old2=0, new2=0, signal2=0;
        for(unsigned k=0;k<30000;++k) {
            now+=periods[k%5];
            double dt=(now-previous_time)*1e-6;
            double sensor_t=(now/1000)*.001, tg=sensor_t-.0099, ta=sensor_t-.0089;
            double wg=moving?cos(2*tg):0, wa=moving?cos(2*ta):0;
            double aa=moving?-2*sin(2*ta):0;
            double translation=moving==2?.12*sin(.8*ta):0;
            float gyro[3]={0,0,(float)Quantize(wg+.002*sin(2*3.141592653589793*80*tg),.00053211256775)};
            float accel[3]={(float)Quantize(.084*wa*wa+translation,.002392578),
                            (float)Quantize(-.084*aa,.002392578),0}, out[3];
            double adjacent=(gyro[2]-last_w)/dt;
            double gain=dt/(IMU_MOTION_LPF_S+dt);
            baseline[0]+=gain*(accel[0]-.084*gyro[2]*gyro[2]-baseline[0]);
            baseline[1]+=gain*(accel[1]+.084*adjacent-baseline[1]);
            reference+=gain*(translation-reference);
            unsigned valid=IMURigidUpdate(&state,now,q,gyro,accel,lever,out);
            if(k>1000 && valid) {
                old2+=(baseline[0]-reference)*(baseline[0]-reference)+baseline[1]*baseline[1];
                new2+=(out[0]-reference)*(out[0]-reference)+out[1]*out[1];
                signal2+=out[0]*out[0]; used++;
            }
            last_w=gyro[2]; previous_time=now;
        }
        printf("sampled scenario=%u adjacent=%.6f endpoint=%.6f signal=%.6f m/s2\n",
               moving,sqrt(old2/used),sqrt(new2/used),sqrt(signal2/used));
        assert(new2<old2*.36); /* 相同低通后，误差 RMS 至少减少 40%。 */
        if(moving==2) assert(sqrt(signal2/used)>.07); /* 真实平移不能被滤掉。 */
    }
    /* 二次角速度的当前导数有解析解：验证端点时间而非窗中心，并跨越回卷。 */
    IMURigidState state={0};
    unsigned now=UINT32_MAX-50000u, elapsed=0;
    double max_error=0;
    for(unsigned k=0;k<200;++k) {
        unsigned step=periods[k%5]; now+=step; elapsed+=step;
        double t=elapsed*1e-6;
        float gyro[3]={0,0,(float)(.2+2*t+2.5*t*t)};
        IMURigidPrepare(&state,now,gyro,lever);
        if(k>30) max_error=fmax(max_error,fabs(state.trace.alpha[2]-(2+5*t)));
    }
    assert(max_error<.002);
    printf("PASS: irregular-timestamp endpoint derivative and timer wrap, max error %.7f rad/s2\n",max_error);
    return 0;
}
