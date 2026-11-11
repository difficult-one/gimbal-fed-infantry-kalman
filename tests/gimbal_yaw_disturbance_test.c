/* 使用生产PID验证积分额度对恒定扰动稳态误差的影响。
 * 一阶惯性+持续负载是机制回归模型，不是实测GM6020模型，不能证明实车调参完成。 */
#include "pid.h"
#include "config.h"
#include <assert.h>
#include <stdio.h>

static float test_step=.002f;
float DWT_GetDeltaT(uint32_t *last)
{
    ++*last;
    return test_step; // 与电机任务约2ms的周期一致，每个PID独立取得同一dt。
}

static void TimingFault(void)
{
    PIDInstance speed,current;
    PID_Init_Config_s s={.Kp=4.5f,.MaxOut=15000,.IntegralLimit=3000,
        .Improve=PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement};
    PID_Init_Config_s c={.Kp=.5f,.Ki=.05f,.MaxOut=8000,.IntegralLimit=5000,
        .Improve=PID_Integral_Limit|PID_OutputFilter,.Output_LPF_RC=.002f};
    PIDInit(&speed,&s); PIDInit(&current,&c);
    /* 与实车速控一致：速度PID串软件电流PID，后者带输出低通。 */
    for(unsigned k=0;k<20;++k) {
        float ref=PIDCalculate(&speed,100,200);
        assert(isfinite(PIDCalculate(&current,0,ref)));
    }
    test_step=0;
    float ref=PIDCalculate(&speed,100,200);
    float output=PIDCalculate(&current,0,ref);
    test_step=.002f;
    for(unsigned k=0;k<100;++k) {
        ref=PIDCalculate(&speed,100,200);
        output=PIDCalculate(&current,0,ref);
        assert(isfinite(output));
    }
    assert(output>100); // 恢复后必须重新驱动，不能被末端NaN转零永久掩盖。
    assert(PIDCalculate(&current,0,NAN)==0);
    assert(isfinite(PIDCalculate(&current,0,450)));
    puts("PASS: zero-dt cascaded PID resumes finite motor output; invalid reference clears history");
}

static float Run(float integral_limit, float load_raw, float tau)
{
    PIDInstance angle, speed;
    PID_Init_Config_s a = {.Kp=8, .MaxOut=500, .DeadBand=.1f,
        .Improve=PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement,
        .IntegralLimit=100};
    PID_Init_Config_s v = {.Kp=50, .Ki=200, .MaxOut=20000,
        .Improve=PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement,
        .IntegralLimit=integral_limit};
    PIDInit(&angle,&a); PIDInit(&speed,&v);
    float heading=0, rate=0, steady=0;
    unsigned count=0;
    for (unsigned tick=0; tick<8000; ++tick) {
        float t=tick*.002f;
        float load=t>=1 && t<10 ? load_raw : 0;
        float target=PIDCalculate(&angle,heading,0);
        float output=PIDCalculate(&speed,rate,target);
        assert(fabsf(output)<=20000.01f);
        assert(fabsf(speed.Iout)<=integral_limit+.01f);
        /* 50 raw/(deg/s)黏性阻尼，时间常数tau；负载符号覆盖两个自旋方向。 */
        rate += (.02f*(output-load)-rate)*.002f/tau;
        heading += rate*.002f;
        if (t>8 && t<9.8f) { steady += heading; ++count; }
        if (t>15) assert(fabsf(heading)<.25f); // 撤去扰动后能恢复，不留下永久偏角。
    }
    return steady/count;
}

int main(void)
{
    TimingFault();
    for (unsigned i=0; i<8; ++i) {
        float load=(i&1 ? -1.0f : 1.0f)*(i&2 ? 7600.0f : 10200.0f);
        float tau=i&4 ? .09f : .03f;
        float old=Run(3000,load,tau);
        float fixed=Run(g_gimbal_config.yaw.controller_param_init_config.speed_PID.IntegralLimit,load,tau);
        float expected=-(load-copysignf(3000,load))/(8*50);
        assert(fabsf(old-expected)<.25f);
        assert(fabsf(fixed)<.25f);
        printf("load=%+.0f tau=%.2f: old error=%.3f deg, fixed=%.3f deg\n",
               load,tau,old,fixed);
    }
    puts("PASS: production cascaded PID, both disturbance signs, 2 loads and 2 time constants; restored heading and bounded output");
    return 0;
}
