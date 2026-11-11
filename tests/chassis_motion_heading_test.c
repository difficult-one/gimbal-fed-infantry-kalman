/* 独立覆盖平台航向到融合的接入。旧测试直接生成一致的 N 系观测，
 * 无法发现平台层多加 pi。这里由传感器方向和独立电机代数生成两路观测。 */
#include "chassis_setup.h"
#include "config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 第十四次原始日志的八个加减速区间：端点四轮反馈与底盘航向，及期间
 * 中心加速度的梯形积分。用户确认落地、无打滑/碰撞。独立证据约束符号，
 * 不把 10 Hz 差分/积分当成精确轮径标定，也不重放成完整 200 Hz 实车轨迹。 */
static void RealTranslationDirection(unsigned *tick)
{
    static const struct {
        unsigned seq[2]; float dps[2][4], psi[2], integral[2];
    } events[] = {
        {{6480,6560},{{0,0,0,0},{1066,-973,1082,-1147}}, {.028f,.026f},{-.075492f,-.020048f}},
        {{6640,6720},{{3633,-3643,3102,-3780},{-41,22,-90,81}}, {.031f,.028f},{.505190f,-.025955f}},
        {{9380,9440},{{-266,-201,206,160},{-3183,-3188,3295,3017}}, {.052f,.055f},{-.056850f,.169100f}},
        {{9780,9840},{{1697,2543,-2274,-1977},{4951,5735,-5649,-5081}}, {.059f,.041f},{.088696f,-.269189f}},
        {{10020,10080},{{-17,-53,-18,59},{-2517,-2696,2799,2722}}, {.045f,.047f},{-.040750f,.179346f}},
        {{10420,10520},{{859,368,-427,-516},{3148,3639,-3745,-3194}}, {.049f,.045f},{.027499f,-.266643f}},
        {{10840,10940},{{422,356,-705,-385},{-3649,-4121,3850,3573}}, {.051f,.052f},{-.060150f,.440308f}},
        {{11240,11340},{{1181,1433,-1209,-1355},{5470,6359,-6613,-5682}}, {.061f,.058f},{.024550f,-.533352f}}
    };
    for(unsigned e=0;e<sizeof(events)/sizeof(events[0]);++e) {
        float vn[2][2];
        for(unsigned k=0;k<2;++k) {
            ChassisMotionInput in={0}; ChassisMotionState s;
            in.time_us=++*tick*5000u; in.imu_time_us=in.time_us; in.imu_seq=*tick;
            in.pose_valid=in.accel_valid=in.imu_center_valid=1; in.wheel_online_mask=15;
            in.psi_imu_n=events[e].psi[k];
            for(unsigned i=0;i<4;++i) {
                in.wheel_seq[i]=*tick; in.wheel_time_us[i]=in.time_us;
                in.motor_dps[i]=events[e].dps[k][i];
            }
            ChassisMotionUpdate(&in); ChassisMotionGetState(&s);
            memcpy(vn[k],s.wheel_n,sizeof(vn[k]));
        }
        float dot=(vn[1][0]-vn[0][0])*events[e].integral[0]+
                  (vn[1][1]-vn[0][1])*events[e].integral[1];
        if(dot<=0) fprintf(stderr,"log seq %u..%u: direction dot=%g\n",events[e].seq[0],events[e].seq[1],dot);
        assert(dot>0);
    }
    puts("PASS: eight real forward/lateral acceleration integrals agree with production wheel projection");
}

int main(void)
{
    ChassisMotionInit();
    ChassisMotionInput in={0}; ChassisMotionState s;
    unsigned tick=0;
    RealTranslationDirection(&tick);
    float max_error=0;
    for(unsigned run=0;run<4;++run) {
        ChassisMotionSetMode(MC_ESTIMATE_ONLY);
        float gamma=-2.0f+run*.9f, theta=.2f*run;
        float c=cosf(gamma), sn=sinf(gamma), ct=cosf(theta), st=sinf(theta);
        float xn[3]={c,sn,0};
        for(unsigned k=0;k<4000;++k) {
            float t=k*.005f;
            /* 右手姿态 G 的 X 对应 sensor+x，Y 对应 sensor-y。
             * 独立轮代数再反射 B.y；第十四次落地侧移提供此符号的实测依据。 */
            float vx=.2f*sinf(.5f*t), vy=.1f*cosf(.4f*t);
            float ax=.1f*cosf(.5f*t), ay=-.04f*sinf(.4f*t);
            float bx=ct*vx-st*vy, by=-(st*vx+ct*vy);
            float wheels[4]={(-bx+by)*.7071067812f,(bx+by)*.7071067812f,
                             (-bx-by)*.7071067812f,(bx-by)*.7071067812f};
            in.time_us=++tick*5000u; in.imu_time_us=in.time_us; in.imu_seq=tick;
            in.pose_valid=in.accel_valid=in.imu_center_valid=1; in.wheel_online_mask=15;
            in.psi_imu_n=ChassisMotionHeading(xn); in.theta_bg=theta;
            /* Rz(gamma)*(ax,-ay,0) 是 INS 导航系加速度。 */
            in.imu_center_n[0]=c*ax+sn*ay;
            in.imu_center_n[1]=sn*ax-c*ay;
            for(unsigned i=0;i<4;++i) {
                in.wheel_time_us[i]=in.time_us; in.wheel_seq[i]=tick;
                in.motor_dps[i]=wheels[i]/g_chassis_config.mps_per_motor_dps;
            }
            ChassisMotionUpdate(&in);
            const float user[2]={.2f,-.1f}; float motor[4];
            ChassisMotionControl(user,0,1,1,motor);
            ChassisMotionGetState(&s);
            if(k>500) {
                float en0=c*vx+sn*vy, en1=-sn*vx+c*vy;
                max_error=fmaxf(max_error,hypotf(s.x[0]-en0,s.x[2]-en1));
                assert(hypotf(s.wheel_n[0]-en0,s.wheel_n[1]-en1)<1e-6f);
                assert(s.estimate_valid && s.accel_used);
                /* 真实平移随固定 G 朝向定义，返回命令必须与 theta 无关。 */
                float hx=vx, hy=-vy;
                assert(hypotf(s.hat_g[0]-hx,s.hat_g[1]-hy)<.002f);
                /* 航向修复仅影响观测投影；四轮命令与旧方向完全相同。 */
                float ux=ct*.2f-st*.1f, uy=-st*.2f-ct*.1f;
                float expected[4]={(-ux+uy)*.7071067812f,(ux+uy)*.7071067812f,
                                  (-ux-uy)*.7071067812f,(ux-uy)*.7071067812f};
                for(unsigned i=0;i<4;++i) assert(fabsf(motor[i]-expected[i]/g_chassis_config.mps_per_motor_dps)<.002f);
            }
        }
        /* 任意航向/云台角度下，执行旧命令后测得同一速度时，主动补偿应为零。
         * 电机独立旧公式生成，验证新的 hat 返回链与控制输出互逆。 */
        ChassisMotionSetMode(MC_ACTIVE_FIXED);
        const float user[2]={.2f,-.1f};
        float ux=ct*user[0]+st*user[1], uy=-st*user[0]+ct*user[1];
        float expected[4]={(-ux+uy)*.7071067812f,(ux+uy)*.7071067812f,
                          (-ux-uy)*.7071067812f,(ux-uy)*.7071067812f};
        /* 从上一段正弦速度跳到常速会留下 KF 过渡，先给 4.5 s 收敛再验稳态。 */
        for(unsigned k=0;k<1000;++k) {
            in.time_us=++tick*5000u; in.imu_time_us=in.time_us; in.imu_seq=tick;
            in.imu_center_n[0]=in.imu_center_n[1]=0;
            for(unsigned i=0;i<4;++i) {
                in.wheel_seq[i]=tick; in.wheel_time_us[i]=in.time_us;
                in.motor_dps[i]=expected[i]/g_chassis_config.mps_per_motor_dps;
            }
            float motor[4]; ChassisMotionUpdate(&in);
            ChassisMotionControl(user,0,1,1,motor); ChassisMotionGetState(&s);
            if(k>900) {
                assert(s.estimate_valid);
                if(hypotf(s.error_g[0],s.error_g[1])>=1e-5f)
                    fprintf(stderr,"steady run=%u k=%u error=(%.9g,%.9g) hat=(%.9g,%.9g) ref=(%.9g,%.9g)\n",run,k,s.error_g[0],s.error_g[1],s.hat_g[0],s.hat_g[1],s.ref_g[0],s.ref_g[1]);
                assert(hypotf(s.error_g[0],s.error_g[1])<1e-5f);
                assert(hypotf(s.dv_g[0],s.dv_g[1])<1e-6f);
                for(unsigned i=0;i<4;++i) assert(fabsf(motor[i]-expected[i]/g_chassis_config.mps_per_motor_dps)<.002f);
            }
        }
    }
    assert(max_error<.002f);
    printf("PASS: sensor heading -> production KF, independent wheel algebra, four headings, translation velocity error %.7f m/s; original motor references preserved\n",max_error);
    return 0;
}
