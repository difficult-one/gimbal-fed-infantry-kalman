/* Independent contact-velocity model exercises the production portability API. */
#include "motion_port.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846f
static void near(float actual, float expected, float tolerance)
{
    if (!(fabsf(actual-expected)<=tolerance)) {
        fprintf(stderr,"actual %.9g expected %.9g tolerance %.9g\n",actual,expected,tolerance);
        assert(0);
    }
}
static float unit(const ChassisKinematicsConfig *c, unsigned i)
{ return PI/180*c->radius_m[i]/c->gear_ratio[i]*c->motor_sign[i]; }

/* Generate encoder feedback from analytic wheel contact velocities. */
static void physical(const ChassisKinematicsConfig *c, const float t[3], float dps[4], float angle[4])
{
    const float ox[]={-1,-1,1,1}, oy[]={1,-1,1,-1}, my[]={-1,1,1,-1};
    for (unsigned i=0;i<4;++i) {
        float vx=t[0]-t[2]*c->y_m[i], vy=t[1]+t[2]*c->x_m[i];
        float speed;
        if (c->type==CK_SWERVE) { speed=hypotf(vx,vy); angle[i]=atan2f(vy,vx); }
        else if (c->type==CK_OMNI_X) speed=(ox[i]*vx+oy[i]*vy)/sqrtf(2);
        else speed=vx+my[i]*vy;
        dps[i]=speed/unit(c,i);
    }
}
static void kinematics(void)
{
    for (ChassisDriveType type=CK_OMNI_X;type<=CK_SWERVE;++type) {
        ChassisKinematicsConfig c;
        ChassisKinematicsProfile(&c,type,.42f,.36f,.067f,19);
        c.steer_cosine_compensation=0;
        for (unsigned i=0;i<4;++i) {
            c.radius_m[i]+=.001f*i; c.gear_ratio[i]+=i;
            c.motor_sign[i]=i&1 ? -1 : 1;
            c.max_motor_dps[i]=12000+2000*i;
        }
        ChassisKinematics m;
        assert(ChassisKinematicsInit(&m,&c));
        for (unsigned k=0;k<300;++k) {
            float t[]={2*sinf(k*.17f),1.5f*cosf(k*.31f),5*sinf(k*.09f)};
            float dps[4], angle[4]={0}, observed[3];
            physical(&c,t,dps,angle);
            assert(ChassisKinematicsObserve(&m,dps,angle,observed));
            for (unsigned a=0;a<3;++a) near(observed[a],t[a],3e-5f);
            ChassisWheelCommand out;
            assert(ChassisKinematicsCommand(&m,t,angle,&out));
            for (unsigned i=0;i<4;++i) near(out.motor_dps[i],dps[i],.025f);
            float alpha=ChassisKinematicsScale(&m,t), limited[]={t[0]*alpha,t[1]*alpha,t[2]*alpha};
            assert(alpha>=0 && alpha<=1);
            physical(&c,limited,dps,angle);
            for (unsigned i=0;i<4;++i) assert(fabsf(dps[i])<=c.max_motor_dps[i]+.02f);
            float v[]={t[0],t[1]}, omega=.5f*t[2];
            float spin[]={0,0,omega};
            omega*=ChassisKinematicsScale(&m,spin)*.99f;
            alpha=ChassisKinematicsTranslationScale(&m,v,omega);
            limited[0]=alpha*v[0]; limited[1]=alpha*v[1]; limited[2]=omega;
            assert(ChassisKinematicsScale(&m,limited)>.99999f);
            if (alpha<.999f) {
                limited[0]=(alpha+.001f)*v[0]; limited[1]=(alpha+.001f)*v[1];
                assert(ChassisKinematicsScale(&m,limited)<1);
            }
        }
        c.gear_ratio[1]=0; assert(!ChassisKinematicsInit(&m,&c));
        c.gear_ratio[1]=19; c.radius_m[0]=NAN; assert(!ChassisKinematicsInit(&m,&c));
        c.radius_m[0]=.06f;
        if (type==CK_SWERVE) {
            for (unsigned i=0;i<4;++i) c.x_m[i]=c.y_m[i]=0;
        } else memset(c.rows,0,sizeof(c.rows));
        assert(!ChassisKinematicsInit(&m,&c));
    }

    ChassisKinematicsConfig c;
    ChassisKinematicsProfile(&c,CK_SWERVE,.4f,.3f,.06f,19);
    ChassisKinematics m; assert(ChassisKinematicsInit(&m,&c));
    float angle[]={0,0,0,0}, forward[]={1,0,0}, dps[4], measured[3];
    physical(&c,forward,dps,angle);
    assert(ChassisKinematicsObserve(&m,dps,angle,measured)); /* Parallel modules remain observable. */
    near(measured[0],1,1e-6f); near(measured[1],0,1e-6f); near(measured[2],0,1e-6f);
    ChassisWheelCommand out;
    float reverse[]={-1,0,0};
    assert(ChassisKinematicsCommand(&m,reverse,angle,&out));
    for (unsigned i=0;i<4;++i) { near(out.steer_rad[i],0,1e-6f); assert(out.motor_dps[i]<0); }
    float side[]={0,1,0};
    assert(ChassisKinematicsCommand(&m,side,angle,&out));
    for (unsigned i=0;i<4;++i) { near(fabsf(out.steer_rad[i]),PI/2,1e-6f); near(out.motor_dps[i],0,.01f); }
    float zero[]={0,0,0};
    for (unsigned i=0;i<4;++i) angle[i]=8*PI+.3f;
    assert(ChassisKinematicsCommand(&m,zero,angle,&out));
    for (unsigned i=0;i<4;++i) near(out.steer_rad[i],angle[i],1e-6f);
    assert(ChassisKinematicsCommand(&m,reverse,angle,&out));
    for (unsigned i=0;i<4;++i) assert(fabsf(out.steer_rad[i]-angle[i])<=PI/2);
    angle[2]=NAN;
    assert(!ChassisKinematicsCommand(&m,side,angle,&out));
    assert(!ChassisKinematicsObserve(&m,dps,angle,measured));
    for (unsigned i=0;i<4;++i) near(out.motor_dps[i],0,0);
    c.x_m[0]=FLT_MAX; assert(!ChassisKinematicsInit(&m,&c));
}

typedef struct {
    ChassisMotionInput input;
    ChassisWheelCommand output;
    uint8_t received, enabled;
    unsigned writes;
} FakePort;
static uint8_t read_port(void *ctx, uint32_t now, ChassisMotionInput *in)
{ (void)now; FakePort *p=ctx; *in=p->input; return p->received; }
static void write_port(void *ctx, const ChassisWheelCommand *out, uint8_t enabled)
{ FakePort *p=ctx; p->output=*out; p->enabled=enabled; p->writes++; }
static void fresh(FakePort *p,uint32_t now)
{
    p->input.imu_time_us=now; p->input.imu_seq++;
    for (unsigned i=0;i<4;++i) {
        p->input.wheel_seq[i]++; p->input.wheel_time_us[i]=now;
        p->input.steer_time_us[i]=now;
    }
}
static void stopped(const FakePort *p)
{
    assert(!p->enabled);
    for (unsigned i=0;i<4;++i) near(p->output.motor_dps[i],0,0);
}
static void interfaces(void)
{
    for (ChassisDriveType type=CK_OMNI_X;type<=CK_SWERVE;++type)
      for (unsigned gimbal=0;gimbal<2;++gimbal) {
        ChassisMotionConfig c;
        ChassisMotionConfigDefault(&c,type);
        c.imu_on_gimbal=gimbal; c.wheels.steer_cosine_compensation=0;
        c.velocity_slew=c.omega_slew=1000; c.sample_max_us=15000;
        assert(ChassisMotionConfigure(&c));
        FakePort p={0}; p.received=1;
        p.input.pose_valid=p.input.accel_valid=p.input.imu_center_valid=1;
        p.input.wheel_online_mask=p.input.steer_online_mask=15;
        float theta=.8f, psi=-1.2f, user[]={.4f,-.2f};
        float body[]={cosf(theta)*user[0]-sinf(theta)*user[1],
                      sinf(theta)*user[0]+cosf(theta)*user[1],.7f};
        p.input.theta_bg=theta; p.input.psi_imu_n=psi+(gimbal ? theta : 0);
        physical(&c.wheels,body,p.input.motor_dps,p.input.steer_rad);
        ChassisMotionPort port={&p,read_port,write_port};
        ChassisMotionCommand cmd={{user[0],user[1]},body[2],1,0};
        uint32_t now=UINT32_MAX-50000;
        for (unsigned k=0;k<100;++k) {
            now+=5000; fresh(&p,now);
            (void)ChassisMotionStep(&port,now,&cmd);
            if (!k) stopped(&p);
        }
        ChassisMotionState state;
        ChassisMotionGetState(&state);
        assert(p.enabled && state.estimate_valid && state.mode==MC_SHADOW);
        near(state.psi_bn,psi,1e-6f);
        near(state.hat_g[0],user[0],1e-5f); near(state.hat_g[1],user[1],1e-5f);
        for (unsigned i=0;i<4;++i) near(p.output.motor_dps[i],p.input.motor_dps[i],.01f);
        /* Explicit active mode runs through the same model for every chassis type. */
        ChassisMotionSetMode(MC_ACTIVE_FIXED);
        cmd.velocity_g_mps[0]=.6f; cmd.velocity_g_mps[1]=0;
        for (unsigned k=0;k<400;++k) {
            now+=5000; fresh(&p,now); ChassisMotionStep(&port,now,&cmd);
        }
        ChassisMotionGetState(&state);
        assert(state.estimate_valid && state.dv_g[0]>.001f);
        near(state.cmd_g[0],state.ref_g[0]+state.dv_g[0],1e-6f);
        float target_body[]={cosf(theta)*state.cmd_g[0]-sinf(theta)*state.cmd_g[1],
                             sinf(theta)*state.cmd_g[0]+cosf(theta)*state.cmd_g[1],state.omega_ref};
        float expected_dps[4],expected_angle[4]={0};
        physical(&c.wheels,target_body,expected_dps,expected_angle);
        for (unsigned i=0;i<4;++i) {
            if (type==CK_SWERVE) {
                float speed=p.output.motor_dps[i]*unit(&c.wheels,i);
                near(speed*cosf(p.output.steer_rad[i]),target_body[0]-target_body[2]*c.wheels.y_m[i],1e-5f);
                near(speed*sinf(p.output.steer_rad[i]),target_body[1]+target_body[2]*c.wheels.x_m[i],1e-5f);
            } else near(p.output.motor_dps[i],expected_dps[i],.02f);
        }
        cmd.velocity_g_mps[0]=user[0]; cmd.velocity_g_mps[1]=user[1];
        /* Custom timeout takes effect; old commands must not be treated as fresh. */
        now+=5000; fresh(&p,now); p.input.cmd_age_us=16000;
        ChassisMotionStep(&port,now,&cmd); stopped(&p);
        p.input.cmd_age_us=0;
        now+=5000; fresh(&p,now); p.input.wheel_online_mask=7;
        ChassisMotionStep(&port,now,&cmd); stopped(&p);
        p.input.wheel_online_mask=15;
        now+=5000; fresh(&p,now); cmd.velocity_g_mps[0]=NAN;
        ChassisMotionStep(&port,now,&cmd); stopped(&p);
        cmd.velocity_g_mps[0]=user[0];
        if (type==CK_SWERVE) {
            ChassisMotionSetMode(MC_BASELINE); /* Steering validity also gates baseline drive. */
            now+=5000; fresh(&p,now); p.input.steer_time_us[1]-=16000;
            ChassisMotionStep(&port,now,&cmd); stopped(&p);
            now+=5000; fresh(&p,now); p.input.steer_online_mask=7;
            ChassisMotionStep(&port,now,&cmd); stopped(&p);
            p.input.steer_online_mask=15;
            now+=5000; fresh(&p,now); p.input.steer_rad[2]=NAN;
            ChassisMotionStep(&port,now,&cmd); stopped(&p);
        }
        p.received=0; now+=5000;
        ChassisMotionStep(&port,now,&cmd); stopped(&p);
        assert(p.writes>=44);
        assert(!ChassisMotionStep(NULL,now,&cmd));
        assert(!ChassisMotionConfigure(NULL));
        ChassisMotionStep(&port,now+5000,&cmd); stopped(&p);
        c.dt_min=NAN; assert(!ChassisMotionConfigure(&c));
    }
}
int main(void)
{
    kinematics(); interfaces();
    puts("PASS: physical omni/mecanum/swerve models, per-wheel calibration, limits, parallel/optimized steering; portable KF frames for chassis/gimbal IMU, wrap, command/driver failures and stale steering");
}
