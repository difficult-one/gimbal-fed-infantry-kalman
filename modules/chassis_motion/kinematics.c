#include "kinematics.h"
#include <math.h>
#include <string.h>

#define CK_PI 3.14159265358979323846f

void ChassisKinematicsProfile(ChassisKinematicsConfig *c, ChassisDriveType type,
                              float base, float track, float radius, float ratio)
{
    memset(c, 0, sizeof(*c));
    c->type = type;
    c->steer_cosine_compensation = 1u;
    const float sx[4] = {1,1,-1,-1}, sy[4] = {1,-1,1,-1};
    const float omni_x[4] = {-1,-1,1,1}, omni_y[4] = {1,-1,1,-1};
    const float mecanum_y[4] = {-1,1,1,-1};
    for (unsigned i=0; i<4; ++i) {
        c->x_m[i] = sx[i]*base*.5f; c->y_m[i] = sy[i]*track*.5f;
        c->radius_m[i] = radius; c->gear_ratio[i] = ratio;
        c->motor_sign[i] = 1; c->max_motor_dps[i] = 36000;
        float dx = type==CK_OMNI_X ? omni_x[i]*.7071067812f : 1;
        float dy = type==CK_OMNI_X ? omni_y[i]*.7071067812f : mecanum_y[i];
        c->rows[i][0]=dx; c->rows[i][1]=dy;
        c->rows[i][2]=-dx*c->y_m[i]+dy*c->x_m[i];
    }
}

uint8_t ChassisKinematicsInit(ChassisKinematics *m, const ChassisKinematicsConfig *c)
{
    memset(m, 0, sizeof(*m));
    if (!c || c->type < CK_OMNI_X || c->type > CK_CUSTOM) return 0;
    m->config = *c;
    float a[8][3]={{0}}, augmented[3][6]={{0}};
    unsigned n = c->type==CK_SWERVE ? 8 : 4;
    for (unsigned i=0; i<4; ++i) {
        if (!isfinite(c->radius_m[i]) || c->radius_m[i]<=0 ||
            !isfinite(c->gear_ratio[i]) || c->gear_ratio[i]<=0 ||
            !isfinite(c->max_motor_dps[i]) || c->max_motor_dps[i]<=0 ||
            !isfinite(c->motor_sign[i]) || fabsf(c->motor_sign[i])!=1 ||
            !isfinite(c->x_m[i]) || !isfinite(c->y_m[i])) return 0;
        float unit=CK_PI/180*c->radius_m[i]/c->gear_ratio[i];
        if (!isfinite(unit) || unit<=0 || !isfinite(unit*c->max_motor_dps[i])) return 0;
        if (c->type==CK_SWERVE) {
            a[2*i][0]=1; a[2*i][2]=-c->y_m[i];
            a[2*i+1][1]=1; a[2*i+1][2]=c->x_m[i];
        } else {
            for (unsigned k=0; k<3; ++k) {
                if (!isfinite(c->rows[i][k])) return 0;
                a[i][k]=c->rows[i][k];
            }
        }
    }
    for (unsigned r=0;r<3;++r) {
        for (unsigned col=0;col<3;++col)
            for (unsigned i=0;i<n;++i) augmented[r][col]+=a[i][r]*a[i][col];
        augmented[r][r+3]=1;
        for (unsigned col=0;col<6;++col) if (!isfinite(augmented[r][col])) return 0;
    }
    for (unsigned col=0;col<3;++col) {
        unsigned pivot=col;
        for (unsigned r=col+1;r<3;++r)
            if (fabsf(augmented[r][col])>fabsf(augmented[pivot][col])) pivot=r;
        if (fabsf(augmented[pivot][col])<1e-8f) return 0;
        for (unsigned k=0;k<6;++k) {
            float t=augmented[col][k]; augmented[col][k]=augmented[pivot][k]; augmented[pivot][k]=t;
        }
        float scale=augmented[col][col];
        for (unsigned k=0;k<6;++k) augmented[col][k]/=scale;
        for (unsigned r=0;r<3;++r) if (r!=col) {
            float factor=augmented[r][col];
            for (unsigned k=0;k<6;++k) augmented[r][k]-=factor*augmented[col][k];
        }
    }
    for (unsigned r=0;r<3;++r)
        for (unsigned i=0;i<n;++i)
            for (unsigned k=0;k<3;++k) m->inverse[r][i]+=augmented[r][k+3]*a[i][k];
    for (unsigned r=0;r<3;++r)
        for (unsigned i=0;i<n;++i) if (!isfinite(m->inverse[r][i])) return 0;
    m->valid=1;
    return 1;
}

float ChassisKinematicsMpsPerDps(const ChassisKinematics *m, unsigned i)
{
    return CK_PI/180*m->config.radius_m[i]/m->config.gear_ratio[i];
}

uint8_t ChassisKinematicsObserve(const ChassisKinematics *m, const float dps[4],
                                 const float steer[4], float twist[3])
{
    memset(twist,0,3*sizeof(float));
    if (!m->valid) return 0;
    float v[8]={0};
    for (unsigned i=0;i<4;++i) {
        if (!isfinite(dps[i])) return 0;
        float speed=dps[i]*m->config.motor_sign[i]*ChassisKinematicsMpsPerDps(m,i);
        if (m->config.type==CK_SWERVE) {
            if (!steer || !isfinite(steer[i])) return 0;
            v[2*i]=speed*cosf(steer[i]); v[2*i+1]=speed*sinf(steer[i]);
        } else v[i]=speed;
    }
    unsigned n=m->config.type==CK_SWERVE ? 8 : 4;
    for (unsigned r=0;r<3;++r) {
        for (unsigned i=0;i<n;++i) twist[r]+=m->inverse[r][i]*v[i];
        if (!isfinite(twist[r])) { memset(twist,0,3*sizeof(float)); return 0; }
    }
    return 1;
}

static float WheelSpeed(const ChassisKinematics *m, const float t[3], unsigned i,
                        float *vx, float *vy)
{
    if (m->config.type==CK_SWERVE) {
        *vx=t[0]-t[2]*m->config.y_m[i]; *vy=t[1]+t[2]*m->config.x_m[i];
        return hypotf(*vx,*vy);
    }
    return m->config.rows[i][0]*t[0]+m->config.rows[i][1]*t[1]+m->config.rows[i][2]*t[2];
}

uint8_t ChassisKinematicsCommand(const ChassisKinematics *m, const float t[3],
                                 const float steer[4], ChassisWheelCommand *out)
{
    memset(out,0,sizeof(*out));
    if (!m->valid || !isfinite(t[0]) || !isfinite(t[1]) || !isfinite(t[2])) return 0;
    if (m->config.type==CK_SWERVE)
        for (unsigned i=0;i<4;++i) if (!steer || !isfinite(steer[i])) return 0;
    for (unsigned i=0;i<4;++i) {
        float vx=0,vy=0,speed=WheelSpeed(m,t,i,&vx,&vy);
        if (!isfinite(speed)) { memset(out,0,sizeof(*out)); return 0; }
        if (m->config.type==CK_SWERVE) {
            out->steer_rad[i]=steer[i];
            if (speed>1e-5f) {
                float delta=remainderf(atan2f(vy,vx)-steer[i],2*CK_PI);
                if (fabsf(delta)>CK_PI*.5f) {
                    delta=remainderf(delta+CK_PI,2*CK_PI); speed=-speed;
                }
                out->steer_rad[i]=steer[i]+delta;
                if (m->config.steer_cosine_compensation) speed*=fmaxf(0,cosf(delta));
            }
        }
        out->motor_dps[i]=speed*m->config.motor_sign[i]/ChassisKinematicsMpsPerDps(m,i);
        if (!isfinite(out->motor_dps[i]) || !isfinite(out->steer_rad[i])) {
            memset(out,0,sizeof(*out)); return 0;
        }
    }
    return 1;
}

float ChassisKinematicsScale(const ChassisKinematics *m, const float t[3])
{
    if (!m->valid || !isfinite(t[0]) || !isfinite(t[1]) || !isfinite(t[2])) return 0;
    float scale=1;
    for (unsigned i=0;i<4;++i) {
        float vx=0,vy=0,speed=fabsf(WheelSpeed(m,t,i,&vx,&vy));
        if (!isfinite(speed)) return 0;
        float max=m->config.max_motor_dps[i]*ChassisKinematicsMpsPerDps(m,i);
        if (speed>max) scale=fminf(scale,max/speed);
    }
    return scale;
}

float ChassisKinematicsTranslationScale(const ChassisKinematics *m, const float v[2], float w)
{
    if (!m->valid || !isfinite(v[0]) || !isfinite(v[1]) || !isfinite(w)) return 0;
    float t[3]={0,0,w};
    if (ChassisKinematicsScale(m,t)<1) return 0;
    if (m->config.type!=CK_SWERVE) {
        float scale=1;
        for (unsigned i=0;i<4;++i) {
            float base=m->config.rows[i][2]*w;
            float delta=m->config.rows[i][0]*v[0]+m->config.rows[i][1]*v[1];
            float limit=m->config.max_motor_dps[i]*ChassisKinematicsMpsPerDps(m,i);
            if (delta>0) scale=fminf(scale,(limit-base)/delta);
            if (delta<0) scale=fminf(scale,(-limit-base)/delta);
        }
        return fmaxf(0,fminf(1,scale));
    }
    /* Each module's norm constraint is convex; feasible segment contains alpha=0. */
    float lo=0,hi=1;
    t[0]=v[0]; t[1]=v[1];
    if (ChassisKinematicsScale(m,t)>=1) return 1;
    for (unsigned k=0;k<24;++k) {
        float mid=(lo+hi)*.5f; t[0]=v[0]*mid; t[1]=v[1]*mid;
        if (ChassisKinematicsScale(m,t)>=1) lo=mid; else hi=mid;
    }
    return lo;
}
