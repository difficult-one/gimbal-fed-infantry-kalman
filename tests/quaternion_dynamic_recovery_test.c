#include "quaternion_ekf.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc,char **argv)
{
    assert(argc==2);
    float dt=(float)atof(argv[1]), q[4]={1,0,0,0};
    IMU_QuaternionEKF_Init(q,10,.001f,1000000,1,0);
    for(unsigned k=0;k<(unsigned)(4/dt);++k)
        IMU_QuaternionEKF_Update(0,0,0,0,0,9.81f,dt);
    assert(QEKF_INS.ConvergeFlag);
    /* Horizontal acceleration barely changes |a|, so the old StableFlag is
     * true. Rejected acceleration must not be forced into gravity after 50
     * samples when the platform supplies no independent stillness permission. */
    for(unsigned k=0;k<(unsigned)(2/dt);++k) {
        IMU_QuaternionEKF_Update(0,0,0,1.5f,0,9.81f,dt);
        assert(QEKF_INS.StableFlag && QEKF_INS.ConvergeFlag);
        assert(QEKF_INS.IMU_QuaternionEKF.SkipEq5);
        assert(fabsf(QEKF_INS.Roll)<1);
    }
    IMU_QuaternionEKF_SetGravityRecoveryAllowed(1);
    for(unsigned k=0;k<(unsigned)(.3f/dt);++k)
        IMU_QuaternionEKF_Update(0,0,0,1.5f,0,9.81f,dt);
    assert(QEKF_INS.GravityRecoveryTime>.28f && QEKF_INS.ConvergeFlag);
    IMU_QuaternionEKF_SetGravityRecoveryAllowed(0);
    assert(QEKF_INS.GravityRecoveryTime==0);
    /* A real gravity tilt can still be reacquired after explicit stillness,
     * with a time qualification that is independent of the sampling rate. */
    IMU_QuaternionEKF_SetGravityRecoveryAllowed(1);
    float elapsed=0, first_recovery=0;
    float before_tilt=fabsf(QEKF_INS.Roll);
    for(unsigned k=0;k<(unsigned)(1/dt);++k) {
        elapsed+=dt;
        IMU_QuaternionEKF_Update(0,0,0,9.81f*sinf(.15f),0,9.81f*cosf(.15f),dt);
        if(!QEKF_INS.ConvergeFlag && !first_recovery) first_recovery=elapsed;
        if(elapsed<.49f) assert(QEKF_INS.ConvergeFlag && QEKF_INS.IMU_QuaternionEKF.SkipEq5);
    }
    assert(first_recovery>=.49f && first_recovery<.53f);
    assert(fabsf(QEKF_INS.Roll)>before_tilt+.01f); /* Correction resumes; no angle jump required. */
    IMU_QuaternionEKF_SetGravityRecoveryAllowed(0);
    /* Permission loss during forced reacquisition must stop that override too. */
    QEKF_INS.GravityRecoveryActive=1; QEKF_INS.ConvergeFlag=0;
    IMU_QuaternionEKF_Update(0,0,0,-2.0f,0,9.81f,dt);
    assert(QEKF_INS.IMU_QuaternionEKF.SkipEq5 && !QEKF_INS.ConvergeFlag);
    printf("PASS: dynamic gravity override blocked; explicit static reacquisition after %.3fs at dt=%.3fs; interrupted qualification cleared\n",first_recovery,dt);
    return 0;
}
