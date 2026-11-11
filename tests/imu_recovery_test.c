#include "imu_compensation.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    IMURecoveryState s={.pending=1};
    for (unsigned i=0;i<499;++i) assert(!IMURecoveryUpdate(&s,0,1000,1,1));
    assert(IMURecoveryUpdate(&s,0,1000,1,1));
    assert(IMURecoveryUpdate(&s,0,1000,1,0)); /* Normal dynamics after release. */
    assert(!IMURecoveryUpdate(&s,1,1000,1,1)); /* Clock repair despite normal dt. */
    for (unsigned i=0;i<450;++i) assert(!IMURecoveryUpdate(&s,1,1000,1,1));
    assert(!IMURecoveryUpdate(&s,1,1000,1,0)); assert(s.good_us==0);
    for (unsigned i=0;i<499;++i) assert(!IMURecoveryUpdate(&s,1,1000,1,1));
    assert(IMURecoveryUpdate(&s,1,1000,1,1));
    assert(!IMURecoveryUpdate(&s,1,0,1,1));
    assert(!IMURecoveryUpdate(&s,1,1000,0,1));
    assert(!IMURecoveryUpdate(&s,1,25000,1,1));
    puts("PASS: IMU recovery requires 500 ms consecutive settled observations after clock/input faults");
    return 0;
}
