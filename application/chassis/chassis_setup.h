#ifndef CHASSIS_SETUP_H
#define CHASSIS_SETUP_H

#include "chassis_motion.h"

void ChassisMotionInit(void);
/* Optional board override, after project calibration and before validation. */
void ChassisMotionCustomizeConfig(ChassisMotionConfig *config);

#endif
