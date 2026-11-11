#ifndef CHASSIS_LOG_H
#define CHASSIS_LOG_H

void ChassisLogInit(void);
/* 由低优先级任务调用，格式化并排空诊断日志。 */
void ChassisMotionLogTask(void);

#endif
