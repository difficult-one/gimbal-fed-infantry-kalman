/* 只跳过含 HAL/通信类型的 robot_def.h；机械量与当前工程一致。
 * CI 主机数学测试使用这些初值，实车几何仍必须单独核验。 */
#define ROBOT_DEF_H
#define WHEEL_BASE 350
#define TRACK_WIDTH 300
#define RADIUS_WHEEL 76.0f
#define REDUCTION_RATIO_WHEEL 19.0f
#define CENTER_GIMBAL_OFFSET_X 0
#define CENTER_GIMBAL_OFFSET_Y 0
#define MC_HOST_TEST 1
