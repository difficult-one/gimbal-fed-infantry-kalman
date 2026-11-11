#ifndef INS_H
#define INS_H
#include "bsp_dwt.h"
#include "imu_compensation.h"
typedef struct
{
    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    // 还需要增加角速度数据
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} attitude_t; // 最终解算得到的角度,以及yaw转动的总角度(方便多圈控制)
typedef struct
{
	float Accel[3];

    float Gyro[3];

    float TempWhenCali;
    float Temperature;

    float AccelScale;
    float GyroOffset[3];

    float gNorm;
} mpu_data_t;

typedef struct
{
	float q[4]; // 四元数估计值

    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // bodyframe在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;

    // IMU量测值
    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    uint8_t init;
} INS_t;
/* 用于修正安装误差的参数 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;
struct INS_Init_Config;
/* 配置由调用者持有，生命周期覆盖 INS_Task 的全部运行时间。 */
attitude_t *INS_Init(const struct INS_Init_Config *config);

void INS_Task(void);

/* 与旧 attitude_t 分离，避免改变云台控制和通信布局。
 * MotionAccel 已去重力，单位 m/s²；gyro_rad 为安装修正后的 rad/s。 */
typedef struct
{
    uint32_t seq, time_us;
    float q[4], xn[3];
    float accel_raw[3], motion_b[3], motion_n[3], gyro_rad[3];
    float center_n[3], yaw_deg; /* 同一 INS 帧内完成的杆臂校正，INS 导航系。 */
    float gravity_b[3], pitch_rad, roll_rad, attitude_chi_ratio;
    uint32_t clock_generation, recovery_good_us;
    uint8_t valid, center_valid, motion_ready, attitude_flags;
} INS_MotionSnapshot;

/* 短临界区复制完整已发布帧；尚未产生有效帧时返回 0。 */
uint8_t INS_GetMotionSnapshot(INS_MotionSnapshot *snapshot);

typedef enum {
    INS_SAMPLE_FRESH = 0,
    INS_SAMPLE_SPI_ERROR,
    INS_SAMPLE_SPI_BUSY,
    INS_SAMPLE_SPI_TIMEOUT,
    INS_SAMPLE_NOT_READY,
} INS_SampleStatus;

typedef enum {
    INS_START_PENDING = 0,
    INS_START_CONFIG_FAILED,
    INS_START_ATTITUDE_FAILED,
    INS_START_READY,
} INS_StartStatus;

typedef struct {
    uint32_t attempt_seq, accepted_seq, read_time_us; /* SPI completion time, including failed attempts. */
    uint32_t timeout_count, error_count, busy_count, not_ready_count, gap_count;
    INS_SampleStatus status;
    INS_StartStatus startup_status;
} INS_SampleDebug;

void INS_GetSampleDebug(INS_SampleDebug *debug);

/* 仅初始化阶段由底盘配置杆臂；不改变 attitude_t 或公共通信布局。 */
void INS_SetLeverArm(const float sensor_m[3]);
/* 独立物理条件与编码器辅助：底盘和云台各发布自己拥有的观测，不互读私有变量。 */
void INS_SetChassisStill(uint32_t time_us, uint8_t still, uint32_t reason);
void INS_UpdateYawBias(const IMUYawBiasInput *input, uint32_t encoder_time_us);
/* 与控制快照分离，避免扩大底盘二进制状态布局。刚体统计为最近完成的秒窗，
 * aid 为当前累计诊断，各自有时间戳；不能当作与 STATUS 严格同帧。 */
void INS_GetCompensationDebug(IMURigidStats *rigid, IMUYawBiasDebug *aid);

/* 启动零偏与编码器辅助残余零偏分开记录，不覆盖启动标定结果。
 * status: 0=尚未完成，1=本次静态标定通过，2=超时退回编译期固定零偏。
 * 温度单位 °C，gyro_offset_z 单位 rad/s。 */
typedef struct {
    float temperature, calibration_temperature, gyro_offset_z;
    float yaw_trim_z, yaw_encoder_deg;
    uint32_t trim_updates, encoder_time_us;
    uint8_t status, trim_status;
} INS_CalibrationDebug;
void INS_GetCalibrationDebug(INS_CalibrationDebug *snapshot);

#endif 
