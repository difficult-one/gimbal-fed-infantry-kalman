#include "config.h"
#include "math_units.h"

const TaskConfig g_task_config = {
    .motor      = {0, 256, 2},   // 电机调度，普通优先级，延时 2 ticks。
    .daemon     = {0, 128, 10},  // 设备离线守护，普通优先级，延时 10 ticks。
    .robot      = {0, 1024, 5},  // 整车应用，普通优先级，延时 5 ticks。
    .ins        = {0, 1024, 1},  // INS 轮询，普通优先级，延时 1 tick。
    .motion_log = {-2, 512, 10}, // 低优先级 RTT 格式化，栈 512 words，延时 10 ticks。
};

const LinkConfig g_link_config = {
    .remote_uart = 1,  // DR16 DBUS，USART1 反相输入。
    .vision_uart = 3,  // 视觉通信，USART3。
    .vt02_uart = 8,    // 图传键鼠，UART8。
    .referee_uart = 6, // 裁判收发，USART6。
#ifdef GIMBAL_BOARD
    .board_can = 1,      // 云台板板间链路，CAN1。
    .board_tx_id = 0x312, // 云台到达底盘的命令帧。
    .board_rx_id = 0x311, // 底盘到达云台的反馈帧。
#else
    .board_can = 2,      // 底盘板板间链路，CAN2。
    .board_tx_id = 0x311, // 底盘到达云台的反馈帧。
    .board_rx_id = 0x312, // 云台到达底盘的命令帧。
#endif
    .board_daemon_count = 10u, // 板间链路失联门槛，守护任务周期数。
    .chassis_motor_can = 1,    // 四轮驱动电机，CAN1。
    .super_cap_can = 2,       // 超级电容，CAN2。
    .super_cap_tx_id = 0x302, // 超电控制帧。
    .super_cap_rx_id = 0x301, // 超电反馈帧。
};

const InputConfig g_input_config = {
    .chassis_dead_band = 10, // 底盘摇杆死区，原始量，开区间。
    .gimbal_dead_band = 10,  // 云台摇杆死区，原始量，闭区间。
    .friction_dial = -150,   // 向上拨过此值启动摩擦轮，原始量。
    .loader_dial = -350,     // 向上拨过此值开始拨弹，原始量。
    .stop_dial = 300,        // 向下拨过此值急停，原始量。
    .rc_yaw_scale = 0.005f,   // 左摇杆横轴，每原始单位、每控制周期的 yaw 增量，deg。
    .rc_pitch_scale = 0.001f, // 左摇杆竖轴，每原始单位、每控制周期的 pitch 增量，deg。
    .rc_vx_scale = -30.0f,   // 右摇杆竖轴，前进正向的旧底盘命令倍率。
    .rc_vy_scale = 30.0f,    // 右摇杆横轴，横移的旧底盘命令倍率。
    .rc_shoot_rate = 8,      // 遥控连续发射频率，发/秒。
    .video_shoot_rate = 30,  // 图传连续发射频率，发/秒。
    .video_speed_scale = 3000.0f, // 图传键盘满指令，旧底盘速度单位。
    .video_force_x_step = 100.0f, // 图传键盘前后力阶跃，N。
    .video_force_y_step = 100.0f, // 图传键盘左右力阶跃，N。
    .video_force_mz_scale = 0.05f, // 图传鼠标到 yaw 力矩倍率，N·m/原始单位。
    .video_force_max = 120.0f, // 图传力分量限值，N。
    .video_mz_max = 5.0f,     // 图传 yaw 力矩限值，N·m。
    .mouse_full_scale = 660, // 鼠标输入归一化分母，原始量。
    .mouse_angle_deg = 10,   // 每控制周期的满量程角增量，deg。
    .yaw_feedback_max_age_us = 20000u, // 完整精度 yaw 编码器反馈时效，µs。
};

#define LOADER_SHOT_ANGLE_DEG 45    // 拨盘输出轴每发转角，deg。
#define LOADER_REDUCTION 36.0f     // M2006 拨盘电机减速比。

const GimbalConfig g_gimbal_config = {
    .motor_can = 2, // yaw、pitch GM6020 接 CAN2。
    .yaw_align_ecd = 370, // 云台正前方与底盘正前方对齐时的 yaw 编码器计数。
    .pitch_horizon_ecd = 3412, // 炮管水平时的 pitch 编码器计数。
    .pitch_min_deg = -30, // INS pitch 目标下限，deg。
    .pitch_max_deg = 30,  // INS pitch 目标上限，deg。
    .yaw_max_track_error_deg = 60.0f, // yaw 目标与反馈最大差值，deg。
    .gyro_yaw_direction = 1,   // +1 表示 INS yaw 与云台 yaw 同向。
    .gyro_pitch_direction = 1, // +1 表示 INS pitch 与云台 pitch 同向。
    .gyro_roll_direction = 1,  // +1 表示 INS roll 与云台 roll 同向。
    .yaw_aid_max_age_us = 20000u, // yaw 零偏辅助的命令和反馈最大样本年龄，µs。
    .yaw = {
        .can_init_config = {
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8,
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 陀螺仪速度反馈含高频噪声,Kp过大会把噪声放大成电流纹波(抖动)
                .Ki = 200,  // 积分用于克服摩擦,过大易在稳态震荡
                .Kd = 0,   // 关键:微分先行对陀螺仪噪声极敏感,是抖动主因,置0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 12000.0f, // DJI CAN 原始量；frame 7 自旋实测旧积分限额 3000 导致约 18° 偏角。
                .MaxOut = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    },
    .pitch = {
        .can_init_config = {
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 原50:速度反馈已换算为°/s,阻尼生效,降低起调
                .Ki = 300, // 原350:放慢积分减少超调
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000, // 原2500:增大以提供抵住炮管重力的稳态力矩
                .MaxOut = 30000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE, // 反转pitch电机转向,消除正反馈
        },
        .motor_type = GM6020,
    },
};

const ShootConfig g_shoot_config = {
    .motor_can = 2, // 左右摩擦轮 M3508、拨盘 M2006 接 CAN2。
    .friction_right_id = 4, // 右摩擦轮电机 ID。
    .friction_right_direction = MOTOR_DIRECTION_NORMAL, // 右轮沿电机正方向出弹。
    .loader_reduction = LOADER_REDUCTION,
    .loader_shot_angle_deg = LOADER_SHOT_ANGLE_DEG,
    .loader_tolerance_deg = 5.0f, // 单发到位允许的电机轴角误差，deg。
    .shots_per_turn = 8, // 拨盘输出轴每圈装弹数。
    .friction_15_dps = 4000, // 弹速 15 m/s 档的电机轴速度，deg/s。
    .friction_18_dps = 6000, // 弹速 18 m/s 档的电机轴速度，deg/s。
    .friction_30_dps = 9000, // 弹速 30 m/s 档的电机轴速度，deg/s。
    .friction_default_dps = 1000, // 无对应弹速档位时的电机轴速度，deg/s。
    .friction = {
        .can_init_config.tx_id = 3,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7,
                .Ki = 0.1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP, // M3508/M2006电调内部已有电流环,软件电流环衰减指令且易振荡,速度环输出直接作为电流指令
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508,
    },
    .loader = {
        .can_init_config = {
            .tx_id = 7,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 5000,
            },
            .speed_PID = {
                .Kp = 10,
                .Ki = 1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0.7,
                .Ki = 0.1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    },
    .jam = {
        .current_threshold   = 3000.0f,   // M2006/C610电流原始值; 必须低于速度环MaxOut=5000, 再按实测lCur调整
        .suspect_timeout_ms  = 150.0f,    // 电流持续超阈值多久才确认卡弹
        .handling_timeout_ms = 800.0f,    // 卡弹回退处理最长时间, 后续按实测回退时间调整
        .reverse_angle       = LOADER_SHOT_ANGLE_DEG * LOADER_REDUCTION, // 卡弹后回退一发对应的电机轴角度
        .speed_threshold     = 200.0f,    // 判定卡弹时允许的最大电机轴速度(度/秒)
        .min_angle_error     = 100.0f,    // 角度环下要求的最小目标误差(电机轴,度)
        .min_speed_reference = 100.0f,    // 速度环下要求的最小速度指令(度/秒)
    },
};

/* 关闭时省去对应的 JScope 编码和缓冲区。通道 1 与 WIMU 采集共用。 */
#define SHOOT_JSCOPE_ENABLED 0  // 置 1 输出摩擦轮波形。
#define GIMBAL_JSCOPE_ENABLED 0 // 置 1 输出云台姿态与角速度。

#if SHOOT_JSCOPE_ENABLED && WHEEL_IMU_CAPTURE
#error "Shoot JScope and wheel/IMU capture share RTT channel 1"
#endif

const LogConfig g_log_config = {
    .shoot_jscope_init = SHOOT_JSCOPE_ENABLED ? RTTDebugJScope4FInit : NULL,
    .shoot_jscope_send = SHOOT_JSCOPE_ENABLED ? RTTDebugJScope4FSend : NULL,
    .gimbal_jscope_init = GIMBAL_JSCOPE_ENABLED ? RTTDebugJScope6FInit : NULL,
    .gimbal_jscope_send = GIMBAL_JSCOPE_ENABLED ? RTTDebugJScope6FSend : NULL,
    .shoot_debug_enabled = 0, // 置 1 输出拨盘和卡弹文本。
    .shoot_debug_period = 20, // 拨盘文本抽样周期，发射控制周期数。
    .shoot = {
        .channel = 1, // 摩擦轮波形 RTT 通道。
        .buffer_name = "JScope_t4f4f4f4f",
        .csv_header = "time_us,left_ref,left_speed,right_ref,right_speed",
        .csv_enable = 0, // 置 1 同时输出 CSV。
        .csv_channel = &g_chassis_log_config.text_rtt_channel,
        .csv_div = 1, // CSV 抽样周期，发射控制周期数。
    },
    .gimbal = {
        .channel = 2, // 云台波形 RTT 通道。
        .buffer_name = "JScope_t4f4f4f4f4f4",
        .csv_header = "time_us,roll,pitch,yaw,gyro_x,gyro_y,gyro_z",
        .csv_enable = 0, // 置 1 同时输出 CSV。
        .csv_channel = &g_chassis_log_config.text_rtt_channel,
        .csv_div = 1, // CSV 抽样周期，云台控制周期数。
    },
};
/* 允许固件调试构建关闭编码器辅助残余零偏学习。 */
#ifndef INS_YAW_BIAS_AID_ENABLE
#define INS_YAW_BIAS_AID_ENABLE 1
#endif

const INS_Init_Config g_imu_config = {
    .heater_pid = {
        .MaxOut = 100,        // 加热 PWM 输出上限，百分比。
        .IntegralLimit = 15,  // 积分限幅，PWM 百分比。
        .DeadBand = 0,        // 恒温误差死区，°C。
        .Kp = 50, .Ki = 1, .Kd = 0, // 温控 PID，输入温差 °C、输出 PWM 百分比。
        .Improve = PID_Integral_Limit,
    },
    .heater_target_c = 40,        // IMU 恒温目标，°C。
    .heater_period_ms = 0.1,       // 加热 PWM 周期，ms。
    .heater_initial_duty = 50,    // PWM 注册初始占空比，百分比；随后先置零。
    .scale = {1, 1, 1},           // INS 机体系陀螺 XYZ 轴比例。
    .yaw_deg = 0,                 // 绕 Z 的安装角修正，deg。
    .pitch_deg = 0,               // 绕 Y 的安装角修正，deg。
    .roll_deg = 0,                // 绕 X 的安装角修正，deg。
    .ekf_quaternion_noise = 10,   // 姿态四元数过程噪声初值。
    .ekf_bias_noise = 0.001,      // 陀螺零偏过程噪声初值。
    .ekf_accel_noise = 1000000,   // 加速度观测噪声初值。
    .ekf_fading = 1,              // EKF 遗忘因子，1 保留全部历史权重。
    .ekf_accel_lpf_s = 0,         // EKF 加速度低通时间常数，s，0 关闭。
    .motion_lpf_s = 0.0085f, // 去重力加速度低通时间常数，s。
    .sample_gap_us = 20000u,      // 采样间隙超限门槛，µs。
    .yaw_aid_max_age_us = 20000u, // 底盘静止反馈有效期，µs。
    .initial_attitude_samples = 100u,          // 初始姿态需要的独立加速度样本数。
    .initial_attitude_timeout_us = 300000u,   // 初始姿态采样时间上限，µs。
    .initial_attitude_poll_s = 0.001f,         // 初始姿态采样轮询间隔，s。
    .calibration_samples = 6000u,             // 静置标定需要的连续样本数。
    .calibration_timeout_us = 15000000u,      // 静置标定总时间上限，µs。
    .calibration_retry_s = 0.005f,             // 静置标定重启窗口前的间隔，s。
    .calibration_poll_s = 0.0005f,            // 静置标定采样轮询间隔，s。
    .calibration_accel_span = 0.5f,           // 静置窗口加速度模长变化上限，m/s²。
    .calibration_gyro_span = 0.15f,           // 静置窗口每轴角速度变化上限，rad/s。
    .calibration_gravity = 9.8f,              // 合理性检查的重力参考，m/s²。
    .calibration_gravity_tolerance = 0.5f,    // 标定重力模长允许误差，m/s²。
    .gyro_offset_limit = {0.04f, 0.04f, 0.04f}, // INS XYZ 启动零偏合理性上限，rad/s。
    .fallback_gyro_offset = {0.037452f, 0.003360f, 0.008532f}, // 标定失败时 INS XYZ 零偏，rad/s。
    .fallback_gravity = 9.605101f,            // 标定失败时的加速度模长基准，m/s²。
    .fallback_temperature_c = 40.0f,         // 标定失败时记录的标定温度，°C。
    .yaw_bias_aid_enabled = INS_YAW_BIAS_AID_ENABLE, // 置 1 启用编码器辅助零偏学习。
};

/* 底盘车辆参数。尺寸按 2026-11-10 确认的 152 mm 外径车轮标定。 */
#define WHEEL_BASE 350                       /* 纵向轴距，mm。 */
#define TRACK_WIDTH 300                      /* 横向轮距，mm。 */
#define CENTER_GIMBAL_OFFSET_X 0              /* 云台轴心相对底盘几何中心，+x 向前，mm。 */
#define CENTER_GIMBAL_OFFSET_Y 0              /* 云台轴心相对底盘几何中心，+y 向右，mm。 */
#define RADIUS_WHEEL 76.0f                   /* 轮子半径，mm。 */
#define REDUCTION_RATIO_WHEEL 19.0f          /* 转子转速 / 轮端转速。 */
#define CHASSIS_FORCE_CURRENT_LIMIT_RAW 8000.0f /* 力控原始电流限制，DJI CAN raw。 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)
/* 集中标定区。以下为工程初值，不能替代调试手册 S01～S14 的实测。 */
#define MC_CONFIG_VERSION 0x20261111u
#define MC_FRAME_REVISION 19u /* 六行核心快照与低频详细诊断/配置，MCL1状态布局不变。 */
#define MC_DEFAULT_MODE MC_SHADOW /* 估计/候选补偿只用于观测；实车验证前不向轮速追加。 */
#ifndef MC_WHEEL_LIMIT_ROTATION_PRIORITY
/* 2026-11-08 平移手感：轮速不足时等比例降低平移和旋转；原地自旋保持原目标。
 * 设1恢复严格旋转优先，但当前高速目标只剩约0.11～0.15 m/s平移余量。 */
#define MC_WHEEL_LIMIT_ROTATION_PRIORITY 0
#endif
#define MC_COS45 0.7071067812f
#define MC_WHEEL_RADIUS_M (RADIUS_WHEEL * 0.001f)
#define MC_GEAR_RATIO REDUCTION_RATIO_WHEEL
#define MC_ARM_M ((WHEEL_BASE + TRACK_WIDTH) * 0.0005f * MC_COS45)
#define MC_MPS_PER_DPS (DEGREE_2_RAD * MC_WHEEL_RADIUS_M / MC_GEAR_RATIO)
/* 保留公共协议旧单位；仅速控入口换算。此映射使零补偿稳态轮速与旧链一致。 */
#define MC_LEGACY_W_SCALE (((WHEEL_BASE + TRACK_WIDTH) * 0.5f * DEGREE_2_RAD) * MC_MPS_PER_DPS / MC_ARM_M)
/* 旧旋转命令单位；保留当前4000对照档，145 mm外径对应约6.575 rad/s。
 * 若改6000，换算约9.863 rad/s，将先受下方9 rad/s总上限约束。 */
#define MC_SPIN_LEGACY_WZ 4000.0f
/* frame=11 四轮共同相位滞后约7.8deg，相对编码器约10.18rad/s，等效13.4ms。
 * 只提前小陀螺的输出变换；姿态/KF和静态方向标定仍使用实测角。
 * 这是本车日志支持的初始值，实车验证前不视作完成延迟辨识；置0回退。 */
#ifndef MC_SPIN_PHASE_LEAD_S
#define MC_SPIN_PHASE_LEAD_S 0.0135f
#endif
#define MC_SPIN_PHASE_MAX_RAD 0.20f
#define MC_THETA_RATE_LPF_S 0.020f
#define MC_THETA_RATE_MAX_RPS 40.0f

/* 顺序恒为 LF(1)、RF(4)、LB(2)、RB(3)，单位 m/m、m。
 * 保留旧电机输出代数；其平移 Y 与姿态右手 B 相反，Update/hat 返回处显式转换。 */
#define MC_J_ROWS { {-MC_COS45, MC_COS45, MC_ARM_M}, \
                    { MC_COS45, MC_COS45, MC_ARM_M}, \
                    {-MC_COS45,-MC_COS45, MC_ARM_M}, \
                    { MC_COS45,-MC_COS45, MC_ARM_M} }
#define MC_ENCODER_SIGNS {1.0f, 1.0f, 1.0f, 1.0f}
#define MC_WHEEL_RADII {MC_WHEEL_RADIUS_M, MC_WHEEL_RADIUS_M, MC_WHEEL_RADIUS_M, MC_WHEEL_RADIUS_M}
/* 2026-11-10 三次152 mm配置的稳定自旋日志反推；仅ONE_BOARD OMNI_X接入。
 * 零均值微调（motor dps），轮序LF/RF/LB/RB；方向随旋转方向翻转。 */
#define MC_SPIN_WHEEL_TRIM_DPS {66.0f, -107.0f, 29.0f, 12.0f}

/* N 为 INS 导航系的 (x,-y,-z)，仍为右手系。
 * 姿态 B/G 是右手系；旧命令/J 则保留历史代数，二者不能当作同一个旋转坐标系。
 * 第十三次落地前后动作确认 X 航向不应额外加 pi；第十四次侧移确认
 * J 逆解的平移向量须先变为 (x,-y)，再按 psi_BN 投影到 N。 */
#ifndef MC_IMU_ON_GIMBAL
#ifdef CHASSIS_BOARD
#define MC_IMU_ON_GIMBAL 0
#else
#define MC_IMU_ON_GIMBAL 1
#endif
#endif
#if defined(CHASSIS_BOARD) && MC_IMU_ON_GIMBAL
#error "Dual-board chassis must use its local chassis IMU"
#endif
/* 用户确认 A 板随 pitch 转动，且 yaw/pitch 两轴相交；使用完整刚体杆臂。 */
#ifndef MC_IMU_FOLLOWS_PITCH
#define MC_IMU_FOLLOWS_PITCH MC_IMU_ON_GIMBAL
#endif
/* Board IMU lever in INS sensor axes, relative to chassis center; measure on target. */
#ifndef MC_CHASSIS_IMU_LEVER_X_M
#define MC_CHASSIS_IMU_LEVER_X_M 0.0f
#endif
#ifndef MC_CHASSIS_IMU_LEVER_Y_M
#define MC_CHASSIS_IMU_LEVER_Y_M 0.0f
#endif
#ifndef MC_CHASSIS_IMU_LEVER_Z_M
#define MC_CHASSIS_IMU_LEVER_Z_M 0.0f
#endif
/* 安装杆臂按物理测量；融合航向按姿态 G 的 +x。二者不可混作同一前向。
 * 此处恢复为 0，不改变公共 INS、编码器、theta_BG 或遥控到电机的方向。 */
#define MC_IMU_HEADING_OFFSET_RAD 0.0f
/* 2026-10-25 实车只动 yaw：IMU 航向与编码器 offset_angle 反向。
 * 仅将旧协议角转换为本模块 theta_BG=psi_GN-psi_BN，不改公共角度及跟随控制器。 */
#define MC_YAW_ENCODER_SIGN (-1.0f)
/* 允许加速度进入估计 KF；保留旧宏名供主机测试/旧配置覆盖。
 * 置 1 不等于全工况验收通过；默认仅观测，不向轮速追加补偿。 */
#ifndef MC_IMU_ACCEL_VALIDATED
#define MC_IMU_ACCEL_VALIDATED 1
#endif
#define MC_ACCEL_BIAS_N_X 0.0f // 转为本模块固定 N 系后的加速度零偏，m/s²
#define MC_ACCEL_BIAS_N_Y 0.0f
/* 平面安装偏置（G/B 系 x前/y右）；pitch 随动时由下方完整三维杆臂替代。
 * yaw 中心相对底盘参考中心 O 的偏置另在 B 系给出。 */
#ifndef MC_IMU_OFFSET_X_M
/* 2026-10-26 用户安装图：轴交点指向 A 板 IMU，沿跟随正前方 +x 约 8.4 cm。 */
#define MC_IMU_OFFSET_X_M 0.084f
#endif
#ifndef MC_IMU_OFFSET_Y_M
#define MC_IMU_OFFSET_Y_M 0.0f
#endif
/* 用户确认 IMU 高于两轴交点 3.7 cm。
 * 2026-10-26 第四次日志：已知上抬枪口后，静态 raw_a_x 从约 +0.33 降到 -3.90 m/s²；
 * INS 传感器系为 x后/y右/z上，与物理 G 系 x前/y右/z下相差 diag(-1,1,-1)。
 * 完整杆臂须与 imu_q、imu_gyro_rad 同属 INS 机体系；安装轴修正变化时也须核对。 */
#ifndef MC_IMU_LEVER_SENSOR_X_M
#define MC_IMU_LEVER_SENSOR_X_M (-MC_IMU_OFFSET_X_M)
#endif
#ifndef MC_IMU_LEVER_SENSOR_Y_M
#define MC_IMU_LEVER_SENSOR_Y_M MC_IMU_OFFSET_Y_M
#endif
#ifndef MC_IMU_LEVER_SENSOR_Z_M
#define MC_IMU_LEVER_SENSOR_Z_M 0.037f
#endif
#ifndef MC_YAW_CENTER_X_M
#define MC_YAW_CENTER_X_M (CENTER_GIMBAL_OFFSET_X * 0.001f)
#endif
#ifndef MC_YAW_CENTER_Y_M
#define MC_YAW_CENTER_Y_M (CENTER_GIMBAL_OFFSET_Y * 0.001f)
#endif
#define MC_ALPHA_LIMIT_RPS2 100.0f

/* R 为方差，Q 使用连续白 jerk 谱密度；数值是离线验证初值。 */
#define MC_R_VX 0.0025f
#define MC_R_AX 0.04f
#define MC_R_VY 0.0025f
#define MC_R_AY 0.04f
/* 云台角速度越大，杆臂/重力残差越不可信。按三轴实测角速度增大方差，
 * 仍保留加速度观测；不以“摇杆为零/轮速为零”强制加速度为零。单位 SI。 */
#define MC_ACCEL_ROTATION_R_GAIN 0.04f
#define MC_Q_X 0.1f
#define MC_Q_Y 0.1f
#define MC_P_V 0.04f
#define MC_P_A 0.25f
#define MC_MAX_P_V 0.25f
#define MC_SAMPLE_MAX_US 20000u
#define MC_SAMPLE_SKEW_US 10000u
#define MC_INERTIAL_MAX_US 100000u
#define MC_RECOVER_SAMPLES 20u
#define MC_ACCEL_CONSISTENCY_WINDOW_S 0.25f
#define MC_ACCEL_CONSISTENCY_MAX_DV 0.04f /* m/s: integral(a)-delta(wheel_v), not a slip verdict */
#define MC_MAX_VELOCITY_INNOVATION_MPS 0.04f /* 超出时撤销融合权限，不判定哪一路是真值。 */
/* 暂定控制保护值，不是第2步传感器验收线；落地观测未通过，继续SHADOW。
 * residual 为 integral(a)-delta(wheel_v) / 实际窗口秒数，允许匹配的真实加速。 */
#define MC_CONTROL_STABLE_S 1.0f
#define MC_CONTROL_VELOCITY_MAX_MPS 0.01f
#define MC_CONTROL_ACCEL_RESIDUAL_MAX_MPS2 0.10f
#define MC_CONTROL_RAMP_S 0.5f
#define MC_DT_MIN_S 0.0005f
#define MC_DT_MAX_S 0.02f
#define MC_MAX_ACCEL_MPS2 30.0f

#define MC_FIXED_KP 0.10f
#define MC_SCHEDULE_KPR 0.10f
#define MC_SCHEDULE_OMEGA_RPS 2.0f
#define MC_KP_MAX 0.30f
#define MC_DV_MAX_MPS 0.10f
#define MC_DV_SLEW_MPS2 0.5f
/* 兼容受限平移配置，仅 direct_translation=0 时使用以下速度上限和斜坡。 */
#define MC_V_MAX_MPS 1.0f
#define MC_V_SLEW_MPS2 3.0f
/* SI角速度上限；轮径修正不同时提高该限制。 */
#define MC_W_MAX_RPS 9.0f
#define MC_W_SLEW_RPS2 10.0f
/* 新纯自旋四轮约34034°/s；同步将原24000上限提高1.5倍，仍统一缩放超限请求。 */
#define MC_WHEEL_MAX_DPS 36000.0f
#define MC_CURRENT_LIMIT_RAW 8000.0f
/* 只限制电流，不伪造已接入裁判功率环。 */
#define MC_LOG_DECIMATION 20u // 文本每 20 个控制周期输出一组，约 10 Hz；0 禁用。

const ChassisConfig g_chassis_config = {
    .wheel_base_mm = WHEEL_BASE,
    .track_width_mm = TRACK_WIDTH,
    .wheel_radius_mm = RADIUS_WHEEL,
    .reduction_ratio = REDUCTION_RATIO_WHEEL,
    .center_offset_x_mm = CENTER_GIMBAL_OFFSET_X,
    .center_offset_y_mm = CENTER_GIMBAL_OFFSET_Y,
    .mps_per_motor_dps = MC_MPS_PER_DPS,
    .legacy_omega_scale = MC_LEGACY_W_SCALE,
    .legacy_spin_command = MC_SPIN_LEGACY_WZ,
    .legacy_omni_center = (HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X +
                           HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD,
    .command_trim_deg = 0.0f,            /* 底盘平移指令零偏，°。 */
    .follow_gain = -50.0f,               /* 云台相对角度到旧底盘旋转指令。 */
    .follow_limit = 4500.0f,             /* 跟随模式旋转指令幅值上限，旧协议单位。 */
    .velocity_command_limit = 20000.0f,  /* 旧协议平移输入的合法幅值。 */
    .offset_angle_limit_deg = 360.0f,    /* 云台相对角度合法范围，°。 */
    .command_timeout_us = 100000u,       /* 停止使用无更新底盘命令的绝对时间，µs。 */
    .command_timeout_cycles = 20u,       /* 无新底盘指令的最大底盘任务周期数。 */
    .motor_tx_id = {1, 4, 2, 3},         /* CAN 发送 ID，LF/RF/LB/RB。 */
    .motor_reversed = {0, 0, 0, 0},      /* LF/RF/LB/RB，1 反转电机控制方向。 */
    .speed_pid = {
        .Kp = 4.5, .Ki = 0, .Kd = 0,
        .IntegralLimit = 3000, .MaxOut = 15000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .Output_LPF_RC = 0.3,            /* 电机速度环输出滤波时间常数，s。 */
    },
    .current_pid = {
        /* 电流反馈和参考均为 DJI CAN 原始量。 */
        .Kp = 0.5f, .Ki = 0.05f, .Kd = 0,
        .IntegralLimit = 5000, .MaxOut = CHASSIS_FORCE_CURRENT_LIMIT_RAW,
        .Improve = PID_Integral_Limit | PID_OutputFilter,
        .Output_LPF_RC = 0.002f,
    },
    .power_buffer_pid = {
        /* 缓冲能量误差到冗余功率，功控链预留。 */
        .Kp = 0.1, .Ki = 0, .Kd = 0,
        .IntegralLimit = 1000, .MaxOut = 1000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    },
};

const ChassisImuConfig g_chassis_imu_config = {
    .on_gimbal = MC_IMU_ON_GIMBAL,
    .follows_pitch = MC_IMU_ON_GIMBAL && MC_IMU_FOLLOWS_PITCH,
    .gimbal_lever_m = {MC_IMU_LEVER_SENSOR_X_M, MC_IMU_LEVER_SENSOR_Y_M, MC_IMU_LEVER_SENSOR_Z_M},
    .chassis_lever_m = {MC_CHASSIS_IMU_LEVER_X_M, MC_CHASSIS_IMU_LEVER_Y_M, MC_CHASSIS_IMU_LEVER_Z_M},
    .heading_offset_rad = MC_IMU_HEADING_OFFSET_RAD,
    .yaw_encoder_sign = MC_YAW_ENCODER_SIGN,
    .still_command_max = 0.001f,       /* 三轴旧指令均低于该幅值才允许静止辅助标定。 */
    .still_raw_dps = 0.5f,             /* 静止辅助的原始转子速度上限，°/s。 */
    .still_filtered_dps = 1.0f,        /* 静止辅助的滤波转子速度上限，°/s。 */
    .min_horizontal_heading = 0.2f,   /* 姿态前轴水平投影长度下限。 */
};

const ChassisLogConfig g_chassis_log_config = {
    .frame_revision = MC_FRAME_REVISION,
    .capture_duration_us = 350000u,     /* WIMU 布防后的原始采集长度，µs。 */
    .text_rtt_channel = 0u,             /* 底盘诊断文本 RTT 通道，使用 SEGGER 启动时分配的通道 0。 */
    .capture_rtt_channel = 1u,          /* WIMU 二进制 RTT 通道。 */
    .force_log_period_cycles = 20u,     /* 力控诊断抽样周期，约 100 ms。 */
};

const ChassisMotionConfig g_chassis_motion_config = {
    .wheels = {
#if defined(CHASSIS_DRIVE_MECANUM)
        .type = CK_MECANUM_X,
#elif defined(CHASSIS_DRIVE_SWERVE)
        .type = CK_SWERVE,
#else
        .type = CK_CUSTOM,
        .rows = MC_J_ROWS,
#endif
        .radius_m = MC_WHEEL_RADII,
        .motor_sign = MC_ENCODER_SIGNS,
        .gear_ratio = {MC_GEAR_RATIO, MC_GEAR_RATIO, MC_GEAR_RATIO, MC_GEAR_RATIO},
        .max_motor_dps = {MC_WHEEL_MAX_DPS, MC_WHEEL_MAX_DPS, MC_WHEEL_MAX_DPS, MC_WHEEL_MAX_DPS},
    },
#if defined(CHASSIS_DRIVE_OMNI_X)
    .algebra_y_sign = -1, /* 保留旧全向轮代数的横向反射。 */
#else
    .algebra_y_sign = 1,
#endif
#if defined(ONE_BOARD) && defined(CHASSIS_DRIVE_OMNI_X)
    .spin_wheel_trim_dps = MC_SPIN_WHEEL_TRIM_DPS,
#endif
    .version = MC_CONFIG_VERSION,
    .initial_mode = MC_DEFAULT_MODE,
    .imu_on_gimbal = MC_IMU_ON_GIMBAL,
    .imu_follows_pitch = MC_IMU_ON_GIMBAL && MC_IMU_FOLLOWS_PITCH,
#if defined(CHASSIS_BOARD) || (!MC_IMU_ON_GIMBAL && !defined(MC_HOST_TEST))
    .use_center_accel = 1, /* 本地底盘 INS 补偿其三维杆臂。 */
#else
    .use_center_accel = MC_IMU_ON_GIMBAL && MC_IMU_FOLLOWS_PITCH,
#endif
    .center_y_sign = -1, /* INS N=(x,-y,-z) */
    .accel_enabled = MC_IMU_ACCEL_VALIDATED,
    .imu_offset_m = {MC_IMU_OFFSET_X_M, MC_IMU_OFFSET_Y_M},
    .yaw_center_m = {MC_YAW_CENTER_X_M, MC_YAW_CENTER_Y_M},
    .accel_bias_n = {MC_ACCEL_BIAS_N_X, MC_ACCEL_BIAS_N_Y},
    .r_velocity = {MC_R_VX, MC_R_VY},
    .r_accel = {MC_R_AX, MC_R_AY},
    .q_jerk = {MC_Q_X, MC_Q_Y},
    .p_velocity = MC_P_V,
    .p_accel = MC_P_A,
    .max_p_velocity = MC_MAX_P_V,
    .rotation_r_gain = MC_ACCEL_ROTATION_R_GAIN,
    .max_accel = MC_MAX_ACCEL_MPS2,
    .max_alpha = MC_ALPHA_LIMIT_RPS2,
    .fixed_kp = MC_FIXED_KP,
    .schedule_kp = MC_SCHEDULE_KPR,
    .schedule_omega = MC_SCHEDULE_OMEGA_RPS,
    .max_kp = MC_KP_MAX,
    .max_dv = MC_DV_MAX_MPS,
    .dv_slew = MC_DV_SLEW_MPS2,
    .max_velocity = MC_V_MAX_MPS,
    .velocity_slew = MC_V_SLEW_MPS2,
    .direct_translation = 1u, /* 平移指令本周期直接进入四轮解算，仍约束轮速与电流。 */
    .max_omega = MC_W_MAX_RPS,
    .omega_slew = MC_W_SLEW_RPS2,
    .current_limit = MC_CURRENT_LIMIT_RAW,
    .phase_lead_s = MC_SPIN_PHASE_LEAD_S,
    .max_phase_lead = MC_SPIN_PHASE_MAX_RAD,
    .theta_rate_lpf = MC_THETA_RATE_LPF_S,
    .max_theta_rate = MC_THETA_RATE_MAX_RPS,
    .dt_min = MC_DT_MIN_S,
    .dt_max = MC_DT_MAX_S,
    .accel_consistency_window_s = MC_ACCEL_CONSISTENCY_WINDOW_S,
    .accel_consistency_max_dv = MC_ACCEL_CONSISTENCY_MAX_DV,
    .velocity_innovation_max = MC_MAX_VELOCITY_INNOVATION_MPS,
    .control_stable_s = MC_CONTROL_STABLE_S,
    .control_velocity_max = MC_CONTROL_VELOCITY_MAX_MPS,
    .control_accel_residual_max = MC_CONTROL_ACCEL_RESIDUAL_MAX_MPS2,
    .control_ramp_s = MC_CONTROL_RAMP_S,
    .sample_max_us = MC_SAMPLE_MAX_US,
    .sample_skew_us = MC_SAMPLE_SKEW_US,
    .inertial_max_us = MC_INERTIAL_MAX_US,
    .recover_samples = MC_RECOVER_SAMPLES,
    .log_decimation = MC_LOG_DECIMATION,
    .rotation_priority = MC_WHEEL_LIMIT_ROTATION_PRIORITY,
};

/* 力估计及分配共用的几何。LF/RF/LB/RB，x 向前，y 向右。 */
#define FORCE_COS45 0.70710678f
#define FORCE_OMNI_MOMENT_ARM_M ((HALF_WHEEL_BASE + HALF_TRACK_WIDTH) * 0.001f * FORCE_COS45)

const ChassisForceConfig g_chassis_force_config = {
    .wheel_radius_m = (RADIUS_WHEEL * 0.001f), /* 力与转矩换算的车轮半径，m。 */
    .gear_ratio = REDUCTION_RATIO_WHEEL, /* 转子转速与轮端转速比。 */
    .drive_efficiency = 0.85f, /* 传动效率。 */
    .motor_kt_nm_per_a = 0.30f, /* 转矩常数，N·m/A。 */
    .current_raw_per_a = (16384.0f / 20.0f), /* 每安培对应的 DJI CAN 原始电流量。 */
    .current_limit_raw = CHASSIS_FORCE_CURRENT_LIMIT_RAW, /* 四轮峰值电流，DJI CAN raw。 */
    .current_slew_raw = 600.0f, /* 单周期电流变化上限，DJI CAN raw。 */
    .speed_damping_raw_per_dps = 0.020f, /* 行驶阻尼，电流 raw / 转子 °/s。 */
    .speed_damping_limit_raw = 1600.0f, /* 行驶阻尼电流上限，DJI CAN raw。 */
    .release_brake_raw_per_dps = 0.60f, /* 松杆制动，电流 raw / 转子 °/s。 */
    .release_brake_limit_raw = 2400.0f, /* 松杆制动电流上限，DJI CAN raw。 */
    .speed_limit_dps = 24000.0f, /* 进入超速制动的转子速度，°/s。 */
    .overspeed_brake_raw = 1200.0f, /* 超速反向制动电流，DJI CAN raw。 */
    .start_translation_n = 50.0f, /* 沿指令平移方向的起步附加力，N。 */
    .start_moment_nm = 1.0f, /* 沿指令旋转方向的起步附加力矩，N·m。 */
    .start_speed_dps = 300.0f, /* 允许起步补偿的转子速度上限，°/s。 */
    .start_exit_speed_dps = 1500.0f, /* 退出起步补偿的转子速度，°/s。 */
    .start_exit_confirm_cycles = 10u, /* 退出起步补偿所需的连续底盘周期。 */
    .start_hold_cycles = 80u, /* 起步补偿保持底盘周期数。 */
    .start_retry_cooldown_cycles = 80u, /* 堵转后重试间隔，底盘周期。 */
    .start_wheel_comp_raw = 1500.0f, /* 起步单轮附加电流，DJI CAN raw。 */
    .start_wheel_min_raw = 80.0f, /* 允许起步补偿的最小单轮电流，DJI CAN raw。 */
    .start_input_deadband_n = 3.0f, /* 起步平移输入死区，N。 */
    .start_input_deadband_nm = 0.08f, /* 起步旋转输入死区，N·m。 */
    .pseudoinverse_damping = 1.0e-4f, /* 轮力分配雅可比最小二乘正则项。 */
    .current_filter_alpha = 0.20f, /* 力估计电流低通系数，200 Hz 下约 7 Hz。 */
    .gyro_damping_nm_per_dps = 0.003f, /* 陀螺阻尼力矩增益，N·m/(°/s)。 */
    .gyro_damping_limit_nm = 1.5f, /* 陀螺阻尼力矩上限，N·m。 */
    .input_max_n = 500.0f, /* 单轴合法平移力幅值上限，N。 */
    .input_max_nm = 100.0f, /* 合法旋转力矩幅值上限，N·m。 */
    .admittance_mass_x = 8.0f, /* 纵向虚拟质量，kg。 */
    .admittance_mass_y = 8.0f, /* 横向虚拟质量，kg。 */
    .admittance_mass_z = 1.5f, /* 偏航虚拟转动惯量，kg·m²。 */
    .admittance_damping_x = 35.0f, /* 纵向导纳阻尼，作用在旧协议速度。 */
    .admittance_damping_y = 35.0f, /* 横向导纳阻尼，作用在旧协议速度。 */
    .admittance_damping_z = 8.0f, /* 偏航导纳阻尼，作用在旧协议速度。 */
    .admittance_max_vx = 1500.0f, /* 纵向导纳速度上限，旧协议单位。 */
    .admittance_max_vy = 1500.0f, /* 横向导纳速度上限，旧协议单位。 */
    .admittance_max_wz = 1200.0f, /* 偏航导纳速度上限，旧协议单位。 */
    .sensor_timeout_cycles = 20u, /* 有效传感器样本最大保留底盘周期数。 */
    .current_offset_raw = {1.0f, 3.7f, 3.0f, -1.6f}, /* LF/RF/LB/RB，DJI CAN raw。 */
    .rows = {
        {-FORCE_COS45,  FORCE_COS45, FORCE_OMNI_MOMENT_ARM_M},
        { FORCE_COS45,  FORCE_COS45, FORCE_OMNI_MOMENT_ARM_M},
        {-FORCE_COS45, -FORCE_COS45, FORCE_OMNI_MOMENT_ARM_M},
        { FORCE_COS45, -FORCE_COS45, FORCE_OMNI_MOMENT_ARM_M},
    },
    .admittance_dt_min_s = 0.001f,
    .admittance_dt_max_s = 0.02f,
    .admittance_dt_fallback_s = 0.005f,
    .force_xy_pid = {
        /* 力跟踪 PID，平移轴限幅 N。 */
        .Kp = 0.40f, .Ki = 0.02f, .Kd = 0.0f,
        .IntegralLimit = 60.0f, .MaxOut = 60.0f,
        .Improve = PID_Integral_Limit | PID_OutputFilter,
        .Output_LPF_RC = 0.01f,
    },
    .force_yaw_pid = {
        /* 偏航力矩跟踪 PID，限幅 N·m。 */
        .Kp = 0.40f, .Ki = 0.02f, .Kd = 0.0f,
        .IntegralLimit = 8.0f, .MaxOut = 8.0f,
        .Improve = PID_Integral_Limit | PID_OutputFilter,
        .Output_LPF_RC = 0.01f,
    },
};
