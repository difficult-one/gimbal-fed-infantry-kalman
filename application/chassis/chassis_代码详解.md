# 底盘代码结构详解（chassis）

> 以下保留框架整理前的代码片段、标定和行号。当前入口与接线见 [chassis.c](chassis.c)，运动配置装配见 [chassis_setup.c](chassis_setup.c)，力控和日志分别见 [chassis_force.c](chassis_force.c)、[chassis_log.c](chassis_log.c)。板型选择见 [build_config.h](../../config/build_config.h)，车型标定见 [config.c](../../config/config.c)，共享消息见 [robot_msg.h](../robot_msg.h)。

> 本文档面向**需要理解并日后重写底盘**的读者，逐段梳理 `application/chassis/` 下的代码。
> 阅读顺序建议：先看「整体结构」和「数据流」，再看「逐段讲解」，最后看「重写清单」。

---

## 目录

1. [文件组成](#1-文件组成)
2. [整体结构一句话总结](#2-整体结构一句话总结)
3. [坐标系与物理量约定](#3-坐标系与物理量约定)
4. [编译开关与 robot_def.h 对底盘的影响](#4-编译开关与-robot_defh-对底盘的影响)
5. [关键数据结构](#5-关键数据结构)
6. [chassis.h 讲解](#6-chassish-讲解)
7. [chassis.c 逐段讲解](#7-chassisc-逐段讲解)
8. [完整数据流](#8-完整数据流)
9. [与其他模块的依赖关系](#9-与其他模块的依赖关系)
10. [重写底盘前的检查清单](#10-重写底盘前的检查清单)

---

## 1. 文件组成

底盘相关文件都在 `application/chassis/` 目录下：

| 文件 | 作用 |
| --- | --- |
| `chassis.c` | 底盘应用的全部实现（电机初始化、运动学解算、限幅、主任务循环） |
| `chassis.h` | 对外暴露的两个函数声明：`ChassisInit()` / `ChassisTask()` |
| `chassis.md` | 作者写的简短说明（工作流程概述，内容较简略） |

底盘是**单例模式**，不需要为底盘建结构体，所有状态用 `static` 变量保存在 `chassis.c` 内部（见下）。

---

## 2. 整体结构一句话总结

> **底盘是一个「订阅控制命令 → 运动学解算 → 限幅 → 输出电机参考值 → 回传反馈」的任务。**
>
> 它不产生控制意图（不读遥控器），控制意图由 `robot_cmd`（cmd 应用）产生并通过「消息中心（单板）」或「CAN 通信（双板）」发来；底盘只负责把「期望的 vx / vy / wz（前进、横移、自旋）」转换成「四个轮子的转速设定值」，交给电机模块的 PID 去执行，最后把真实速度/热量等反馈回传。

---

## 3. 坐标系与物理量约定

来自 `chassis.c` 头注释与 `robot_def.h`：

- **右手系**：平面视图下，底盘前进方向为 **+x**，横向向右为 **+y**。
- **旋转正方向**：逆时针（CCW）为角度正方向。
- 云台命令的坐标系以**云台指向方向为 x 轴**（x 指正北时 y 指正东），所以需要把云台系速度「旋转」到底盘系。
- 速度单位：命令 `vx/vy` 来自遥控器映射（见 `robot_cmd.c` 的 `chassis_cmd_send.vx = 10.0f * rocker`，量级约 ±6600），`wz` 单位是 **度/秒（°/s）**（自旋模式给 `wz = CHASSIS_ROTATE_WZ`，默认 4000）。
- 轮子反馈 `measure.speed_aps` 单位是 **度/秒（°/s）**，见 `dji_motor.h:37`。

> ⚠️ **单位说明（重要，重写时会踩坑）**：这套代码的运动学输出 `vt_*` 与电机的速度环参考值之间存在**松散的标定关系**——正运动学里 `vx/vy` 是「线速度 mm/s」量级，`wz * 力臂` 通过 `* DEGREE_2_RAD` 折成同样的线速度量级，二者相加后直接当作电机速度环参考（°/s）。中间并没有显式的 `线速度 → 轮角速度` 换算（本应除以轮半径再乘 `RAD_2_DEG`）。**实际效果靠 PID 参数（Kp=2.6 等）和斜坡限幅的标定去兜底**，所以 PID 参数是经验调出来的，不纯是物理推导。重写时建议补上单位换算，让参考值就是明确的 `°/s`。

---

## 4. 编译开关与 robot_def.h 对底盘的影响

`robot_def.h` 通过 `#define` 决定底盘走哪条代码路径：

```c
// 板类型（三选一，只能定义一个）
#define ONE_BOARD      // 单板控制整车（当前启用）
// #define CHASSIS_BOARD // 底盘板（双板，板载 IMU + CAN 通信）
// #define GIMBAL_BOARD  // 云台板

// 运动学类型（二选一）
// #define CHASSIS_DRIVE_MECANUM // 麦克纳姆轮
#define CHASSIS_DRIVE_OMNI_X    // X 型四全向轮（当前启用）
#define CHASSIS_CMD_DIR_TRIM_DEG -45.0f // 平动命令方向零偏修正（deg）；实车标定值：X 型全向轮机械安装/IMU 零位与底盘前进方向差 45°，保持 -45° 才走直线
```

对应的分支逻辑：

| 开关 | 影响 |
| --- | --- |
| `ONE_BOARD` | 底盘用**消息中心（PubSub）**收发：`chassis_sub` / `chassis_pub` |
| `CHASSIS_BOARD` | 底盘用 **CANComm** 收发；额外初始化 IMU（`INS_Init`）获取底盘转动角速度 |
| `CHASSIS_DRIVE_MECANUM` | 编译 `MecanumCalculate()` |
| `CHASSIS_DRIVE_OMNI_X` | 编译 `OmniCalculate()` |
| `CHASSIS_CMD_DIR_TRIM_DEG` | 坐标旋转变换时叠加的角度偏置 |

底盘相关的物理参数（`robot_def.h:44-49`）：

```c
#define WHEEL_BASE 350              // 纵向轴距 mm（前进后退方向）
#define TRACK_WIDTH 300             // 横向轮距 mm（左右平移方向）
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心相对底盘几何中心的 X 偏置
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心相对底盘几何中心的 Y 偏置
#define RADIUS_WHEEL 72.5f          // 轮子半径 mm，四轮外径145 mm
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比（编码器测转子，需折算到输出轴）
```

---

## 5. 关键数据结构

定义在 `robot_def.h`，底盘只直接使用前两个。

### 5.1 控制命令 `Chassis_Ctrl_Cmd_s`（cmd → 底盘）

```c
typedef struct {
    float vx;            // 前进方向速度命令（底盘系）
    float vy;            // 横移方向速度命令
    float wz;            // 旋转速度命令（°/s）
    float offset_angle;  // 云台与底盘归中位置的夹角（°），用于坐标变换/跟随
    chassis_mode_e chassis_mode; // 底盘工作模式（见下）
    int chassis_speed_buff;      // 速度缓冲（键鼠模式下可调速度档位）
} Chassis_Ctrl_Cmd_s;
```

`offset_angle` 是由 `robot_cmd.c` 的 `CalcOffsetAngle()` 根据云台 yaw 电机编码器算出并**归一化到 `[-180, 180]`** 的（`robot_cmd.c:96`）。

### 5.2 反馈数据 `Chassis_Upload_Data_s`（底盘 → cmd）

```c
typedef struct {
    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 敌方颜色 0:blue 1:red
} Chassis_Upload_Data_s;
```

> 注意：当前 `ChassisTask()` 里这个结构体**没有被实际赋值**，相关字段代码被注释掉了（`chassis.c:376-381`）。所以现在它只是以一个空壳结构体被推送到消息中心。

### 5.3 底盘模式 `chassis_mode_e`（`robot_def.h:89`）

```c
typedef enum {
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入（急停/离线）
    CHASSIS_ROTATE,            // 小陀螺模式（自旋）
    CHASSIS_NO_FOLLOW,         // 不跟随云台，但允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环去追云台朝向
} chassis_mode_e;
```

---

## 6. chassis.h 讲解

`chassis.h` 只有两个声明（`chassis.h:8-14`）：

```c
void ChassisInit();  // 底盘初始化，请在开启 RTOS 之前调用（被 RobotInit() 调用）
void ChassisTask();  // 底盘任务，放入 RTOS 以一定频率运行
```

它被 `robot.c` 调用：

```c
void RobotInit() {              // robot.c:10
    __disable_irq();
    BSPInit();
    RobotCMDInit();
    //GimbalInit(); ShootInit();
    ChassisInit();              // ← 初始化底盘
    __enable_irq();
}
void RobotTask() {              // robot.c:27，被 RTOS 调度
    RobotCMDTask();
    //GimbalTask(); ShootTask();
    ChassisTask();              // ← 周期运行底盘任务
}
```

---

## 7. chassis.c 逐段讲解

### 7.1 文件头注释与依赖（`chassis.c:1-27`）

```c
/**
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系...正前方为x正方向;右侧为y正方向
 */
#include "chassis.h"
#include "robot_def.h"
#include "power_control.h"   // 功率限制 SetPowerLimit()
#include "super_cap.h"       // 超级电容
#include "message_center.h"  // 单板 PubSub
#include "referee_task.h"    // 裁判系统
#include <math.h>            // fabsf()
#include "general_def.h"     // PI / DEGREE_2_RAD
#include "bsp_dwt.h"         // DWT_GetTimeline_us()
#include "referee_UI.h"      // 裁判系统 UI
#include "arm_math.h"        // arm_cos_f32 / arm_sin_f32（ARM DSP 加速三角函数）
#include "SEGGER_RTT.h"      // JScope / RTT 调试输出
```

### 7.2 由 robot_def.h 自动计算的派生宏（`chassis.c:29-32`）

```c
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距 = 175
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距 = 150
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长（当前未使用，供速度→转速换算预留）
```

### 7.3 模块实例与状态变量（`chassis.c:36-62`）

这是底盘的全部「私有状态」，全部 `static`（单例）：

```c
#ifdef CHASSIS_BOARD
static CANCommInstance *chasiss_can_comm; // 双板通信实例
attitude_t *Chassis_IMU_data;             // 底盘板 IMU 姿态数据
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;    // 发布底盘反馈
static Subscriber_t *chassis_sub;   // 订阅底盘控制命令
#endif
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 回传的反馈
static PIDInstance buffer_PID;             // 缓冲能量 PID（TODO：尚未接入功率控制）
static referee_info_t *referee_data;       // 裁判系统数据指针
static Referee_Interactive_info_t ui_data; // UI 数据结构（由 UITaskInit 初始化）
static SuperCapInstance *cap;                                       // 超级电容实例
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // 左前/右前/左后/右后电机

static float chassis_vx, chassis_vy;  // 云台系速度投影到底盘系后的结果
static float vt_lf, vt_rf, vt_lb, vt_rb; // 解算后待限幅的四个轮速输出
```

### 7.4 JScope / RTT 调试辅助（`chassis.c:64-133`）

这一整段用于**实时观测某个轮子的「参考值 vs 反馈值」**，帮助调 PID：

```c
#define CHASSIS_JSCOPE_CH 1
#define CHASSIS_JSCOPE_WHEEL_LF 0
...
#define CHASSIS_JSCOPE_WHEEL CHASSIS_JSCOPE_WHEEL_LF   // 选择观测哪个轮子
#define CHASSIS_RTT_CSV_LOG_ENABLE 0                   // 是否额外打印 CSV
#define CHASSIS_REF_SLEW_RATE 15000.0f                 // 参考值变化率上限（见 7.6）
#define CHASSIS_TASK_DT 0.005f                         // 任务周期 5ms = 200Hz
```

- `ChassisJScopePacket_t`：`{ uint32 time_us; float ref; float fb; }`，`#pragma pack(1)` 紧凑排列，直接二进制写入 RTT 通道。
- `ChassisJScopeInit()`：`SEGGER_RTT_ConfigUpBuffer()` 注册通道 1，名字 `"JScope_t4f4f4"`（`t4`=4 字节时间戳 + 两个 `f4` 浮点通道），并在通道 0 打印表头 `time_us,ref_x100,fb_x100`。
- `ChassisJScopeSend()`：根据 `CHASSIS_JSCOPE_WHEEL` 宏选一个轮子，把它的 `vt_*`（参考）和 `motor_*->measure.speed_aps`（反馈）打包写入通道；若开启 CSV 日志则按 `DIV` 分频打印整数化（×100）的值。

> 这段是**调试代码**，重写时可以保留也可以去掉，不影响底盘逻辑。

### 7.5 斜坡限幅函数 `ChassisRampLimit`（`chassis.c:135-145`）

```c
static float ChassisRampLimit(float target, float last) {
    float delta = target - last;
    float max_delta = CHASSIS_REF_SLEW_RATE * CHASSIS_TASK_DT; // 15000 * 0.005 = 75
    if (delta > max_delta)  return last + max_delta;
    if (delta < -max_delta) return last - max_delta;
    return target;
}
```

作用：**限制参考值每拍的允许变化量（slew-rate limiter / 斜率限幅）**。防止目标速度突变导致电机电流尖峰或失控。每次调用最多让参考值变化 ±75（对应每秒 15000 的变化率）。

### 7.6 初始化 `ChassisInit`（`chassis.c:147-231`）

初始化顺序如下：

**① 电机初始化（`chassis.c:150-187`）**

用一个配置结构体 `Motor_Init_Config_s chassis_motor_config` 初始化四个 M3508 电机：

```c
Motor_Init_Config_s chassis_motor_config = {
    .can_init_config.can_handle = &hcan2,       // 挂载在 CAN2 总线
    .controller_param_init_config = {
        .speed_PID = {
            .Kp = 2.6, .Ki = 0.10, .Kd = 0,
            .IntegralLimit = 800,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit
                     | PID_Derivative_On_Measurement | PID_OutputFilter,
            .MaxOut = 12000,
            .Output_LPF_RC = 0.006,
        },
    },
    .controller_setting_init_config = {
        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,
        .outer_loop_type = SPEED_LOOP,  // 只跑速度环（电流环恒在最内层）
        .close_loop_type = SPEED_LOOP,
    },
    .motor_type = M3508,
};
```

- **PID 只配了速度环**：`outer_loop_type = SPEED_LOOP`、`close_loop_type = SPEED_LOOP`，意味着底盘电机闭环只有「速度环 + 电流环」，没有角度环。
- 四个电机只是 `tx_id` 不同：**LF=1, RF=4, LB=2, RB=3**。M3508 反馈报文是 `0x200 + id`。
- `motor_reverse_flag` 目前都设 `MOTOR_DIRECTION_NORMAL`，注释里写明正反转目前还没做进电机模块，需要靠解算时的正负号手动处理（`chassis.c:171`）。

**② 裁判系统初始化（`chassis.c:189`）**

```c
referee_data = UITaskInit(&huart6, &ui_data);
```

`UITaskInit` 初始化裁判系统（挂在串口 6），同时初始化 UI；返回 `referee_info_t*` 供后续读裁判数据。

**③ 缓冲能量 PID + 超级电容（`chassis.c:191-207`）**

```c
PID_Init_Config_s Buffer_pid_conf = { .Kp=0.1, .Ki=0, .Kd=0, ... };
PIDInit(&buffer_PID, &Buffer_pid_conf);   // 缓冲能量 PID（TODO 暂未接入）

SuperCap_Init_Config_s cap_conf = {
    .can_config = {
        .can_handle = &hcan2,
        .tx_id = 0x302, // 超级电容默认接收 id
        .rx_id = 0x301, // 超级电容默认发送 id（注意 tx/rx 在对方看来是反的）
    }};
cap = SuperCapInit(&cap_conf);
```

> 缓冲能量 PID 目前只是初始化，还没有参与控制（注释 `chassis.c:191` 说明这是「计算期望 buffer 与实际 buffer 差值 → 转冗余功率」的 TODO）。

**④ 通信初始化（单板 vs 双板）（`chassis.c:209-228`）**

```c
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init();   // 底盘 IMU 初始化
    CANComm_Init_Config_s comm_conf = {
        .can_config = { .can_handle = &hcan2, .tx_id = 0x311, .rx_id = 0x312 },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf);
#endif

#ifdef ONE_BOARD
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif
```

- 双板：底盘板通过 CANComm 与云台板通信（`0x311` 发反馈 / `0x312` 收命令，与 cmd 侧正好相反，见 `robot_cmd.c:69-79`）。
- 单板：通过消息中心的**话题名**配对——`"chassis_cmd"` 和 `"chassis_feed"`，与 `robot_cmd.c` 里注册的名字一致。

**⑤ JScope 初始化（`chassis.c:230`）**

### 7.7 运动学力臂宏（`chassis.c:233-236`）

```c
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
```

- 这是**每个轮子到云台旋转中心的「有效力臂」**：`半轮距 ± X偏置 + 半轴距 ± Y偏置`。
- 末尾 `* DEGREE_2_RAD`：把「角度/秒」的 `wz` 折成「线速度」量级（因为 `wz` 单位是 °/s，乘上 `π/180` 后 `wz·L` 就是 mm/s 量级，与 `vx/vy` 对齐）。
- 当前 `CENTER_GIMBAL_OFFSET_X/Y` 都是 0，四个力臂值相等，因此 X 型全向轮用 `OMNI_L = LF_CENTER` 统一代用即可。

### 7.8 正运动学解算（`chassis.c:238-270`）

**麦克纳姆轮 `MecanumCalculate()`（`chassis.c:243-249`，默认被注释掉）**

```c
vt_lf = -chassis_vx - chassis_vy - wz * LF_CENTER;
vt_rf = -chassis_vx + chassis_vy - wz * RF_CENTER;
vt_lb =  chassis_vx - chassis_vy - wz * LB_CENTER;
vt_rb =  chassis_vx + chassis_vy - wz * RB_CENTER;
```

标准的麦轮逆运动学公式：把底盘的 `(vx, vy, wz)` 分解到四个轮子上，正负号体现各轮的安装方向。

**X 型四全向轮 `OmniCalculate()`（`chassis.c:263-269`，当前启用）**

```c
#define COS45 0.70710678f
#define OMNI_L (LF_CENTER)

vt_rf =  chassis_vx * COS45 + chassis_vy * COS45 + wz * OMNI_L;
vt_lf = -chassis_vx * COS45 + chassis_vy * COS45 + wz * OMNI_L;
vt_lb = -chassis_vx * COS45 - chassis_vy * COS45 + wz * OMNI_L;
vt_rb =  chassis_vx * COS45 - chassis_vy * COS45 + wz * OMNI_L;
```

- 轮子布局：`RF=45°、LF=135°、LB=225°、RB=315°`（互成 90°）。
- 每个轮子的速度分量 = `vx·cos(θ) + vy·sin(θ) ± wz·L`，即把底盘速度投影到该轮子的驱动方向。
- 系数 `COS45` 来自 45° 布局的投影（`cos45 = sin45 ≈ 0.7071`）。

### 7.9 输出限幅 + 设定参考值 `LimitChassisOutput()`（`chassis.c:276-300`）

```c
static void LimitChassisOutput() {
    static float last_lf, last_rf, last_lb, last_rb; // 记住上一拍，供斜坡限幅

    // 功率限制待添加（占位注释）：
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    vt_lf = ChassisRampLimit(vt_lf, last_lf); // 对四个轮速逐一斜坡限幅
    ...（同 rf/lb/rb）
    last_lf = vt_lf; ...                     // 更新上一拍

    DJIMotorSetRef(motor_lf, vt_lf);         // 把限幅后的参考值写入电机速度环
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);

    ChassisJScopeSend();                     // 输出调试数据
}
```

功能：
1. **斜坡限幅**（用 `ChassisRampLimit` 限制每拍变化量，平滑参考值，防止突变）。
2. **写参考值**到四个电机（`DJIMotorSetRef` 设的是速度环参考，电机模块内部会跑速度/电流串级 PID）。
3. 触发 JScope 发送。

> 真正的「功率限制」目前是**待实现**的占位；当前功率限制是在 `ChassisTask()` 里通过 `SetPowerLimit()` 单独设置的（见 7.11）。

### 7.10 逆运动学估计 `EstimateSpeed()`（`chassis.c:307-312`）

```c
static void EstimateSpeed() {
    // 根据电机速度和陀螺仪的角速度进行解算...
    // chassis_feedback_data.vx vy wz =
    //  ...
}
```

**空函数 / 占位**。用途是「由四个轮速反馈反推底盘真实速度（可用于打滑检测等）」，目前未实现。

### 7.11 主任务 `ChassisTask()`（`chassis.c:315-390`）

这是底盘的核心循环，**建议 200Hz 运行**（`CHASSIS_TASK_DT = 0.005`），依次做 8 件事：

**Step 1 — 获取新控制命令（`chassis.c:319-324`）**

```c
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif
```

单板从消息中心取，双板从 CANComm 取（强转成 `Chassis_Ctrl_Cmd_s`）。

**Step 2 — 设置功率限制（`chassis.c:326`）**

```c
SetPowerLimit(referee_data->GameRobotState.chassis_power_limit);
```

把裁判系统下发的底盘功率限制传给功率控制模块。

**Step 3 — 急停判断（`chassis.c:327-340`）**

```c
if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
    DJIMotorStop(motor_lf); ...（4 个）
} else {
    DJIMotorEnable(motor_lf); ...（4 个）
}
```

- `CHASSIS_ZERO_FORCE` 时**直接给电机电流置零**（`DJIMotorStop` 不是设参考 0，而是发 0 电流，见 `dji_motor.h:110`）。
- 正常则 `DJIMotorEnable` 让电机响应参考值。

**Step 4 — 根据模式计算旋转速度 `wz`（`chassis.c:343-356`）**

```c
switch (chassis_cmd_recv.chassis_mode) {
case CHASSIS_NO_FOLLOW:          // 底盘不旋转，维持全向机动（调云台时用）
    chassis_cmd_recv.wz = 0;
    break;
case CHASSIS_FOLLOW_GIMBAL_YAW:  // 跟随云台：用误差角度的平方做非线性 P 输出
    chassis_cmd_recv.wz = -1.5f * offset_angle * fabsf(offset_angle);
    break;
case CHASSIS_ROTATE:             // 小陀螺：恒定自旋
    chassis_cmd_recv.wz = CHASSIS_ROTATE_WZ;
    break;
default:
    break;
}
```

- `FOLLOW_GIMBAL_YAW` 是一个**简化的角度环**：`wz = -1.5 * θ * |θ|`，误差越大转得越快（平方项），符号负表示收敛到 0，没单独用 PID。
- `ROTATE` 固定 `CHASSIS_ROTATE_WZ`（4000°/s），注释里说后续会加「不规则变速策略」。

**Step 5 — 坐标系旋转（`chassis.c:360-365`）**

```c
theta = (offset_angle + CHASSIS_CMD_DIR_TRIM_DEG) * DEGREE_2_RAD;
cos_theta = arm_cos_f32(theta);
sin_theta = arm_sin_f32(theta);
chassis_vx = vx * cos_theta - vy * sin_theta;
chassis_vy = vx * sin_theta + vy * cos_theta;
```

- 这是标准 **2D 旋转矩阵 R(θ)**，把「云台系」的速度命令投影到「底盘系」。
- `CHASSIS_CMD_DIR_TRIM_DEG = -45°`：实车标定的固定零偏。`OmniCalculate()` 虽已按 45° 轮子布置投影，但实车 X 型全向轮的机械安装/IMU 零位与底盘前进方向差 45°，需此偏置修正，否则「前进」会偏 45°。
- 用 ARM DSP 的 `arm_cos_f32/arm_sin_f32` 代替 `math.h` 的三角函数，更省 CPU。

**Step 6 — 正运动学解算（`chassis.c:368`）**

```c
OmniCalculate();   // 或 MecanumCalculate()，由编译开关决定
```

**Step 7 — 限幅并设定参考值（`chassis.c:371`）**

```c
LimitChassisOutput();
```

**Step 8 — 逆解算真实速度（占位）+ 推送反馈（`chassis.c:374-389`）**

```c
EstimateSpeed();   // 目前为空

#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif
```

---

## 8. 完整数据流

```
                    ┌──────────────────────────────────────────────┐
                    │              robot_cmd (cmd 应用)             │
                    │  读遥控器/键鼠 → 生成 vx/vy/wz/mode/offset    │
                    └───────────────┬──────────────────────────────┘
                                    │ 发布 "chassis_cmd"
                                    │ (单板: PubSub / 双板: CANComm)
                                    ▼
                    ┌──────────────────────────────────────────────┐
                    │                ChassisTask()                  │
                    │                                              │
                    │  1. 订阅/接收  Chassis_Ctrl_Cmd_s             │
                    │  2. SetPowerLimit(裁判系统功率限制)            │
                    │  3. 急停? → DJIMotorStop : DJIMotorEnable    │
                    │  4. 按模式算 wz                               │
                    │  5. 坐标旋转(云台系→底盘系) → chassis_vx/vy   │
                    │  6. Omni/Mecanum 正运动学 → vt_lf..vt_rb      │
                    │  7. 斜坡限幅 → DJIMotorSetRef(四个电机)        │
                    │  8. EstimateSpeed(空) → 回传反馈              │
                    └───────┬────────────────────────┬─────────────┘
                            │ 速度环参考                 │ 发布 "chassis_feed"
                            ▼                            ▼
                 ┌────────────────────┐        （回传给 robot_cmd 订阅）
                 │ 电机模块 DJIMotor  │
                 │ 速度环+电流环 PID   │
                 │ CAN 发电流到 M3508 │
                 └────────────────────┘
```

---

## 9. 与其他模块的依赖关系

| 模块 | 头文件 | 底盘用它做什么 |
| --- | --- | --- |
| `robot_cmd`（cmd） | `robot_cmd.h` | 控制命令的生产者（底盘通过消息中心/CAN 订阅它） |
| 消息中心 | `message_center.h` | 单板模式下的 PubSub：`SubRegister/SubGetMessage/PubPushMessage` |
| CAN 通信 | `can_comm.h` | 双板模式下的跨板通信：`CANCommInit/Get/Send` |
| DJI 电机 | `dji_motor.h`（经 `motor_def.h`） | 四个 M3508 的初始化与速度环控制 |
| 功率控制 | `power_control.h` | `SetPowerLimit()` 设底盘功率上限 |
| 超级电容 | `super_cap.h` | `SuperCapInit()` 初始化，为缓冲能量供能（TODO） |
| 裁判系统 | `referee_task.h` / `referee_UI.h` | 读裁判数据、功率限制、UI 显示 |
| IMU | `ins_task.h` | 双板时 `INS_Init()` 获取底盘转动角速度 |
| DWT 计时 | `bsp_dwt.h` | `DWT_GetTimeline_us()` 给 JScope 打时间戳 |
| ARM DSP | `arm_math.h` | `arm_cos_f32/arm_sin_f32` 三角函数 |
| 通用定义 | `general_def.h` | `PI`、`DEGREE_2_RAD` 等常量 |

---

## 10. 重写底盘前的检查清单

如果你想日后自己重写这个底盘，下面是要把握的「关键决策点」：

1. **运动学类型**：麦轮 `MECANUM` 还是 X 型全向轮 `OMNI_X`？两种正解公式不同，且要确认轮子安装方向（决定公式里的正负号）。
2. **板类型**：单板 `ONE_BOARD`（消息中心）还是双板 `CHASSIS_BOARD`（CANComm + IMU）？这决定收发方式。
3. **坐标约定**：确认「前进=+x、右=+y、CCW=正」和你车是否一致；`CHASSIS_CMD_DIR_TRIM_DEG` 的零偏（实车标定为 -45°）；改轮子安装方向或 IMU 零位后需重新标定。
4. **物理参数**：`WHEEL_BASE / TRACK_WIDTH / RADIUS_WHEEL / REDUCTION_RATIO_WHEEL`、云台偏置都要按实车改。
5. **单位一致性（重点）**：明确 `vx/vy/wz` 的输入单位，以及电机速度环参考值的单位（°/s），**补上「线速度 → 轮角速度」的显式换算**（原代码靠 PID 标定兜底，这是最需要重构的地方）。
6. **控制模式语义**：`ZERO_FORCE / ROTATE / NO_FOLLOW / FOLLOW_GIMBAL_YAW` 各模式应产生什么行为，`FOLLOW` 的角度环是手写二次项还是换成真正的 PID。
7. **限幅与安全**：保留斜坡限幅（`RampLimit`）；急停路径要确实走「0 电流」而不是「0 参考」。
8. **功率与缓冲**：把 `power_control`、`buffer_PID`、超级电容真正接入（原代码是 TODO 占位）。
9. **反馈数据**：决定 `Chassis_Upload_Data_s` 里到底回传什么（真实速度、热量、弹速、敌我颜色…），并在 `EstimateSpeed()` 里真正算出来。
10. **任务频率**：`CHASSIS_TASK_DT = 0.005`（200Hz），要和 RTOS 调度频率、斜坡限幅的 `SLEW_RATE` 一起对应，改动频率时要同步改限幅斜率。

---

*本文档基于 `application/chassis/chassis.c`、`chassis.h`、`robot_def.h`、`application/cmd/robot_cmd.c` 以及相关模块头文件整理，行号对应撰写时源码版本。*
