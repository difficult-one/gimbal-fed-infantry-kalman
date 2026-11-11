# 如何新增一个 Application（应用）

> 本文档面向本框架（HNU YueLu EC `basic-framework`）的应用层开发。
> 阅读前提：先理解「BSP / Module / APP 三层架构」以及「应用之间通过 message_center 以发布-订阅通信」这两个核心思想（详见 `.Doc/架构介绍与开发指南.md` 与 `application/APP层应用编写指引.md`）。
> 本文按**可照抄的步骤**组织，最后给出一个最小可编译示例。

---

## 目录

1. [先理解：Application 是什么](#1-先理解application-是什么)
2. [一个 Application 的完整组成](#2-一个-application-的完整组成)
3. [新增 Application 的完整步骤](#3-新增-application-的完整步骤)
4. [最小可编译示例（lift 抬升机构）](#4-最小可编译示例lift-抬升机构)
5. [应用间通信（PubSub）详解](#5-应用间通信pubsub详解)
6. [双板通信（可选）](#6-双板通信可选)
7. [编译配置：Makefile / CubeMX](#7-编译配置makefile--cubemx)
8. [命名与代码规范](#8-命名与代码规范)
9. [注意事项与常见坑](#9-注意事项与常见坑)
10. [新增后自检清单](#10-新增后自检清单)

---

## 1. 先理解：Application 是什么

一个 application 是对机器人**某一个机械/控制单元**的抽象（云台、底盘、发射、抬升、机械臂……）。它的职责是：

```
订阅 robot_cmd 发布的控制命令  →  按模式把命令转成执行单元目标值  →  调模块接口执行  →  发布反馈
```

现有应用及职责（参考 `application/application.md`）：

| 应用 | 目录 | 职责 |
| --- | --- | --- |
| robot_cmd | `application/cmd/` | 机器人大脑：读遥控器/键鼠/视觉 → 生成定量的控制命令 → 发往各执行应用 |
| chassis | `application/chassis/` | 底盘：运动学解算，驱动 4 个底盘电机 |
| gimbal | `application/gimbal/` | 云台：yaw/pitch 电机 + IMU |
| shoot | `application/shoot/` | 发射：摩擦轮、拨盘、弹舱盖 |

**关键约定**：应用之间**互不包含、平行工作**，通信只走 message_center 的发布-订阅。永远不要用全局变量在应用间传数据。

---

## 2. 一个 Application 的完整组成

以 `lift`（抬升机构，工程机器人）为例，一个应用由这些文件构成：

```
application/
├── lift/
│   ├── lift.c      # 应用实现（模块实例、初始化、任务循环）
│   ├── lift.h      # 对外接口（Init / Task 两个函数声明）
│   └── lift.md     # 说明文档（总体说明、代码结构、接口、范例）
├── robot_msg.h     # （共享）定义 lift 的控制/反馈结构体与模式枚举
├── robot.c         # （共享）注册 lift 的 Init/Task
└── cmd/robot_cmd.c # （共享）给 lift 注册 cmd 发布者/反馈订阅者，生成命令
```

---

## 3. 新增 Application 的完整步骤

下面 9 步覆盖了从「新建目录」到「能编译运行」的全部内容。步骤 1~3 是新建应用本体，步骤 4~6 是把它接入整车，步骤 7~9 是通信与编译。

### Step 1 — 建立目录和文件

在 `application/` 下新建子目录，命名用**小写**（与现有 `chassis`/`gimbal`/`shoot` 一致）：

```
application/lift/lift.c
application/lift/lift.h
application/lift/lift.md
```

### Step 2 — 在 `lift.h` 声明对外接口

应用对外**只暴露两个函数**（这是本框架统一的 app 接口风格，参考 `chassis.h`、`shoot.h`）：

```c
#ifndef LIFT_H
#define LIFT_H

/**
 * @brief 抬升机构初始化，请在开启 RTOS 之前调用（会被 RobotInit() 调用）
 */
void LiftInit();

/**
 * @brief 抬升机构任务，放入实时系统以一定频率运行（会被 RobotTask() 调用）
 */
void LiftTask();

#endif // LIFT_H
```

### Step 3 — 在 `lift.c` 实现应用

一个应用实现遵循固定骨架：

```c
#include "lift.h"
#include "robot_msg.h"
#include "message_center.h"   // 单板 PubSub
#include "dji_motor.h"        // 假设抬升用 DJI 电机
// ... 其他用到的 module 头文件

/* ============ 模块实例指针（static，单例） ============ */
static DJIMotorInstance *lift_motor;

/* ============ 通信：发布者/订阅者 + 收发数据 ============ */
static Subscriber_t *lift_sub;
static Publisher_t *lift_pub;
static Lift_Ctrl_Cmd_s   lift_cmd_recv;      // 订阅 robot_cmd 的命令
static Lift_Upload_Data_s lift_feedback_data; // 回传给 robot_cmd 的反馈

void LiftInit()
{
    /* 1. 初始化本应用拥有的模块（电机等） */
    Motor_Init_Config_s cfg = { /* ... 配置 ... */ };
    lift_motor = DJIMotorInit(&cfg);

    /* 2. 注册通信话题（名字要与 robot_cmd 侧完全一致） */
    lift_sub = SubRegister("lift_cmd",  sizeof(Lift_Ctrl_Cmd_s));
    lift_pub = PubRegister("lift_feed", sizeof(Lift_Upload_Data_s));
}

void LiftTask()
{
    /* 1. 获取控制命令 */
    SubGetMessage(lift_sub, &lift_cmd_recv);

    /* 2. 急停/使能处理 */
    if (lift_cmd_recv.lift_mode == LIFT_ZERO_FORCE) {
        DJIMotorStop(lift_motor);
    } else {
        DJIMotorEnable(lift_motor);
    }

    /* 3. 按模式转换控制目标并设定电机参考值 */
    switch (lift_cmd_recv.lift_mode) {
    case LIFT_UP:   DJIMotorSetRef(lift_motor,  500); break;
    case LIFT_DOWN: DJIMotorSetRef(lift_motor, -500); break;
    default:        DJIMotorSetRef(lift_motor,    0); break;
    }

    /* 4. 回传反馈 */
    PubPushMessage(lift_pub, (void *)&lift_feedback_data);
}
```

### Step 4 — 在 `robot_msg.h` 定义通信数据结构与枚举

应用间传输的数据结构**必须**定义在 `robot_msg.h` 中（共享），并遵守两条规则：

1. 用 `#pragma pack(1)` / `#pragma pack()` 包裹，**取消字节对齐**（因为数据可能通过 CAN 传输）。
2. 只含 POD 数据（float / int / enum / 定长数组），不能有指针。

在 `robot_msg.h` 的「CMD 应用发布控制数据」区域加：

```c
/* 抬升模式 */
typedef enum {
    LIFT_ZERO_FORCE = 0,  // 零力
    LIFT_UP,              // 上升
    LIFT_DOWN,            // 下降
} lift_mode_e;

/* cmd 发布的抬升控制数据，由 lift 订阅 */
typedef struct {
    float lift_vel;       // 抬升速度目标
    lift_mode_e lift_mode;
} Lift_Ctrl_Cmd_s;

/* lift 发布的反馈数据，由 cmd 订阅 */
typedef struct {
    float real_height;    // 当前高度
} Lift_Upload_Data_s;
```

> 这些 `typedef` 建议与现有的 `Chassis_Ctrl_Cmd_s`、`Gimbal_Ctrl_Cmd_s` 放在同一区域，保持集中、易维护。

### Step 5 — 在 `robot.c` 注册应用

在 `robot.c` 中把新应用接进整车的初始化与任务循环（参考现有 `ChassisInit/ChassisTask`）：

```c
#include "lift.h"

void RobotInit()
{
    __disable_irq();
    BSPInit();
    RobotCMDInit();
    //GimbalInit();
    //ShootInit();
    ChassisInit();
    LiftInit();          // ← 新增：初始化
    __enable_irq();
}

void RobotTask()
{
    RobotCMDTask();
    //GimbalTask();
    //ShootTask();
    ChassisTask();
    LiftTask();          // ← 新增：任务
}
```

> **任务频率**：`RobotTask()` 由 FreeRTOS 任务 `robottask` 以 **200Hz**（`osDelay(5)`）调度（见 `Core/Src/freertos.c:179-187`），所有 app 的 `XxxTask()` 在它内部**顺序执行**，共享 200Hz。如果你的应用需要不同频率（例如 1kHz），才需要像 `motortask`/`instask` 那样在 `freertos.c` 单独建 RTOS 任务——绝大多数 app 不需要。

### Step 6 — 在 `robot_cmd.c` 接入控制数据（标准做法）

如果你的应用是被 robot_cmd 控制的**执行机构**（绝大多数情况），还要改 `robot_cmd.c`：

1. **注册通信实例**（在 `RobotCMDInit()` 里，参考 `robot_cmd.c:60-68`）：

```c
lift_cmd_pub  = PubRegister("lift_cmd",  sizeof(Lift_Ctrl_Cmd_s));
lift_feed_sub = SubRegister("lift_feed", sizeof(Lift_Upload_Data_s));
```

2. **在遥控器/键鼠映射里生成命令**（在 `RemoteControlSet()` / `MouseKeySet()` 里）：

```c
// RemoteControlSet() 内，参考 chassis 的 vx/vy 写法
lift_cmd_send.lift_vel = 10.0f * (float)ChassisRCDeadBand(rc_data[TEMP].rc.rocker_l_);
```

3. **在 `RobotCMDTask()` 里订阅反馈 + 推送命令**（参考 `robot_cmd.c:291-321`）：

```c
SubGetMessage(lift_feed_sub, &lift_fetch_data);
// ...
PubPushMessage(lift_cmd_pub, (void *)&lift_cmd_send);
```

4. **在 `EmergencyHandler()` 里处理急停**（参考 `robot_cmd.c:260-283`）：

```c
// 急停时
lift_cmd_send.lift_mode = LIFT_ZERO_FORCE;
```

### Step 7 — 确认话题名精确配对

发布者和订阅者通过**字符串话题名**配对（大小写敏感，必须逐字符一致）：

| 数据方向 | 发布者注册（robot_cmd） | 订阅者注册（lift） |
| --- | --- | --- |
| cmd → lift | `PubRegister("lift_cmd", ...)` | `SubRegister("lift_cmd", ...)` |
| lift → cmd | `SubRegister("lift_feed", ...)` | `PubRegister("lift_feed", ...)` |

> 现有话题名一览（供参考，勿重名）：`gimbal_cmd` / `gimbal_feed` / `shoot_cmd` / `shoot_feed` / `chassis_cmd` / `chassis_feed`。

### Step 8 — 编译配置（Makefile）

本工程用 **Makefile** 编译，源文件是**手动列举**的（不是自动扫描）。在根目录 `Makefile` 中加两处：

1. 在 `C_SOURCES` 里加源文件（`Makefile:120-124` 附近）：

```make
application/gimbal/gimbal.c \
application/chassis/chassis.c \
application/shoot/shoot.c \
application/lift/lift.c \          # ← 新增
application/cmd/robot_cmd.c \
```

2. 在 `C_INCLUDES` 里加头文件搜索路径（`Makefile:216-220` 附近）：

```make
-Iapplication/chassis \
-Iapplication/shoot \
-Iapplication/gimbal \
-Iapplication/lift \               # ← 新增
-Iapplication/cmd \
```

> ⚠️ **重要**：`Makefile` 是 CubeMX 自动生成的（文件头 `File automatically-generated by tool`）。若之后用 CubeMX 重新生成工程，手动加的这两行会**被覆盖**，需要重新添加。正式做法是在 CubeMX 的 Project Manager 里把源文件夹加进工程再重新生成；日常开发直接手改 Makefile 更快。

### Step 9 — 编译验证

```bash
mingw32-make          # 或 make
mingw32-make clean    # 先清理再全量编译（新增文件后建议 clean）
```

确认 `build/` 下生成 `lift.o`、`basic_framework_ATYPE.elf/.hex/.bin` 且无报错。

---

## 4. 最小可编译示例（lift 抬升机构）

一个**只订阅一个话题、驱动一个电机**的最小应用，完整照抄后把名字替换即可：

### `application/lift/lift.h`

```c
#ifndef LIFT_H
#define LIFT_H

void LiftInit();   // 初始化，被 RobotInit() 调用
void LiftTask();   // 任务，被 RobotTask() 调用

#endif // LIFT_H
```

### `application/lift/lift.c`

```c
#include "lift.h"
#include "robot_msg.h"
#include "dji_motor.h"
#include "message_center.h"

/* 本应用拥有的模块实例 */
static DJIMotorInstance *lift_motor;

/* 通信实例与数据 */
static Subscriber_t *lift_sub;
static Publisher_t *lift_pub;
static Lift_Ctrl_Cmd_s   lift_cmd_recv;
static Lift_Upload_Data_s lift_feedback_data;

void LiftInit()
{
    Motor_Init_Config_s cfg = {
        .can_init_config = { .can_handle = &hcan1, .tx_id = 6 },
        .controller_param_init_config = {
            .speed_PID = { .Kp = 5, .Ki = 0.1, .Kd = 0,
                           .Improve = PID_Integral_Limit,
                           .IntegralLimit = 5000, .MaxOut = 10000 },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    lift_motor = DJIMotorInit(&cfg);

    lift_sub = SubRegister("lift_cmd",  sizeof(Lift_Ctrl_Cmd_s));
    lift_pub = PubRegister("lift_feed", sizeof(Lift_Upload_Data_s));
}

void LiftTask()
{
    SubGetMessage(lift_sub, &lift_cmd_recv);

    if (lift_cmd_recv.lift_mode == LIFT_ZERO_FORCE) {
        DJIMotorStop(lift_motor);
    } else {
        DJIMotorEnable(lift_motor);
    }

    switch (lift_cmd_recv.lift_mode) {
    case LIFT_UP:   DJIMotorSetRef(lift_motor,  1000); break;
    case LIFT_DOWN: DJIMotorSetRef(lift_motor, -1000); break;
    default:        DJIMotorSetRef(lift_motor,     0); break;
    }

    lift_feedback_data.real_height = 0; // TODO: 由电机编码器反推实际高度
    PubPushMessage(lift_pub, (void *)&lift_feedback_data);
}
```

### `application/lift/lift.md`

按框架规范写说明文档，至少包含：**总体说明、代码结构、外部接口和类型定义、私有函数和变量、使用说明与范例、注意事项**。

---

## 5. 应用间通信（PubSub）详解

message_center 是**伪发布-订阅**（`modules/message_center/message_center.h`），接口只有 4 个：

| 函数 | 作用 | 返回 |
| --- | --- | --- |
| `PubRegister(name, data_len)` | 注册成为某话题的发布者 | `Publisher_t*` |
| `SubRegister(name, data_len)` | 订阅某话题 | `Subscriber_t*` |
| `PubPushMessage(pub, data_ptr)` | 向所有订阅者推送数据 | 成功推送的订阅者数 |
| `SubGetMessage(sub, data_ptr)` | 取最新消息到 `data_ptr` | 0=无新消息，1=有新消息 |

**使用要点**：

- `data_len` 一律用 `sizeof(你的结构体)`。
- 同一个 `name` 的 `PubRegister` 和 `SubRegister` 会通过话题名链表自动关联，**无需额外绑定**。
- 数据按 `memcpy` 复制，`data_ptr` 指向发布/接收方的内存即可。
- 订阅后读到的消息放在局部结构体变量里，之后随意使用。

**内置上限**（`message_center.h:18-20`，新增大量话题时注意）：

```c
#define MAX_EVENT_NAME_LEN 32  // 话题名最长 32 字符
#define MAX_EVENT_COUNT 12     // 话题（事件）总数上限
#define QUEUE_SIZE 1           // 每个订阅者队列深度（只保留最新一条）
```

> 当前已用 6 个话题（gimbal/shoot/chassis 各 cmd+feed）。加一个 app 通常再加 2 个话题，仍在 12 以内；若超过，需要改 `MAX_EVENT_COUNT` 并重编译。

---

## 6. 双板通信（可选）

如果应用运行在**双板**架构下（由 `EXTRA_DEFS` 选择 `CHASSIS_BOARD` 或 `GIMBAL_BOARD`，见 `config/build_config.h`），跨板数据要走 `can_comm` 而不是 message_center。模式与 `chassis.c` / `robot_cmd.c` 完全一致：

```c
#ifdef GIMBAL_BOARD
#include "can_comm.h"
static CANCommInstance *lift_can_comm;
#endif

// Init 里：
CANComm_Init_Config_s comm_conf = {
    .can_config = { .can_handle = &hcan2, .tx_id = 0x313, .rx_id = 0x314 },
    .recv_data_len = sizeof(Lift_Upload_Data_s),
    .send_data_len = sizeof(Lift_Ctrl_Cmd_s),
};
lift_can_comm = CANCommInit(&comm_conf);

// Task 里（替换掉 SubGetMessage / PubPushMessage）：
lift_cmd_recv = *(Lift_Ctrl_Cmd_s *)CANCommGet(lift_can_comm);
CANCommSend(lift_can_comm, (void *)&lift_feedback_data);
```

要点：
- **两侧 id 要配对**：本板 `tx_id` = 对板 `rx_id`，本板 `rx_id` = 对板 `tx_id`。
- `tx_id` 不能与同一条 CAN 总线上的电机（`0x200+id`）、超级电容（`0x301/0x302`）等冲突。
- 经 CAN 传输的结构体**必须** `#pragma pack(1)`（`can_comm.h:77-81` 有明确说明）。

---

## 7. 编译配置：Makefile / CubeMX

- 当前使用 **Makefile + arm-none-eabi-gcc**（`mingw32-make`），源文件**手动列举**。
- 新增 app 需要改两处：`C_SOURCES`（加 `.c`）和 `C_INCLUDES`（加 `-I` 路径）。
- **文件名必须全局唯一**：Makefile 用 `$(notdir ...)` 提取文件名生成目标文件（`Makefile:294`），不同目录下同名 `.c` 会互相覆盖。所以 `lift.c` 不能和任何已有 `.c` 重名。
- Makefile 是 CubeMX 生成的，重新生成工程会覆盖手改内容。

---

## 8. 命名与代码规范

来自 `.Doc/架构介绍与开发指南.md`，写新 app 时务必遵守：

| 项目 | 规范 | 示例 |
| --- | --- | --- |
| 函数 | 动宾短语、≤4 词、单词首字母大写 | `SetMotorControl()` |
| 变量 | 下划线命名、小写、表意清晰 | `gimbal_recv_cmd` |
| 私有变量/函数 | `static`，私有变量加 `_` 前缀（可选） | `static float _offset;` |
| 复杂结构体 | `typedef` 首字母大写 + 下划线 + `_s` 后缀 | `Lift_Ctrl_Cmd_s` |
| 简单类型 | `_t` 后缀 | `IMU_Data_t` |
| 模块实例类型 | `xxxInstance` | `SuperCapInstance` |
| 对外接口注释 | 写在 `.h` 中（doxygen 风格 `@brief`） | 见 `chassis.h` |
| 私有函数注释 | 写在 `.c` 中 | 见 `chassis.c` |

**其他硬性要求**：
- 应用层**禁止**出现 `HAL_...` 调用（应使用 `BSP_...` 或 module 接口）。
- 通信结构体**必须** `#pragma pack(1)` 包裹。
- 加入**安全检查**（"treat your users as idiots!"），对未知模式用 `default` 兜底（参考 `shoot.c:166-168` 的 `while(1);` 占位）。

---

## 9. 注意事项与常见坑

1. **不要在应用 Task 里做初始化**。所有模块初始化必须在 `XxxInit()` 里完成（`RobotInit()` 在开启 RTOS 之前调用）。架构文档明确：*"应用初始化不得放入其对应任务中，否则可能导致需要定时器的任务初始化异常。"*
2. **`RobotInit()` 期间中断是关闭的**（`robot.c:15` `__disable_irq()`），初始化里不要用会依赖中断的延时/通信，只能用 `DWT_Delay()`。
3. **话题名拼写错误 = 静默收不到数据**。`SubGetMessage` 返回 0 不会报错，务必核对发布/订阅两侧字符串一致。
4. **订阅与发布顺序**：先注册（`SubRegister`/`PubRegister`，在 `XxxInit()`），后收发（`SubGetMessage`/`PubPushMessage`，在 `XxxTask()`）。
5. **`Motor_Init_Config_s` 的 `tx_id` 不能重复**，且不能与 CAN 总线其他设备冲突（M3508/M2006 反馈是 `0x200+id`，GM6020 是 `0x204+id`）。
6. **急停路径**：`CHASSIS_ZERO_FORCE`/`SHOOT_OFF` 这类零力模式要调 `DJIMotorStop()`（发 0 电流），而不是 `SetRef(0)`——后者电机仍在闭环出力。
7. **反馈数据要真正赋值**：很多应用（如当前 `chassis.c`）的 `Xxx_Upload_Data_s` 字段是空壳/注释占位，回传前确认填充了有效数据。
8. **别改 CubeMX 生成的文件**（`Core/Src/*`、`Makefile` 会自动覆盖），用户代码放 `application/` 层。

---

## 10. 新增后自检清单

逐条打勾，全部通过才算接入完成：

- [ ] `application/xxx/xxx.c`、`xxx.h`、`xxx.md` 已创建，`.md` 按规范写了说明
- [ ] `xxx.h` 只有 `XxxInit()` / `XxxTask()` 两个对外函数，且带注释
- [ ] `xxx.c` 中模块实例、通信实例均为 `static` 单例
- [ ] 通信结构体与枚举已在 `robot_msg.h` 定义，且用 `#pragma pack(1)` 包裹
- [ ] `robot.c` 的 `RobotInit()` 加了 `XxxInit()`，`RobotTask()` 加了 `XxxTask()`
- [ ] `robot_cmd.c` 已注册对应 `PubRegister`/`SubRegister` 并生成命令（被控执行机构场景）
- [ ] 发布/订阅话题名两侧**完全一致**，且不与现有 6 个话题重名
- [ ] `Makefile` 的 `C_SOURCES` 加了 `.c`，`C_INCLUDES` 加了 `-I` 路径
- [ ] 新 `.c` 文件名全局唯一
- [ ] `mingw32-make clean && mingw32-make` 编译通过，`build/xxx.o` 生成
- [ ] 急停/零力路径走 `DJIMotorStop`，未知模式有 `default` 兜底
- [ ] 应用层无 `HAL_...` 调用

---

*本文档基于 `application/`、`robot.c`、`robot_msg.h`、`Makefile`、`modules/message_center` 等源码与 `.Doc/架构介绍与开发指南.md` 整理，行号对应撰写时源码版本。*
