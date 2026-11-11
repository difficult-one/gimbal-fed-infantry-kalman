# jam_detector 接入 shoot.c 指南

> 目标：把 `modules/jam_detector` 的卡弹检测 FSM 接入发射任务 `ShootTask`，让拨盘电机在卡弹（电流持续超阈值）时自动反转处理。

## 0. 前置条件

接入前确认下面两项已经完成，否则代码编不进固件或编进去也跑不起来：

1. **Makefile 路径已修正**：`Makefile` 源文件列表里必须是
   ```makefile
   modules/jam_detector/jam_detector.c \
   ```
   （之前拼成了 `moudles/...`，已改回 `modules`。）

2. **shoot.c 已包含头文件**（当前已经加了）：
   ```c
   #include "jam_detector.h"
   ```

---

## 1. 声明检测器实例（文件顶部）

在 [shoot.c](shoot.c) 的全局变量区，`loader` 声明附近加一行：

```c
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
static JamDetectorInstance *jam;                          // 卡弹检测器实例 (新增)
// static servo_instance *lid; 需要增加弹舱盖
```

---

## 2. 初始化（ShootInit 末尾）

在 `ShootInit()` 里，`loader = DJIMotorInit(&loader_config);` **之后**、注册 pub/sub 之前，加入：

```c
    loader = DJIMotorInit(&loader_config);

    /* ---- 卡弹检测器初始化 (新增) ----
     * 传入的配置项为 0 时会自动使用 jam_detector.c 内部默认值,
     * 这里显式填上, 便于后续实测后直接改这一处即可覆盖默认值 */
    JamDetector_Init_Config_s jam_conf = {
        .current_threshold   = 4000.0f,   // M2006/C610电流原始值; 必须低于速度环MaxOut=5000, 再按实测lCur调整
        .suspect_timeout_ms  = 300.0f,    // 电流持续超阈值多久才确认卡弹
        .handling_timeout_ms = 1500.0f,   // 电机轴回退1620度需要更长时间, 防止未停稳就恢复正转
        .reverse_angle       = LOADER_ONE_BULLET_MOTOR_ANGLE, // 卡弹后回退一发对应的电机轴角度
    };
    jam = JamDetectorInit(&jam_conf, loader);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
```

> 如果暂时不想自定义参数，直接 `JamDetector_Init_Config_s jam_conf = {0};` 即可全部用默认值。

---

## 3. 急停时复位（ShootTask 的 SHOOT_OFF 分支）

在 `ShootTask()` 开头的停止分支里加一行 `JamDetectorReset`，保证急停/离线时卡弹状态归零，恢复运行后从 NORMAL 重新开始：

```c
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
        JamDetectorReset(jam);   // 急停时复位卡弹状态 (新增)
    }
```

---

## 4. 轮询 FSM 并跳过常规拨盘控制（核心）

这是最关键的一步。`JamDetectorTask()` 内部在检测到卡弹时会直接对拨盘电机调用
`DJIMotorOuterLoop(ANGLE_LOOP)`，并把目标设置为“当前角度 - `reverse_angle`”，
**因此处理期间必须跳过下面原本的 `switch (load_mode)` 常规控制**，否则这个位置目标会立刻被覆盖。

把 [shoot.c](shoot.c) 里原来的这段：

```c
    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    ...
    }
```

改成下面这样（在 switch 前先轮询，再按状态决定是否执行）：

```c
    // ---- 卡弹检测轮询 (新增) ----
    JamState_e jam_state = JamDetectorTask(jam);

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    // 卡弹确认/回退处理期间跳过常规控制, 回退位置由 JamDetector 内部设定
    if (jam_state == JAM_CONFIRMED || jam_state == JAM_HANDLING)
    {
        /* 卡弹处理中: 保持 JamDetector 设置的回退位置, 不进入常规 load_mode 控制 */
    }
    else
    {
        switch (shoot_cmd_recv.load_mode)
        {
        // 停止拨盘
        case LOAD_STOP:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, 0);
            break;
        // 单发
        case LOAD_1_BULLET:
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, loader->measure.total_angle + LOADER_ONE_BULLET_MOTOR_ANGLE);
            hibernate_time = DWT_GetTimeline_ms();
            dead_time = 150;
            break;
        // 三连发
        case LOAD_3_BULLET:
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, loader->measure.total_angle + 3 * LOADER_ONE_BULLET_MOTOR_ANGLE);
            hibernate_time = DWT_GetTimeline_ms();
            dead_time = 300;
            break;
        // 连发
        case LOAD_BURSTFIRE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
            break;
        // 反转
        case LOAD_REVERSE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            break;
        default:
            while (1)
                ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
        }
    }
```

> 原理说明：`JamDetectorTask()` 返回的状态里，`JAM_CONFIRMED` 和 `JAM_HANDLING` 都是「正在回退处理卡弹」，此时要跳过常规控制；`JAM_NORMAL` 和 `JAM_SUSPECT` 都让发射机构正常走 `load_mode` 逻辑。

---

## 5. （可选）调试打印加上卡弹状态

原调试打印在第 6 行参数里再加一个 `jam=%d`，方便观察 FSM 流转和电流值：

```c
        SEGGER_RTT_printf(0, "[shoot] mode=%d fric=%d load=%d fRef=%d fSpd=%d fCur=%d lSpd=%d lCur=%d stop=%d jam=%d\r\n",
                          (int)shoot_cmd_recv.shoot_mode,
                          (int)shoot_cmd_recv.friction_mode,
                          (int)shoot_cmd_recv.load_mode,
                          (int)friction_l->motor_controller.pid_ref,
                          (int)friction_l->measure.speed_aps,
                          (int)friction_l->measure.real_current,
                          (int)loader->measure.speed_aps,
                          (int)loader->measure.real_current,
                          (int)friction_l->stop_flag,
                          (int)jam_state);
```

---

## 6. current_threshold 怎么定（重要）

`real_current` 存的是 **CAN 原始值**，不是安培。当前 `shoot.c` 中 M2006 拨盘速度环的 `MaxOut` 是 `5000`，也就是发送给 C610 的电流参考上限；阈值设成 `9830` 高于这个控制范围，通常无法触发检测条件。

阈值必须先低于拨盘控制输出上限，再通过 `lCur` 实测校准：

典型值参考：

| 状态 | 参考值 |
|---|---|
| 速度环输出上限 | `5000` |
| 当前建议起步阈值 | `4000` |

**精准做法（必须实测）**：用第 5 节的 `lCur` 打印：

1. 正常连发，记录 `lCur` 峰值 `I_normal`；
2. 手动卡住拨盘，记录堵转值 `I_stall`；
3. 阈值取 `(I_normal + I_stall) / 2`，略偏低，填回 `jam_conf.current_threshold`。

---

## 7. 状态机回顾（对照理解代码）

```
NORMAL ──电流超阈值──> SUSPECT ──持续300ms──> CONFIRMED ──回退固定角度──> HANDLING ──到位/超时──> NORMAL
   ↑                      │                                                              │
   └────电流恢复正常──────┘                                                              └──超时回到正常
```

- `JAM_SUSPECT`：电流刚超阈值，还没持续够时间，属「嫌疑」，拨盘照常工作；
- `JAM_CONFIRMED`：确认卡弹，本周期设置回退位置（外部应跳过常规控制）；
- `JAM_HANDLING`：回退处理中，持续保持回退位置，直到到位或超时；
- 超时或电流恢复后回到 `JAM_NORMAL`，恢复正常拨弹。

---

## 8. 常见坑

1. **忘了在 SHOOT_OFF 分支加 `JamDetectorReset`**：急停后残留 `JAM_HANDLING` 状态，恢复运行时拨盘可能先反转一下。
2. **跳过逻辑漏写**：若不在回退期间跳过 `switch`，`JamDetector` 设的回退位置会被 `load_mode` 立即覆盖，卡弹处理永远不生效。
3. **`memset` 隐式声明告警**：`jam_detector.c` 用了 `memset`，但只 `#include <stdlib.h>`，若编译报 `implicit declaration of 'memset'`，在该文件顶部补 `#include <string.h>` 即可。
4. **阈值高于控制输出上限**：当前拨盘速度环 `MaxOut=5000`，阈值不能照搬 `9830` 这类高于上限的数值，应按 `lCur` 实测调整。
