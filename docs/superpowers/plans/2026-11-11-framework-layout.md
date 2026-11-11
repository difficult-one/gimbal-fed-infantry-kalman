# 步兵电控框架整理实施方案

**目标**　按跃鹿 A 板框架的职责划分整理当前工程。通过目录和短文件名即可定位业务、设备驱动和板级接口，整车参数统一到 `config.c`，对应配置结构体声明在 `config.h`。

**工程环境**　STM32F427、CubeMX、STM32 HAL、FreeRTOS、GNU Make、ARM GCC、PowerShell、宿主 GCC 和 Python。

**范围**　本次交付方案。以下任务用于后续实施，每项完成后单独验证。

## 一、目标结构和边界

本地 README 已说明工程来自跃鹿 `basic_framework` 的 A 型板适配版。沿用 `application → modules → bsp` 的三层结构，按现有代码的职责归位。应用之间沿用已有 pub/sub 和板间 CAN 通路。参考 [跃鹿 basic_framework](https://github.com/HNUYueLuRM/basic_framework) 和 [跃鹿开源发布帖](https://bbs.robomaster.com/article/9272)。

```text
application/
  robot.c/.h               整车初始化和应用调度
  robot_msg.h              应用消息、控制模式、板间帧
  cmd/robot_cmd.c/.h        遥控和图传输入、整车命令
  chassis/
    chassis.c/.h           底盘应用、模式切换、电机和消息接线
    chassis_setup.c/.h     车型配置装配、弱覆盖、运动模块初始化
    chassis_force.c/.h     力控、导纳、力估计及其状态
    chassis_log.c/.h       低优先级运动日志和采集服务
  gimbal/gimbal.c/.h       云台控制
  shoot/shoot.c/.h         发射控制
  referee/chassis_ui.c/.h  机器人状态到裁判 UI 的映射
config/
  config.c                 车型、接线、控制和调试参数的唯一来源
  config.h                 车型配置类型及只读数据声明
  build_config.h           只在预处理阶段使用的构建选择和组合校验
modules/
  chassis_motion/          运动解算、轮组运动学、可移植读写接口
  imu/                     INS、MPU6500 SPI 驱动、补偿、温控
  diagnostics/             RTT 采集编码与缓存
  motor/                   电机驱动和调度
  referee/                 裁判协议、串口收发与通用绘图
  ...                      遥控、视觉通信、消息中心、算法等设备与服务
bsp/                       CAN、UART、SPI、GPIO、DWT、PWM 等板级封装
Core/ Drivers/ Middlewares/ CubeMX 生成代码、芯片与第三方代码
tests/ tools/ docs/         回归测试、离线工具、使用文档和实测资料
```

| 位置 | 放什么 | 依赖规则 |
| --- | --- | --- |
| `application/` | 机器人角色、控制模式、设备接线、应用协调 | 向下组装模块。保留板间消息和同步反馈已有的读写时序 |
| `modules/` | 电机、IMU、裁判协议、运动学等可复用实现 | 接收本模块的初始化参数或回调。模块头文件可独立于应用编译 |
| `bsp/` | STM32 外设封装和通用时基 | 向下使用 HAL 和板级句柄，业务决策由调用方负责 |
| `config/` | 当前机器人使用的标定值、限值、PID、接线和调试策略 | 应用读取并注入。可移植模块测试可直接传入自己的参数 |
| `Core/` | HAL 启动、外设初始化、RTOS 调度胶水 | 从 CubeMX 用户区调用现有入口。任务时序和初始化顺序按当前代码逐项对照 |

文件名使用小写 `snake_case`，优先选“对象 + 职责”，例如 `mpu6500_spi.c` 和 `chassis_log.c`。同一模块的头文件与实现使用相同的主名。已有明确含义的 `robot_cmd.c`、`chassis.c`、`gimbal.c`、`shoot.c` 继续沿用。Makefile 当前按文件主名生成对象文件，迁移时校验全工程 `.c`、`.s` 的主名冲突。

## 二、文件迁移表

| 现有文件 | 目标文件 | 职责和迁移检查 |
| --- | --- | --- |
| `application/robot_def.h` | `config/config.c`、`config/config.h`、`config/build_config.h`、`application/robot_msg.h` | 依次分离车型参数、编译选项和消息定义。保留枚举值、条件字段、对齐、话题及板间字节布局 |
| `application/chassis/chassis_motion_config.h` | `config/config.c/.h` | 收拢整车运动配置，保留现有覆盖方式。固定协议和内核容量归各自所有者，调用和脚本迁完后移除旧头 |
| `application/chassis/chassis_motion_project.c` | `application/chassis/chassis_setup.c/.h` | 装配对应轮型和当前车型参数。保留 `ChassisMotionInit`、弱覆盖 `ChassisMotionCustomizeConfig` 及其调用顺序 |
| `application/chassis/chassis_motion.c/.h` | `modules/chassis_motion/chassis_motion.c/.h` | 保持状态估计、轮速分配和失效退化。应用初始化声明迁入 `chassis_setup.h` |
| `application/chassis/chassis_kinematics.c/.h` | `modules/chassis_motion/kinematics.c/.h` | 轮型几何、正逆解和约束分配，保持轮序和坐标约定 |
| `application/chassis/chassis_motion_port.c/.h` | `modules/chassis_motion/motion_port.c/.h` | 通过读写回调驱动运动内核。现有可移植性测试用到该入口 |
| `application/chassis/chassis.c` | 原文件 + `chassis_force.c/.h` + `chassis_log.c/.h` | 分两次提取低优先级日志和力控状态。初始化、应用模式仲裁、设备接线留在 `chassis.c` |
| `application/chassis/wheel_imu_capture.c/.h` | `modules/diagnostics/wheel_imu_capture.c/.h` + `chassis_log.c` | 模块提供记录、缓存和编码。应用准备标定元数据和轮序、保留布防与安全许可。设备模块从自己的采样点传入原始事件 |
| `modules/mpu6500/mpu6500.c/.h` | `modules/imu/ins.c/.h` | 文件实际负责 INS 任务、校准、姿态与运动快照 |
| `modules/mpu6500/mpu6500driver.c/.h` | `modules/imu/mpu6500_spi.c/.h` | 寄存器访问、SPI 传输、原始数据和错误状态 |
| `modules/mpu6500/{imu_compensation,imu_temperature,mpu6500_reg,ist8310_reg}.*` | `modules/imu/` 同名文件 | 保留现有职责与接口，随目录搬迁 |
| `modules/referee/referee_task.c/.h`、`rm_referee.h` 中的 UI 状态 | `application/referee/chassis_ui.c/.h` | UI 交互状态由应用持有。裁判数据包与通用绘图留在模块 |
| `modules/referee/rm_referee.c/.h`、`referee_UI.c/.h` | `modules/referee/referee_rx.c/.h`、`referee_draw.c/.h` | 区分裁判收发与画图。拆除对 `robot_def.h` 的引用，绘图序号由模块持有 |
| `modules/algorithm/controller.*`、`QuaternionEKF.*`、`MahonyAHRS.*` | `pid.*`、`quaternion_ekf.*`、`mahony_ahrs.*` | 控制器文件当前实现 PID。三组改名均保留现有公开函数名和算法 |
| `modules/general_def.h`、`modules/algorithm/user_lib.*` | `modules/algorithm/math_units.h`、`math_utils.*`、`modules/memory/mem_alloc.*` | 单位换算、数值工具、清零分配分别按职责归位。沿用现有 FreeRTOS/宿主分配路径 |
| `modules/jscope/rtt_debug.*`、`modules/referee/crc_ref.*` | `modules/diagnostics/jscope_rtt.*`、`modules/referee/referee_crc.*` | 标明 JScope 日志格式和裁判协议校验的用途 |
| `modules/motor/{DJImotor,DMmotor,HTmotor,LKmotor}/` | `modules/motor/{dji,dm,ht,lk}/` | 目录按厂商归类，`dmmotor.*` → `dm_motor.*`，`HT04.*` → `ht04.*`，`LK9025.*` → `lk9025.*` |
| `modules/{TFminiPlus,bluetooth,master_machine}/` | `modules/{tfmini,bluetooth,vision}/` | 距离传感器 `tfmini_plus.*`、蓝牙 `hc05.*`、视觉收发 `vision_link.*`。`seasky_protocol.*` 保持协议名 |

文件迁移同步修改 `#include`、include guard、Makefile 的 `C_SOURCES/C_INCLUDES`、实际引用路径的 VS Code 配置、宿主测试和文档。纯大小写改名通过临时名完成，并核对 Git 索引记录最终大小写。CubeMX、第三方库、启动文件和链接脚本沿用厂商命名。

## 三、集中配置的约定

每项整车部署参数、标定、调试开关和相关默认值集中在 `config.c`。小范围组合、数组初始化和派生公式可以使用该文件私有的宏。跨文件读取时由 `config.h` 声明结构体和 `extern const` 对象。

| 配置组 | 迁入内容 | 原值举例 |
| --- | --- | --- |
| `ChassisConfig` | 轮距/轴距、轮径、减速比、中心偏移、驱动方向、四轮电机 CAN 接线 | 轴距 350 mm、轮距 300 mm、轮半径 76 mm、减速比 19 |
| 底盘运动配置 | 采用现有 `ChassisMotionConfig` 作为运动模块接口。车型配置在 `config.h` 声明，`config.c` 保存实际标定和默认值 | 默认 `MC_SHADOW`、自旋提前 0.0135 s、LF/RF/LB/RB 微调 66/-107/29/12 dps |
| `ChassisForceConfig` | 电流换算、力控/导纳、斜率、零点偏移、起步补偿、超速刹车与时限 | 电流限幅原始量 8000、力传感器超时 20 个底盘周期 |
| `GimbalConfig`、`ShootConfig` | 机械零位、角度范围、方向、PID、摩擦轮/拨盘、卡弹、接线 | yaw/pitch 编码器零位 370/3412、pitch ±30°、拨盘每发 45°、减速比 36 |
| `InputConfig`、`ImuConfig` | 输入死区和倍率、IMU 安装与杆臂、校准门限、温控和滤波初值 | 遥控死区 10，IMU 采样间隙 20,000 µs |
| `LinkConfig` | 逻辑串口、板间 CAN 总线与收发 ID、超电 ID | DBUS USART1、视觉 USART3、VT02 UART8、裁判 USART6。底盘板 CAN2 发 0x311 收 0x312，云台板 CAN1 发 0x312 收 0x311 |
| `TaskConfig`、`LogConfig` | 自建任务的栈、优先级、延时，RTT/JScope 开关、通道、抽样周期 | 电机/守护/整车/INS/日志任务现有 `osDelay` 实参为 2/10/5/1/10 |

逐项中文注释写清用途和单位，带方向的量补坐标系、正方向或轮序，布尔量写清开关效果，经验标定注明出处。先保持数值、类型、符号、计算顺序和覆盖优先级，再判断同名参数是否真能共用。力控、运动限流等当前数值相同的参数按语义分组。

代表性写法如下。实际迁移可保留现有精度和派生公式。

```c
/* config.h */
typedef struct {
    float wheel_base_mm;
    float track_width_mm;
    float wheel_radius_mm;
} ChassisGeometryConfig;

extern const ChassisGeometryConfig g_chassis_geometry;

/* config.c */
#define CHASSIS_WHEEL_BASE_MM   350.0f  /* 前后轮轴中心间距，单位 mm。 */
#define CHASSIS_TRACK_WIDTH_MM  300.0f  /* 左右轮中心间距，单位 mm。 */
#define CHASSIS_WHEEL_RADIUS_MM  76.0f  /* 车轮半径，单位 mm，对应外径 152 mm。 */

const ChassisGeometryConfig g_chassis_geometry = {
    .wheel_base_mm = CHASSIS_WHEEL_BASE_MM,
    .track_width_mm = CHASSIS_TRACK_WIDTH_MM,
    .wheel_radius_mm = CHASSIS_WHEEL_RADIUS_MM,
};
```

- **编译选择。** `ONE_BOARD`、`CHASSIS_BOARD`、`GIMBAL_BOARD`，三种 `CHASSIS_DRIVE_*` 和 `WHEEL_IMU_CAPTURE` 会影响多份源码的 `#if`、消息字段或静态缓存是否入固件。继续通过 `EXTRA_DEFS` 传给各编译单元，由小型 `build_config.h` 提供现有默认值和冲突校验。默认单板 + X 型全向轮，采集默认关闭，固件采集只允许单板 + X 型全向轮。
- **可在初始化时决定的选项。** `MC_IMU_ON_GIMBAL`、`MC_IMU_FOLLOWS_PITCH` 通过 `config.c` 求值并暴露字段，由现有的底盘和云台初始化分支消费。底盘板仍须使用本地 IMU。维持现有 `-DMC_*` 覆盖，包括相位提前、杆臂、加速度门控和分配优先级。
- **协议、芯片和布局常量。** 数据包魔数、协议版本、固定字段长度、寄存器地址、数学常数归各自实现。日志队列 `MC_LOG_DEPTH` 是静态数组容量，保留一处内核常量。项目的日志抽样周期属于 `config.c`。配置版本与 frame revision 原值迁移并供日志与采集共用。
- **配置、模块接口和状态。** 车型配置结构体放 `config.h`。`PID_Init_Config_s`、`ChassisMotionConfig` 等通用模块接口类型留在所属模块，`config.h` 引用已有类型。应用复制只读模板后再绑定 HAL 句柄、反馈指针、回调，并传给可能写入初始化结构的 API，例如 `DJIMotorInit`。运行状态由模块自身持有。
- **板级绑定与生成配置。** `config.h` 使用逻辑端口编号，应用在初始化时通过 BSP 接线取得 HAL 句柄。手写任务的 `osThreadDef` 在 `MX_FREERTOS_Init` 函数内定义，可从配置字段取栈大小和优先级，保留 `osThreadCreate` 调用点。CubeMX 生成的外设和 RTOS 配置从 `.ioc` 管理。同步 README 中过时的视觉串口描述。

## 四、执行顺序与每步验收

### 1. 建立当前工作区基线

- [ ] 记录 Git 基线和相交路径。当前仓库位于工作区下一层 `gimbal-fed infantry`，分支 `底盘`，HEAD 为 `e48fcd9`。调研时已有 36 个修改项及 60 个未跟踪项，索引为空。基线取当前工作树，包含 Makefile 已引用的未跟踪底盘运动、IMU 补偿、卡弹、图传和测试文件。
- [ ] 在新输出目录构建默认固件、九组板型/轮型和开启 WIMU 的固件。保存工具链版本、ELF/MAP、`arm-none-eabi-size`、现有告警和测试结果。将旧 build 中的诊断记录和回退固件作为独立资料保护。
- [ ] 从当前代码记录角色初始化顺序、FreeRTOS 参数、CAN/USART 绑定、消息布局和采集日志字段。已知基线失败先单独登记，后续比较变化。

方案调研时已用 Arm GNU GCC 13.3.1 在独立目录构建默认固件、九组板型与轮型、开启 WIMU 的固件，全部通过。普通固件和 WIMU 固件的 `.bss` 分别为 56,600 和 149,124 字节。现有告警包括未使用符号、newlib 系统调用桩和 RWX 段。源码副本中的宿主运动测试通过，Python 测试 19 项通过。后续资源验收应分别对照普通和采集构建，并查看链接 MAP 的 RAM 区域以及台架上的 FreeRTOS 栈高水位。

### 2. 先拆消息和配置，保留算法与接口

- [ ] 将编译选项从 `robot_def.h` 提到 `build_config.h`，将应用消息和板间帧归到 `robot_msg.h`。裁判 UI 状态留待 UI 迁移时同步归位。检查板型条件字段、枚举、`sizeof`、`offsetof` 和代表性帧字节。
- [ ] 先迁底盘几何、运动/KF，再迁力控、云台、发射、输入、IMU、任务、日志。每组改完对比旧值和新字段，核对源文件中的内联 PID、校准、接线等初始化值。模块的配置从原调用点传入。记录遥控死区的边界差异，底盘用开区间 `(-10, 10)`，云台用闭区间 `[-10, 10]`。
- [ ] 迁移 `chassis_motion_project.c` 为 `chassis_setup.c`，维持 profile → 项目参数 → 弱覆盖 → Configure 的顺序。核对 X 型全向轮旧代数 Y 符号、SWERVE 无反馈时的拒绝输出，以及各板角色的 IMU 所有权。
- [ ] 宿主工程接入真正的配置类型。删掉 `tests/host/platform.h` 中屏蔽整个 `robot_def.h` 的绕行和重复机械参数，保留所需桩。只测运动内核的测试继续独立编译。

**验收**　默认车型及参数覆盖与基线一致。三种角色、三种轮型可编译，冲突组合继续被拒绝，测试和应用读取同一套车型标定。旧配置头的引用清零后再移除。

### 3. 按职责迁层和重命名

- [ ] 纯搬迁优先。先移动运动解算与 IMU、再处理其他短命名，逐批修改源列表和路径。保持 `RobotInit` 的中断开关和 BSP 初始化顺序、`RobotTask` 的 Cmd → Gimbal → Shoot → Chassis 调度顺序及各角色的分支。Cmd 读取上一拍 yaw 反馈、云台读取本拍命令的行为保留。
- [ ] 先提取 `chassis_log.c`，再提取 `chassis_force.c`。保持底盘应用中的模式入口、安全许可和启停路径。力控提取时明确测量输入和电流输出，核对模式切换、超时、刹车、积分清理与电流限幅。RTT 格式化、排队和 drain 在现有低优先级任务运行。
- [ ] 拆除 DJI/INS 对应用采集头的引用。将 WIMU 记录和缓冲封装成模块，应用在布防时准备元数据、轮位映射和安全许可。在原 CAN ISR、IMU 采样尝试和控制记录点传递原事件及时间戳。保留临界区、事件顺序、触发、丢包计数和非阻塞 RTT 行为。
- [ ] 将裁判 UI 逻辑与交互状态迁到应用，通用裁判收发和绘图保留模块接口。检查 `RefereeSend`、自定义交互帧及绘图序号的现有语义。
- [ ] 更新 `tests/run_chassis_motion_tests.ps1` 和直接编译源码的 Python 测试路径。`test_imu_attitude_lever.py`、`test_imu_diagnostics.py` 有源码摘录及路径依赖，迁移到新位置或改为编译真实接口，沿用原有断言。

**验收**　模块无需 include 应用目录。底盘运动的宿主可移植测试、单/双板消息、各角色启动、WIMU 开关、日志回放和协议布局检查全部通过。

### 4. 单独清理文件

| 处理 | 文件或目录 | 依据 |
| --- | --- | --- |
| 删除候选 | `STM32F427.svd.txt` | 与 `STM32F427.svd` 内容相同，VS Code 调试引用后者。执行时再扫一遍脚本和文档引用 |
| 删除候选 | `task.ps1` | 内容是 GNU Make 示例配方。确认团队使用现有 Makefile 与 VS Code 任务后清理 |
| 删除候选 | `modules/standard_cmd/std_cmd.[ch]`、`modules/unicomm/unicomm.[ch]` | 当前均为零字节。移除 Makefile 对应条目，设计草稿可移至文档归档 |
| 删除候选 | `bsp/adc/bsp_adc.[ch]`、`modules/motor/step_motor/step_motor.[ch]` | 当前均为零字节，清除空头引用和源列表。ADC 的真实 CubeMX 初始化在 `Core/Src/adc.c` |
| 清除并忽略 | `tests/__pycache__/`、`tools/__pycache__/` | 可再生缓存，先确认路径在 Git 根内且无其他文件 |
| 先比对并归档 | `modules/jam_detector.zip` | 压缩包版本的可疑卡弹超时为 300 ms，现用源码为 150 ms 并新增判定参数 |
| 逐项核验 | OLED、HC05、TFmini、Mahony、未生效的 `VISION_USE_VCP` 等候选 | Makefile、任务调用、角色条件、弱符号、回调注册、测试、调试功能和文档均需核对。TFmini 源码已有工作区改动 |
| 保留 | 卡弹、JScope、图传、裁判、DM/LK 电机、功控和超电 | 存在现行调用。DM/LK 电机控制和功控仍由 `MotorControlTask` 调度 |
| 保留 | `basic_framework_ATYPE.ioc`、`.mxproject`、启动/链接文件、有效 SVD、OpenOCD/JFlash、驱动和第三方代码 | 生成、调试和烧录入口 |
| 保留或移动到文档目录 | 调试记录、实测日志、Python 解码器、README 引用的 `.Doc/.assets` | 实车标定和复现依赖 |

先形成逐项清单，记录候选文件、引用搜索和保留的替代入口。删完用独立输出目录重新构建并跑回归。根目录外层的 `.vscode`、`tmp` 属于工作区资料，单独识别其归属。

## 五、验证与提交

在内层 Git 根运行。ARM GCC 和宿主 GCC 分别用于固件与宿主测试。示例为 PowerShell。

```powershell
$buildRoot = "build/reorg_$(Get-Date -Format yyyyMMdd_HHmmss)"
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
make -j4 "BUILD_DIR=$buildRoot/default"
if ($LASTEXITCODE -ne 0) { throw '默认固件构建失败' }
foreach ($board in 'ONE_BOARD', 'CHASSIS_BOARD', 'GIMBAL_BOARD') {
    foreach ($drive in 'OMNI_X', 'MECANUM', 'SWERVE') {
        make -j4 "BUILD_DIR=$buildRoot/${board}_${drive}" "EXTRA_DEFS=-D$board -DCHASSIS_DRIVE_$drive"
        if ($LASTEXITCODE -ne 0) { throw "变体构建失败：$board / $drive" }
    }
}
make -j4 "BUILD_DIR=$buildRoot/capture" 'EXTRA_DEFS=-DWHEEL_IMU_CAPTURE=1'
if ($LASTEXITCODE -ne 0) { throw 'WIMU 固件构建失败' }
./tests/run_chassis_motion_tests.ps1
python -B -m unittest discover -s tests -p 'test_*.py'
git diff --check
```

基线与迁移后均使用全新或明确隔离的输出目录。现有 `tests/build_chassis_variants.ps1` 需要 `-GccPath` 且复用固定目录。本机工具链路径带空格，当前 Makefile 直接拼接该路径会拆开命令，因此上例从 PATH 调用。实施时给脚本增加可选输出根目录和 PATH 模式。最后两类测试可在源码快照中运行，宿主脚本先向 `build/host_motion` 写入解码样本，Python 测试随后读取。保持现有兼容日志夹具，并比较固件 MAP 中的任务入口、角色关键符号、静态内存和采集关闭时的缓存占用。

| 回归项 | 验收 |
| --- | --- |
| 编译覆盖 | 三种板型 × 三种轮型共九组通过。默认固件和开启 WIMU 的单板 X 型全向轮单独构建，错误组合验证拒绝 |
| 行为一致 | 现有测试覆盖底盘融合、力控权限、退化、CAN 快照、IMU 补偿/SPI、云台、日志解码。核对参数、默认状态、调用顺序及失联/异常时的零输出 |
| 协议兼容 | 板间 CAN、裁判帧、MCL1 和 WIMU 的枚举、字段、布局、版本、日志通道和时间基准与基线一致。当前 RTT 文本仍可由现有工具解析 |
| 台架与实车 | 先限能、禁发射验证启动、IMU、遥控失联、CAN 与电机方向。再依次验证低速平移、小陀螺、云台限位、发射保护和力控切换，沿用当前工程的安全模式 |

按配置/消息、目录与职责迁移、清理分批形成可验证的本地提交。开始前记录原索引、文件状态和相交改动，按路径或片段只暂存本任务内容。若现有未跟踪源码是迁移的直接前置，先明确哪些内容纳入实施基线，再提交可独立构建的改动。每批记录提交号和验证结果，保留已有暂存与其他工作区改动。
