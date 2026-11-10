# 轮速与 IMU 观测一致性实施计划

> 执行时逐项完成下列检查，并为每项保留可独立验证的结果。后续由代理执行时，可按任务分派或在同一会话分批执行。

**目标**

在可重放的原始数据上定位轮速与 IMU 的速度增量差异，并用独立实车轨迹判断修正是否改善估计。

**实施路线**

先校正历史回放、修复采样状态和采集 frame18 基线，再用同一原始序列逐项验证对齐、杆臂和去重力。离线证据支持后，才把有效的改动接入固件 SHADOW。三维参考点扩展和主动补偿各自设验收门槛。

**工程环境**

STM32F427、FreeRTOS、MPU-6500、DJI CAN 电机、SEGGER RTT、ARM GCC、主机 GCC、Python/NumPy。基线以当前 Kalman 工程工作区的 frame18 源码为准。建议文档和落地调试记录分别描述待验证方案和历史数据。

[优化建议](../../../application/chassis/轮速与IMU观测一致性优化建议.md) · [frame17 落地记录](../../../application/chassis/调试记录/20261110_第四十次frame17落地纯旋转漂移分析.md)

## 全局约束

- 首轮目标平台是 `ONE_BOARD + CHASSIS_DRIVE_OMNI_X`，其余板型和驱动配置纳入编译及现有主机回归。比较采集时保持 SHADOW，固定轮径、逐轮微调、13.5 ms 控制输出超前、门限和协方差。记录固件产物哈希及实际配置。
- 配对和积分统一使用 SI 单位与同一右手导航系。采集时间、SPI 完成时间、CAN 接收时间和控制执行时间分别记录。外部定位用场地标记与视频或等效设备，并给出同步与测量误差。
- 代码验收用下列命令。构建输出使用新目录，历史原始日志保留原字节。测试修复与功能分开提交，暂存范围只覆盖本任务改动。

~~~powershell
python -B -m unittest discover -s tests -p 'test_*.py'
./tests/run_chassis_motion_tests.ps1
make -j4 BUILD_DIR=build/wheel_imu_obs_check
arm-none-eabi-size ./build/wheel_imu_obs_check/basic_framework_ATYPE.elf
~~~

## 0. 校准基线测试夹具

文件 · `tests/host/platform.h`、`tests/chassis_project_config_test.c`、`application/robot_def.h`

- [ ] 用现有主机测试重现 `chassis_project_config_test.c:33` 的轮径断言。当前夹具传入 72.5 mm，实际项目宏和测试的独立物理期望使用 76 mm。Python 基线已通过，历史轮径用例仍断言 frame17 为 72.5 mm。
- [ ] 将主机项目夹具对齐当前固件配置，保留 19:1、152 mm 外径下 19000 转子 deg/s 应为 1.32645023 m/s 的物理断言。72.5 mm 留在历史回放用例中。
- [ ] 跑完整 C 测试和 3×3 板型/驱动配置矩阵。计划编制时，原主机测试止于轮径断言，临时夹具只改为 76 mm 后 9 个配置用例均通过，完整套件仍需在正式修复后验收。

## 1. P0 · 修正回放配置和速度单位

文件 · `tools/analyze_spin_phase.py`、`tools/analyze_chassis_run.py`、`tools/analyze_frame17_spin_drift.py`、`tests/test_chassis_wheel_calibration.py`

新增接口 · `tools/chassis_capture_config.py` 的 `resolve_capture_config(rows, metadata, raw_sha256)`。两个通用 CLI 支持 `--metadata PATH`。

- [ ] 为新采集定义版本化 JSON，至少包含 `schema_version`、`capture_id`、`fw`、`frame`、`config_id`、原始文件 SHA-256、底盘类型、LF/RF/LB/RB 顺序、每轮配置半径（m）、减速比、编码器符号、运动学矩阵、坐标约定、纯自旋逐轮微调（转子 deg/s）和筛选参考（mrad/s）。负载实测滚动半径单独记录。
- [ ] 先校验文件、版本、配置和有限正参数，再做工况筛选。保留已登记的 frame1–15 / frame16 的 60 / 72.5 mm 历史口径。frame17、frame18 和未登记版本要求显式配置。混合固件或参数冲突应报错。`--spin-ref-mradps` 只覆盖筛选目标。两个脚本共同使用 `v_i = sign_i × rotor_dps_i × π/180 × radius_m_i / ratio_i`，相位拟合用同一配置的几何、旋转基项和适用的逐轮微调。对算法未支持的轮型显式报错。
- [ ] 为 `build/frame17_ground_drift_20261110/` 的原始文件逐份核对 SHA-256 和配置来源，按 0.076 m / 6892 mrad/s 重放这批数据。将专用摘要的 `wheel_odometry_n_velocity_mmps` 更名为 `wheel_odometry_n_velocity_mps`，标记输出 schema 版本，沿用原数值。原 JSON 作为历史格式。通用审计如需取本批稳定区间，使用专用摘要或显式参考值。
- [ ] 测试已知轮速、缺失/冲突配置、非法参数、手动筛选绕过、历史缩放、旧摘要单位和重复输出。用含稳定非零平移的合成日志验证相位拟合，用真实纯自旋日志验证参数选择与专用统计。相同文件与配置应得到同一结果。

## 2. P0 · 传播 SPI 状态并区分采样新鲜度

文件 · `modules/mpu6500/mpu6500driver.[ch]`、`mpu6500_reg.h`、`mpu6500.[ch]`，以及 `tests/test_imu_attitude_lever.py`、`tests/imu_recovery_test.c` 和主机 SPI 桩测试。

- [ ] 为命令和数据两段 SPI 分别检查状态，所有退出路径释放 CS。失败时丢弃整个样本，累计 timeout/error/busy。读取成功后再判定新鲜度，区分新样本与普通轮询未就绪。分别记录尝试序号、已接收的新样本序号和无新数据次数。
- [ ] 核验板上 PB8 的 DRDY 连通性及实际边沿。若使用 DRDY，记录边沿时间和序号，并处理 SPI 跨边沿导致的归属不明。若只能轮询，用读回验证过的 `INT_STATUS.RAW_DATA_RDY_INT` 判断新鲜度，时间明确标为 SPI 完成时间。启用寄存器校验时核对 `WHO_AM_I` 地址 `0x75`（期望 `0x70`）、最终量程、DLPF 和采样分频。`RobotInit` 初始化期间屏蔽中断，SPI 启动超时须使用中断屏蔽下可推进的 DWT 截止时间，或迁移到验证过的中断启用阶段。 [MPU-6500 寄存器手册 Rev. 2.1](https://www.tinytronics.nl/product_files/001439_MPU-6500-Register-Map2%20%281%29.pdf)
- [ ] 初始化和标定仅累计成功且新鲜的样本，保留有界超时与已知标定回退，样本不足时保持观测无效。运行时真实 SPI 错误立即发布无效快照，保留最后接受样本的序号、时间与姿态，清空刚体差分历史，停止推进 EKF 和低通，并避免使用失败读取的温度。普通未就绪跳过更新，持续断流由现有样本年龄和间隙保护处理。复用 `IMURecoveryUpdate`，核对恢复前底盘融合和控制许可仍被禁止。
- [ ] 注入命令段和数据段超时、部分传输、启动全失败、重复读取、内容相同但序号不同的新样本、断流和恢复。检查 CS、接受序号、姿态更新计数、刚体历史、保护原因和恢复时长。

## 3. P0 · 采集可重放的 frame18 基线

文件 · `modules/mpu6500/mpu6500.c`、`modules/motor/DJImotor/dji_motor.[ch]`、`application/gimbal/gimbal.[ch]`、`application/chassis/chassis.c`。新增诊断模块放在 `application/chassis/`，主机解码入口放在 `tools/`。

- [ ] 在原生更新点记录 IMU 原始六轴、标定后的输入、姿态和重力、杆臂前后值、软件低通输出、有效性；记录四轮与 yaw 的原始/滤波反馈及 CAN 序号、接收时间；记录控制实际使用的 yaw、轮速参考和门控理由。记录头包含版本、来源、序号、时间类型和时钟代际。采集配置从 `ChassisMotionGetConfig` 获取实际生效值，覆盖定制配置。导出记录用固定字节序及显式长度，解码端校验版本、记录边界和校验和。
- [ ] 各生产者写入预分配的短时缓冲，写满或到达采集时长即冻结，记录缺帧、溢出及各流覆盖范围。用低优先级非阻塞 RTT 独立通道导出冻结数据，保留当前 10 Hz 文本诊断格式。先用 `.map`、栈余量和任务周期分布确认内存及采样开销。容量至少覆盖完整 0.25 s 窗口、对齐前后余量及连续有效窗口。测试溢出、坏包、时间回绕和导出期间无控制周期超时。
- [ ] 在 SHADOW 下采集静止、固定底盘云台俯仰、直线启停和匀速、正反向落地自旋。每类至少三次，同步记录外部中心位置及航向。采集开始前固定工况、分段边界规则和测量精度，留出至少一轮独立数据验收。先保留原 frame18 构建和低频对照，再用带诊断的 frame18 构建采集逐样本基线。frame17 的 10 Hz 文本与秒级统计仅用来复核历史现象。

## 4. P1 · 用原始序列做对齐与刚体消融

文件 · 新增 `tools/replay_wheel_imu_observations.py` 及对应回放测试。入口接收 `capture --metadata PATH --output PATH`，输出按工况和窗口起止时刻标记的基线及候选残差。对照 `chassis_motion.c`、`imu_compensation.c` 和 `QuaternionEKF.c` 的生产顺序。

- [ ] 先重现 frame18 当前路径的轮速、去重力和残差，并与导出诊断交叉核对。回读 MPU 配置确认 20 Hz 档加速度 19.8 ms、陀螺仪 9.9 ms，修正驱动注释。以启停和有激励的片段辨识实际延迟与频响，把 8.5 ms 软件低通、CAN 接收延迟和轮速 IIR 纳入对照。 [MPU-6500 寄存器手册 Rev. 2.1，第 14–15 页](https://www.tinytronics.nl/product_files/001439_MPU-6500-Register-Map2%20%281%29.pdf#page=14)
- [ ] 在同一有效时间轴对齐四轮、IMU、姿态和 yaw。姿态归一化插值，yaw 按最短角差插值；遇到跨时钟代际、超龄、断流或插值范围不足时判无效。用同一导航系计算 `e = trapz(a_N, t) − (v_wheel,N(t1) − v_wheel,N(t0))`，只比较相同有效窗口，同时报告覆盖率。
- [ ] 按旧路径、只对齐、对齐加杆臂输入及 20 ms 端点求导、重算 EKF 与重力投影的顺序比较。先固定记录姿态隔离时间和杆臂效应，再重跑 EKF 验证去重力反馈。固定底盘俯仰检查传感器到轴交点，独立外部平移检查去重力。留出数据只使用已锁定的参数。
- [ ] 用已知延迟、异步采样、抖动、±π 跨界、无延迟、重复/丢失帧和时钟代际切换验证回放。若留出数据的改善未超过基线重复性与外部测量误差，保留诊断链，返回采集与建模阶段。

## 5. P1 · 按证据接入 SHADOW

文件 · `chassis_motion.[ch]`、`chassis.c`、`application/gimbal/gimbal.[ch]`、`tests/run_chassis_motion_tests.ps1`。新增 `tests/chassis_observation_alignment_test.c`。

- [ ] 用短历史和有界插值生成按时刻配对的观测。单板 yaw 使用真实编码器快照时间及在线状态，采集序号从 DJIMotor 快照传递。实时安全检查继续用当前源状态，配对用有效时间。断流、序号回绕和时钟重建分别覆盖，恢复前清空跨代际历史。
- [ ] 明确 KF 观测时刻与发布时刻。若观测含固定延迟，状态更新在观测时间轴上进行，需要当前状态时按模型投影并测试有界误差。复用离线合成序列和已采原始数据校验 C 端行为。并行记录现行与候选残差，检查过期/缺失时撤销许可，以及 `chassis_motion_delay_test.c` 的控制输出相位回归。
- [ ] 使用固定门限复测后再确定是否采纳候选。按 0.25 s / 0.04 m/s 对照同一工况的 RMS、P95、覆盖率、拒绝分布、恢复耗时、轮速跟踪和外部中心轨迹。SHADOW 每帧核对 `cmd_g == ref_g`。

## 6. P2 · 测量后决定是否增加第二段三维转换

文件 · 现有 `chassis_motion.c`、`chassis_motion_project.c`、`application/robot_def.h`，合成测试沿用 `tests/chassis_motion_rigid3d_test.c`。

- [ ] 测量从轮组运动学中心 O 到 yaw/pitch 交点 P 的 `r_OP`，并用独立底盘测量取得俯仰、横滚和底盘角速度。现有两段转换分别是 IMU 到交点，以及交点到底盘中心的水平偏置。
- [ ] 仅在同步数据证实高度项能解释剩余水平加速度且能改善留出数据时，按 `a_O = a_P − α×r_OP − ω×(ω×r_OP)` 扩展。各量先转换到同一坐标系。测试非零高度下的纯横滚/俯仰、零偏置、平面退化与当前 yaw 修正一致，以及底盘角运动不可用时的降级。
- [ ] 对照外部轨迹和四轮跟踪确认漂移、滑移与参考点误差，记录实测几何、误差和保留平面模型或扩展的依据。

## 7. P3 · 验收门控、恢复和主动许可

文件 · `chassis_motion_config.h`、`chassis_motion.c`、`tests/chassis_observation_guard_test.c`、`tests/chassis_control_permission_test.c`、`tests/chassis_transient_guard_test.c`。

- [ ] 汇总各工况同区间的二维残差 RMS/P95、有效覆盖率、`reject_reason`、`accel_used`、`estimate_valid`、`control_ready`、`control_inhibit`、故障恢复耗时和外部轨迹误差。要求留出数据上的收益可重复，覆盖率、轨迹和故障保护达到基线水平。先验证门控原因，再逐项试验恢复或 `R` 参数。积分门控在 KF 更新前执行。
- [ ] 注入 SPI/CAN 断流、过期、时钟切换、残差突变和打滑。检查撤销许可、恢复样本数、连续稳定窗、补偿斜坡及限制，和禁止时 `cmd_g == ref_g`。现行底盘配置为 20 个新配对样本、1 s 控制稳定期、0.5 s 斜坡。
- [ ] 只有 SHADOW 留出验证、实车外部轨迹和故障保护全部通过，才按工况安排单独的 ACTIVE 试验。保留可直接切回 SHADOW 的固件与配置。任一异常退出失效、跟踪或外部漂移恶化，即切回 SHADOW 并记录触发原因。
