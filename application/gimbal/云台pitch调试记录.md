# 云台 pitch 轴调试记录

> 问题：pitch 轴不跟遥控器摇杆，上电头朝下顶死，电机又硬又烫。
> 结论：**方向反了（正反馈）**，而不是扭矩不足或遥控器/IMU 问题。最终通过 `feedback_reverse_flag` 反转电机转向解决。

---

## 一、问题现象

1. 推左摇杆（竖直方向，`rocker_l1`），pitch 轴电机不动。
2. 上电后云台头朝下（-40° 附近）顶死，用手掰才动。
3. 两种状态下电机都很硬、发烫（满电流堵转）。

---

## 二、根因分析

### 遥控器映射 —— 正常
`rocker_l1` 即 Channel 3（左摇杆竖直），映射链路正确：

- [remote_control.c](modules/remote/remote_control.c) 中 `rocker_l1 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff - RC_CH_VALUE_OFFSET`（Channel 3）
- [robot_cmd.c](application/cmd/robot_cmd.c) 中 `gimbal_cmd_send.pitch += 0.001f * rocker_l1`

数据验证：pitch 命令值确实随摇杆 0 → ±30° 变化，消息链路（cmd pub → gimbal sub）也通。

### IMU 解算 —— 有一个单位问题（已修），轴映射无 bug
- **单位**：四元数 EKF 内部使用 **rad/s**，但 `mpu6500.c` 对外填入 `attitude_t.Gyro[]` 时已经转换成 **°/s**。电机速度环的反馈单位也是 **°/s**，因此云台控制应直接使用 `gimbal_IMU_data->Gyro[]`，不能再次乘 `RAD_2_DEGREE`。
- **轴映射**：EKF 内部 Roll/Pitch 命名是反的（`QEKF.Pitch` 实为绕 X 轴、`QEKF.Roll` 实为绕 Y 轴），但 [mpu6500.c](modules/mpu6500/mpu6500.c) 又交换回来，最终 `INS.Pitch` 与 `INS.Gyro[1]` **都是 Y 轴**，角度和角速度一致，无 bug。

### 方向 —— 根因：正反馈
症状「头朝下 + 很硬 + 发烫」是**正反馈**的经典表现：电机把云台往最低点死压、满电流堵转。GM6020 扭矩大，正常不会抬不动，因此方向反了的可能性最大。

**关键：两个反向标志的区别**（见 [dji_motor.c](modules/motor/DJImotor/dji_motor.c)）：

| 标志 | 作用位置 | 实际效果 |
|---|---|---|
| `motor_reverse_flag` | 参考值（PID 输入）取反 | 目标位置镜像 `Pitch → -命令值`；**命令=0 时取反 0 还是 0，没用** |
| `feedback_reverse_flag` | 最终电流（PID 输出）取反 | **真正反转电机转向** |

之前误改 `motor_reverse_flag` 无效的原因：云台用 IMU 绝对姿态做反馈，命令=0（云台卡住的位置）时取反参考值等于没变。正确的转向反转标志是 `feedback_reverse_flag`。

---

## 三、改动清单

### 1. `application/gimbal/gimbal.c` —— 根因修复

**（1）pitch 电机反转转向**（核心修复）：

```c
.controller_setting_init_config = {
    .angle_feedback_source = OTHER_FEED,
    .speed_feedback_source = OTHER_FEED,
    .outer_loop_type = ANGLE_LOOP,
    .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE, // 反转pitch电机转向,消除正反馈
},
```

**（2）速度环反馈单位换算 rad/s → °/s**：

让 `other_speed_feedback_ptr` 直接指向 INS 对外的角速度反馈：

```c
// INS 对外的 Gyro[] 已经是 °/s:
.other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2], // yaw
.other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[1], // pitch
```

**（3）速度环 PID 参数说明**：

当前代码保留 yaw 的 `Kp=50, Ki=200` 和 pitch 的 `Kp=50, Ki=350`。这次先修正
陀螺仪反馈单位，再观察实际响应；不要在单位未统一前盲目重调 PID。A 当前 pitch
使用 `IntegralLimit=10000, MaxOut=30000`，如果实车出现积分堆积、超调或发热，
可先回到 `(3)` 的 `IntegralLimit=2500, MaxOut=20000`，再逐步增加。

### 2. `application/cmd/robot_cmd.c` —— pitch 软件限位

反馈是 IMU 绝对欧拉角，参考值是摇杆增量累加，若不限位会一直累加、云台顶死机械限位。加 ±30° 软件限位：

```c
// RobotCMDTask() 中:
if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
    gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
else if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
    gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
```

> 限位值按实际机械行程调整。

---

## 四、验证方法

1. 重新编译烧录。
2. 上电 → 右拨杆拨到**下**（GYRO 模式）→ 摇杆居中，云台应**自动回平（0°）**，不再头朝下顶死。
3. 推左摇杆上下，pitch 跟随。

**结果判断**：

- ✅ 云台回平、跟摇杆动 → 方向正确，解决。
- ❌ 云台反向顶到**最高点**、更硬 → 方向本来就对，把 `feedback_reverse_flag` 改回 `FEEDBACK_DIRECTION_NORMAL`，问题转为配重/扭矩。

---

## 五、后续待办

1. **清理调试打印**：`gimbal.c` 的 `[gim] pitch_recv=...` 和 `robot_cmd.c` 的 `[cmd] pitch=...` 两处 SEGGER_RTT 调试打印，正式交付前删除。
2. **yaw 轴检查**：若 yaw 也乱转/顶死限位，用同样方式给 yaw 的 `controller_setting_init_config` 加 `feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE`（同型号同总线，接线方向通常一致）。
3. **重力前馈**：`gimbal.c` 中仍有重力补偿前馈的 TODO。配重调平后加前馈，可让电机不必持续大电流维持，更稳更凉。
4. **配重**：若 30000 电流仍抬不动或长期发热，说明头重接近电机极限，需机械配重调平。
