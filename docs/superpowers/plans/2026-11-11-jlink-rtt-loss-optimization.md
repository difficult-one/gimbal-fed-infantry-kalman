# J-Link RTT 丢包优化方案

**目标**是在实际采集链路上确认丢包发生的位置，让通道 0 能连续保存约 10 Hz 的底盘核心数据，并保留约 1 Hz 的配置与辅助诊断。先验证读取速率，再根据计数器决定是否需要改固件。

**基线**使用本地提交 `d5821ac` 生成的单板 X 型四全向轮固件，日志标识为 `fw=20261111 frame=19 mode=2`。`frame=19` 标识文本格式，修复前后的两版都使用这个值，所以每轮还需记录固件文件及其 SHA-256。测试使用现有卡尔曼影子模式和底盘控制配置。

## 当前证据

| 发现 | 证据及边界 |
| --- | --- |
| 辅助包曾挤占积压的核心帧 | 共享缓冲只剩 2500 字节时，旧顺序会丢掉第二帧。现已改为排完核心队列再发送辅助包，有限缓冲回归通过。 |
| 审计曾漏数整帧缺失 | 现有 `missing_core_frames` 统计完整序号之间整组未出现的帧，`incomplete_frames` 单独统计收到但不完整的帧。 |
| 历史缺口出现在 RTT 写出阶段 | 四份旧实车日志的 `qdrop` 未增长，`tdrop` 增量与整帧缺失数分别同为 44、1868、553、2272。新版尚无实车采集，当前链路仍需复测。 |
| 100 kHz 的调试配置可能限制持续读取 | 内外两份 `Debug-Jlink` 都设置了 `-speed 100`。该参数单位是 kHz。100 kHz 的原始位速率折合最多 12.5 kB/s，实际读取还要扣除 SWD 开销。代表性主机快照估算为 12.18 kB/s，尚未计入独立的 RIGID/AID/CLOCK。是否使用此速率以采集会话实际配置为准。 |

[固件日志发送](../../../application/chassis/chassis_log.c) · [审计工具](../../../tools/audit_chassis_text.py) · [项目调试配置](../../../.vscode/launch.json) · [SEGGER 速率说明](https://kb.segger.com/J-Link_GDB_Server#-speed)

## P0 · 固定烧录文件和采集入口

- [ ] 选择 `build/rtt-frame19/basic_framework_ATYPE.hex` 烧录。默认 `download_jlink` 会直接使用所选构建目录里的 HEX，且默认路径下曾留有较旧产物。若用 VS Code 启动调试，确认它加载的是同一构建的 ELF，避免启动时换回旧固件。
- [ ] 在工程根目录记录提交号和固件指纹。

~~~powershell
git rev-parse --short HEAD
Get-FileHash -Algorithm SHA256 -LiteralPath '.\build\rtt-frame19\basic_framework_ATYPE.hex'
~~~

- [ ] 选定一种采集入口贯穿对照试验。用 RTT Viewer 直连时，在连接参数里选 SWD 速率。用 `Debug-Jlink` 配合 RTT Client 时，在实际生效的调试配置里设置 `-speed`，并从 GDB Server 日志确认实际速率。两档都关闭 Live Watch，使用一个客户端读取通道 0 并直接保存原始文本。
- [ ] 每次上电单独保存文件。核对日志头 `fw=20261111 frame=19 mode=2`，并确认能收到 `CONFIG`。文件名包含速率、工况和轮次，例如 `100k_static_01.txt`。

[烧录目标](../../../Makefile) · [SEGGER RTT Viewer 连接与文件记录](https://kb.segger.com/J-Link_RTT_Viewer)

## P1 · 做单变量速率对照

- [ ] 固定同一固件、J-Link、线缆、供电和采集方式。分别用 **100 kHz** 和 **1 MHz** 采集，只有连接稳定才把 1 MHz 作为候选。每档先在启动和读取稳定后录 30–60 秒静止数据，再用相同的低速前后移动和松杆动作录一段。每档至少重复两次，记录实际速率和有效观察时间。
- [ ] 保留完整原始文件。若启动段已有丢包，将稳定观察段另存为分析副本，保留完整的日志组和 `CONFIG`，并在记录中标注截取范围。
- [ ] 审计每个观察段。例如将 `100k_static_01.txt` 保存到 `build/rtt-validation/` 后，在工程根目录运行：

~~~powershell
python -B .\tools\audit_chassis_text.py '.\build\rtt-validation\100k_static_01.txt' --output '.\build\rtt-validation\100k_static_01.audit.json'
~~~

| 每段记录的量 | 通过标准 |
| --- | --- |
| `missing_core_frames`、`incomplete_frames`、`sequence_unchecked_transitions` | 均为 0。默认 `seq` 每组增加 20，按采集到的 `CONFIG.decimation` 复核抽样间隔。 |
| `qdrop` 与 `tdrop` | 从有效段首尾完整的 `[STATUS]` 读取，增量均为 0。若只有 `tdrop` 增长且核心帧完整，进一步检查辅助记录。 |
| `CONFIG`、`DETAIL`、`HEALTH` | 稳定区间持续收到，周期约 1 秒。需要秒级 RIGID/AID/CLOCK 诊断时，另核对 `ddrop` 增量。 |
| 采集时长与核心组数 | 采样率与配置相符，默认约 10 组/秒。若中途断连或复位，该段分开记录。 |

`sequence_gaps` 是完整帧之间出现序号跳变的次数，`missing_core_frames` 是其中未收到的整帧数，两项不相加。序号统计需要前后两帧作为边界，不能计算文件结束后未保存的数据。

## P2 · 按复测结果处理

| 复测结果 | 下一步 |
| --- | --- |
| 提高速率后，静止和运动重复采集均达标 | 将验证过的速率写入实际使用的采集配置，并记录调试器、接线和固件版本。 |
| `qdrop` 增长 | 测量日志任务耗时和快照队列高水位，先找出日志任务未及时取走快照的时段，再决定是否减少格式化负担。 |
| `tdrop` 增长且缺核心帧，`qdrop` 稳定 | 先核对实际 SWD 速率和主机连续读取情况。读速率充足但仍出现短时拥塞时，再评估将通道 0 缓冲由 4096 提到 8192 字节，并对同一工况比较缺帧数。 |
| `tdrop` 增长但核心帧完整 | 对照低频记录的间隔。若需要区分两类失败，可给核心包与辅助包增加独立丢包计数，再复测。 |
| 缺帧但 `qdrop`、`tdrop` 都稳定，或残缺帧增多 | 检查上位机保存方式、调试连接是否中断、是否把不同上电会话拼入同一文件。保留原始文件用于对照。 |

扩大缓冲用于吸收短时积压，持续读速率低于写入速率时，积压仍会增长。涉及固件改动时继续使用非阻塞整包发送。若调整缓冲，默认和 `WHEEL_IMU_CAPTURE=1` 两个版本都要链接通过，核对 SRAM 占用后再重复 P1。

## 软件验收与归档

- [x] 核心帧优先和整帧缺失统计已提交至 `d5821ac`，32 项 Python 测试与默认 ARM 构建通过。
- [ ] 若 P2 引入代码修改，先运行主机测试生成二进制样本，再执行 Python 测试。改变 RTT 缓冲时，在独立输出目录构建普通版和高频版。

~~~powershell
pwsh -NoProfile -File .\tests\run_chassis_motion_tests.ps1
python -B -m unittest discover -s tests -p 'test_*.py'
New-Item -ItemType Directory -Path 'build/rtt-verify', 'build/rtt-verify-wimu' -Force | Out-Null
make -j4 'GCC_PATH=' 'BUILD_DIR=build/rtt-verify'
make -j4 'GCC_PATH=' 'BUILD_DIR=build/rtt-verify-wimu' 'EXTRA_DEFS=-DWHEEL_IMU_CAPTURE=1'
arm-none-eabi-size .\build\rtt-verify\basic_framework_ATYPE.elf .\build\rtt-verify-wimu\basic_framework_ATYPE.elf
~~~

- [ ] 归档每轮固件哈希、实际 SWD 速率、工况、原始文本和审计 JSON。以重复实车采集满足 P1 表格为结论。代码测试和编译结果用于验证修改，实车采集用于验证链路。
