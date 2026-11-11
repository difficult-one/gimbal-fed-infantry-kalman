# JScope RTT

`jscope_rtt.c` 封装 RTT 数据包和 CSV 抽样。应用在初始化时传入通道、缓冲名称和抽样配置。

## RTTDebugJScope2F

`RTTDebugJScope2F` 发送一种固定格式的数据包：

```c
uint32_t time_us;
float value0;
float value1;
```

当前底盘用它发送单个 M3508 的目标速度和反馈速度：

- `value0`：目标速度 `ref`
- `value1`：反馈速度 `fb`
- RTT 通道：默认通道 1
- JScope buffer name：`JScope_t4f4f4`

整车波形通道、轮位和抽样策略集中在 [config.c](../../config/config.c)。
