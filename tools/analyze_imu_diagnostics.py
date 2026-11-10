"""分析 fw=20261028/29 通道 0 的 RIGID/AID 文本，不需要二进制窗口。"""
import argparse
import json
import math
import re
from pathlib import Path

# 顺序对应 imu_compensation.h 的 IMUYawAidCause，下标同时就是原因位号。
CAUSES = (
    "未标明的上游无效", "底盘模式不允许", "底盘命令过期或缺失", "底盘有运动命令",
    "车轮离线", "车轮反馈过期", "车轮原始速度超限", "车轮滤波速度超限",
    "底盘静止许可过期", "启动标定未通过", "辅助学习被关闭", "IMU无效或过期",
    "YAW反馈离线或过期", "PITCH反馈离线或过期", "遥控器离线", "输入源不是遥控摇杆",
    "YAW输入不为零", "云台不是陀螺模式", "云台命令过期", "输入或四元数无效",
    "加速度模长偏离重力", "角速度超限", "姿态接近航向奇异区", "学习采样间隔异常",
    "YAW目标变化", "PITCH目标变化", "PITCH编码器单帧变化", "PITCH编码器累计变化",
    "YAW编码器单帧变化", "IMU航向单帧变化", "学习窗内编码器平均速度超限",
)


def reason_names(mask):
    return [name for i, name in enumerate(CAUSES) if mask & (1 << i)]


def decode(text):
    result = {"rigid": [], "aid": [], "clock": []}
    previous = None
    for line_number, line in enumerate(text.splitlines(), 1):
        match = re.search(r"\[(RIGID|AID|CLOCK)\]\s+(.*)", line)
        if not match:
            continue
        kind, body = match.groups()
        try:
            row = {}
            for key, value in re.findall(r"(\w+)=([^\s]+)", body):
                numbers = [int(x, 16) if x.lower().startswith("0x") else int(x)
                           for x in value.split(",")]
                row[key] = numbers if "," in value else numbers[0]
            if row["dver"] != 1:
                raise ValueError("不支持的诊断版本")
            if kind == "CLOCK":
                for key in "t_us hal_ms cyccnt demcr ctrl gen flags last_t_us".split():
                    if not isinstance(row[key], int):
                        raise ValueError(f"{key} 不是完整标量")
                row["flag_names"] = [name for bit, name in enumerate(
                    ("TRCENA关闭", "CYCCNTENA关闭", "DWT与HAL时间不一致")) if row["flags"] & (1 << bit)]
            elif kind == "AID":
                if not isinstance(row["hits"], list) or len(row["hits"]) != len(CAUSES):
                    raise ValueError("hits 长度不完整")
                row["now_names"] = reason_names(row["now"])
                row["last_names"] = reason_names(row["last"])
                # 首帧只有累计值，没有可确定的区间增量；计数回卷按 uint32 处理。
                row["hits_delta"] = None if previous is None else {
                    name: (a - b) & 0xFFFFFFFF
                    for name, a, b in zip(CAUSES, row["hits"], previous["hits"])
                    if a != b
                }
                previous = row
            else:
                if row["n"] != row["good"] + row["bad"]:
                    raise ValueError("采样总数不匹配")
                for key, size in (("dt_us", 2), ("alpha_rms_mradps2", 3),
                                  ("alpha_pk_mradps2", 3), ("tangent_rms_mmps2", 3),
                                  ("tangent_pk_mmps2", 3), ("centrip_pk_mmps2", 3),
                                  ("centrip_rms_mmps2", 3), ("before_rms_mmps2", 2),
                                  ("after_rms_mmps2", 2), ("before_mean_mmps2", 2),
                                  ("after_mean_mmps2", 2), ("corr_rms_mmps2", 2),
                                  ("cross_um2ps4", 2)):
                    if not isinstance(row[key], list) or len(row[key]) != size:
                        raise ValueError(f"{key} 长度不完整")
            result[kind.lower()].append(row)
        except (ValueError, KeyError) as error:
            raise ValueError(f"第 {line_number} 行诊断不完整或非法：{error}") from error
    return result


def describe(result):
    lines = []
    rigid, aid = result["rigid"], result["aid"]
    clocks = result.get("clock", [])
    lines.append(f"有效诊断记录：RIGID {len(rigid)}，AID {len(aid)}")
    if clocks:
        lines.append(f"CLOCK {len(clocks)}条，时钟代次 {clocks[0]['gen']}→{clocks[-1]['gen']}；最近异常：" +
                     ("、".join(clocks[-1]["flag_names"]) or "无"))
    if rigid:
        good = [r for r in rigid if r["good"] and
                all(0 <= x < 2147483647 for key in ("before_rms_mmps2", "after_rms_mmps2") for x in r[key])]
        count = sum(r["good"] for r in good)
        lines.append(f"已记录窗的采样间隔范围：{min(r['dt_us'][0] for r in rigid)}–{max(r['dt_us'][1] for r in rigid)} μs")
        lines.append(f"最后诊断丢弃累计 ddrop={rigid[-1]['ddrop']}；缺失秒窗不能补画为连续数据")
        if count:
            rms = [math.sqrt(sum(sum(x*x for x in r[key])*r["good"] for r in good)/count)/1000
                   for key in ("before_rms_mmps2", "after_rms_mmps2")]
            lines.append(f"已记录有效采样 {count} 个，水平 RMS：{rms[0]:.5f} → {rms[1]:.5f} m/s^2（整数日志量化后重算）")
    if aid:
        lines.append("当前阻断：" + ("、".join(aid[-1]["now_names"]) or "无"))
        lines.append("最近阻断：" + ("、".join(aid[-1]["last_names"]) or "无"))
        if len(aid) > 1:
            totals = {name: (a-b) & 0xFFFFFFFF
                      for name, a, b in zip(CAUSES, aid[-1]["hits"], aid[0]["hits"]) if a != b}
            lines.append("首末诊断间各门槛命中次数（可同时命中，不能相加为暂停帧数）：")
            lines.extend(f"  {name}: {count}" for name, count in sorted(totals.items(), key=lambda x: -x[1]))
            if not totals:
                lines.append("  无新增命中")
        else:
            lines.append("只有一条 AID：累计命中次数不能解释为本段新增次数")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--json", type=Path, help="同时保存完整解析结果和逐条原因增量")
    args = parser.parse_args()
    result = decode(args.input.read_text(encoding="utf-8-sig"))
    print(describe(result))
    if args.json:
        args.json.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
