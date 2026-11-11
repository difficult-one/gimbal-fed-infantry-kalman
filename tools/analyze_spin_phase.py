"""从文本日志拟合自旋时四轮共同相位；轮速反馈不是独立车体定位真值。"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from chassis_capture_config import motor_mps_per_dps, read_capture_input, resolve_capture_config
from chassis_text_parser import has_speed_spin_command, parse


def vector(row, tag, key):
    return np.array([float(x) for x in row[tag][key].split(",")])


def wheel_radius_m(row):
    """Return the common radius of a registered historical omni profile."""
    return resolve_capture_config([row], None, None)["wheel_radius_m"][0]


def spin_reference(rows, requested=None, config=None):
    if config is None:
        config = resolve_capture_config(rows, None, None)
    return config["spin_ref_mradps"] if requested is None else requested


def select_spin_rows(rows, spin_ref_mradps=None, config=None):
    if config is None:
        config = resolve_capture_config(rows, None, None)
    target = str(spin_reference(rows, spin_ref_mradps, config))
    trim_enabled = any(config["spin_wheel_trim_dps"])
    decimation = config.get("decimation", 20)
    history_steps = max(1, round(500_000 / (config.get("period_us", 5000) * decimation)))
    selected, rates = [], []
    for i, row in enumerate(rows):
        if i == 0:
            continue
        previous = rows[i-1]
        if (row["STATUS"]["exit"] == "0" and
                has_speed_spin_command(row) and has_speed_spin_command(previous) and
                row["WHEEL"]["omega_ref_mradps"] == target and
                previous["WHEEL"]["omega_ref_mradps"] == target and
                int(row["STATUS"]["seq"]) - int(previous["STATUS"]["seq"]) == decimation):
            dt = (int(row["STATUS"]["t_us"]) - int(previous["STATUS"]["t_us"])) * 1e-6
            delta = (int(row["IMU"]["theta_mrad"]) - int(previous["IMU"]["theta_mrad"])) * .001
            if dt > 0:
                rates.append(math.atan2(math.sin(delta), math.cos(delta))/dt)
        if i < history_steps or row["STATUS"]["exit"] != "0" or row["WHEEL"]["omega_ref_mradps"] != target:
            continue
        ref = vector(row, "KF", "ref_g_mmps")
        norm = np.linalg.norm(ref)
        # Firmware applies trim only to a near-zero translation command. Keep
        # those samples outside the translation phase fit even in ACTIVE logs.
        if norm < 80 or (trim_enabled and np.linalg.norm(vector(row, "KF", "cmd_g_mmps")) < 5):
            continue
        history = rows[i-history_steps:i+1]
        if any(not has_speed_spin_command(h) or
               int(h["STATUS"]["seq"]) != int(row["STATUS"]["seq"]) - decimation*(history_steps-j)
               for j, h in enumerate(history)):
            continue
        if any(np.linalg.norm(vector(h, "KF", "ref_g_mmps")) < 80 or
               np.dot(ref, vector(h, "KF", "ref_g_mmps")) /
               (norm*np.linalg.norm(vector(h, "KF", "ref_g_mmps"))) < .99 for h in history):
            continue
        selected.append(row)
    return selected, rates


def analyze(text, spin_ref_mradps=None, metadata=None, raw_sha256=None):
    rows, _, parsing = parse(text)
    config = resolve_capture_config(rows, metadata, raw_sha256, parsing["config_records"])
    target = spin_reference(rows, spin_ref_mradps, config)
    selected, rates = select_spin_rows(rows, target, config)
    if not selected or not rates:
        raise ValueError("没有足够的同方向稳定旋转样本")
    ref = np.array([vector(r, "KF", "ref_g_mmps") for r in selected])
    wheel = np.array([vector(r, "WHEEL", "dps") for r in selected])
    theta = np.array([float(r["IMU"]["theta_mrad"])*.001 for r in selected])
    bx = np.cos(theta)*ref[:, 0] + np.sin(theta)*ref[:, 1]
    by = -np.sin(theta)*ref[:, 0] + np.cos(theta)*ref[:, 1]
    conversion = motor_mps_per_dps(config)
    fits = {}
    for i, name in enumerate(config["wheel_order"]):
        jx, jy, arm_m = config["kinematics"][i]
        rotation_dps = target * .001 * arm_m / conversion[i]
        delta = (jx*bx + jy*by) * .001 / conversion[i]
        quadrature = (-jx*by + jy*bx) * .001 / conversion[i]
        matrix = np.column_stack((delta, quadrature, np.ones(len(delta))))
        response = wheel[:, i] - rotation_dps
        fitted = np.linalg.lstsq(matrix, response, rcond=None)[0]
        residual = response - matrix @ fitted
        fits[name] = {
            "phase_deg": math.degrees(math.atan2(fitted[1], fitted[0])),
            "gain": math.hypot(fitted[0], fitted[1]),
            "bias_dps": float(fitted[2]),
            "residual_rms_dps": float(np.sqrt(np.mean(residual**2))),
        }
    phase = float(np.mean([r["phase_deg"] for r in fits.values()]))
    rate = float(np.mean(rates))
    return {
        "selected_samples": len(selected), "theta_rate_samples": len(rates),
        "wheel_radius_m": sorted(set(config["wheel_radius_m"])),
        "capture_config": config,
        "spin_ref_mradps": target,
        "parsing": parsing,
        "relative_rate_radps": rate, "four_wheel_fits": fits,
        "mean_phase_deg": phase,
        "equivalent_time_shift_ms": math.radians(phase)/rate*1000,
        "assumptions": "适用记录的 OMNI_X 配置，仅拟合 exit=0 的恒定自旋平移，相位相对原始 theta 和 ref",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--metadata", type=Path, help="采集配置 JSON，frame17/18 或无内嵌 CONFIG 时需要")
    parser.add_argument("--spin-ref-mradps", type=int, help="只覆盖配置中的自旋参考筛选值")
    args = parser.parse_args()
    try:
        capture, metadata, raw_sha256 = read_capture_input(args.input, args.metadata)
        report = analyze(capture, args.spin_ref_mradps, metadata, raw_sha256)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    result = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(result + "\n", encoding="utf-8")
    print(result)
