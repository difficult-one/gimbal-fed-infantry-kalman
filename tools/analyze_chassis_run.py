"""统计落地日志的平移方向、轮速限幅与松杆过渡；轮速不等于真实地速。"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np

from analyze_spin_phase import select_spin_rows, spin_reference, vector
from chassis_capture_config import motor_mps_per_dps, read_capture_input, resolve_capture_config
from chassis_text_parser import parse


def summary(values):
    if not len(values):
        return None
    return {"n": len(values), "mean": float(np.mean(values)),
            "p95": float(np.percentile(values, 95)),
            "min": float(min(values)), "max": float(max(values))}


def wheel_g(row, config=None):
    if config is None:
        config = resolve_capture_config([row], None, None)
    wheel_mps = vector(row, "WHEEL", "dps") * motor_mps_per_dps(config)
    b = np.linalg.lstsq(config["kinematics"], wheel_mps, rcond=None)[0][:2] * 1000
    # theta_mrad 是实际相对角；theta_out 包含控制提前量。
    theta = int(row["IMU"]["theta_mrad"]) * .001
    c, s = math.cos(theta), math.sin(theta)
    return np.array([c*b[0]-s*b[1], s*b[0]+c*b[1]])


def current_g_proxy(row, config=None):
    if config is None:
        config = resolve_capture_config([row], None, None)
    # J^T 投影的平移分量，仅比较方向。原始电流尚未换算为轮力。
    a = vector(row, "WHEEL", "send_i") * config["motor_sign"]
    b = a @ np.asarray(config["kinematics"])[:, :2]
    theta = int(row["IMU"]["theta_mrad"]) * .001
    c, s = math.cos(theta), math.sin(theta)
    return np.array([c*b[0]-s*b[1], s*b[0]+c*b[1]])


def analyze(text, spin_ref_mradps=None, metadata=None, raw_sha256=None):
    rows, _, parsing = parse(text)
    config = resolve_capture_config(rows, metadata, raw_sha256)
    target = spin_reference(rows, spin_ref_mradps, config)
    selected, _ = select_spin_rows(rows, target, config)
    groups = {}
    for row in selected:
        ref = vector(row, "KF", "ref_g_mmps")
        actual = wheel_g(row, config)
        norm = np.linalg.norm(ref)
        cross = (ref[0]*actual[1]-ref[1]*actual[0]) / norm
        angle = math.degrees(math.atan2(cross*norm, np.dot(ref, actual)))
        category = ("x+" if ref[0] > 0 else "x-") if abs(ref[0]) >= abs(ref[1]) else ("y+" if ref[1] > 0 else "y-")
        entry = [angle, cross, np.linalg.norm(actual), norm,
                 np.linalg.norm(vector(row, "KF", "hat_g_mmps")-actual)]
        for group in ("all", category):
            groups.setdefault(group, []).append(entry)
    out = {"parsing": parsing, "directions": {}, "spin_ref_mradps": target,
           "wheel_radius_m": sorted(set(config["wheel_radius_m"])), "capture_config": config}
    for key, values in groups.items():
        a = np.array(values)
        out["directions"][key] = {
            "n": len(values), "signed_angle_mean_deg": float(np.mean(a[:, 0])),
            "abs_angle_mean_deg": float(np.mean(abs(a[:, 0]))),
            "angle_std_deg": float(np.std(a[:, 0])),
            "abs_angle_p95_deg": float(np.percentile(abs(a[:, 0]), 95)),
            "cross_mean_mmps": float(np.mean(a[:, 1])),
            "cross_rms_mmps": float(np.sqrt(np.mean(a[:, 1]**2))),
            "actual_mean_mmps": float(np.mean(a[:, 2])),
            "ref_mean_mmps": float(np.mean(a[:, 3])),
            "hat_wheel_difference_mean_mmps": float(np.mean(a[:, 4])),
        }
    spin = [r for r in rows if r["STATUS"]["exit"] == "0" and r["WHEEL"]["omega_ref_mradps"] == str(target)]
    large = [r for r in spin if np.linalg.norm(vector(r, "KF", "user_g_mmps")) > 800]
    out["large_input"] = {
        "n": len(large),
        **{key: summary([np.linalg.norm(vector(r, "KF", f"{key}_g_mmps")) for r in large])
           for key in ("user", "ref", "hat")},
        "flags": dict(Counter(r["KF"]["flags"] for r in large)),
    }
    if spin:
        out["spin_execute"] = {
            "n": len(spin),
            "current_abs_peak": max(abs(x) for r in spin for x in vector(r, "WHEEL", "send_i")),
            "req_send_mismatch": sum(int(np.any(vector(r, "WHEEL", "req_i") != vector(r, "WHEEL", "send_i")))
                                     for r in spin if "req_i" in r["WHEEL"]),
            "current_sat": dict(Counter(r["KF"]["sat"] for r in spin)),
            "wheel_req_abs_peak_dps": max(abs(x) for r in spin for x in vector(r, "WHEEL", "req_dps")),
            "wheel_actual_abs_peak_dps": max(abs(x) for r in spin for x in vector(r, "WHEEL", "dps")),
            "center_a_component_rms_mmps2": np.sqrt(np.mean([vector(r, "IMU", "center_a_mmps2")**2 for r in spin], axis=0)).tolist(),
        }
        if "DIR" in spin[0]:
            out["lead"] = {
                "rate_valid": dict(Counter(r["DIR"]["rate_valid"] for r in spin)),
                **{key: summary([int(r["DIR"][key])*.001 for r in spin])
                   for key in ("lead_mrad", "rel_omega_mradps")},
            }
    out["gaps"] = [{"from_seq": int(p["STATUS"]["seq"]), "to_seq": int(r["STATUS"]["seq"]),
                    "dt_us": int(r["STATUS"]["t_us"])-int(p["STATUS"]["t_us"])}
                   for p, r in zip(rows, rows[1:]) if int(r["STATUS"]["seq"])-int(p["STATUS"]["seq"]) > 20]
    out["dt_us"] = summary([int(r["STATUS"]["dt_us"]) for r in rows])
    out["tdrop_last"] = int(rows[-1]["STATUS"]["tdrop"])
    out["qdrop_last"] = int(rows[-1]["STATUS"]["qdrop"])
    out["release_windows"] = []
    for i, (previous, row) in enumerate(zip(rows, rows[1:]), 1):
        if (np.linalg.norm(vector(previous, "KF", "user_g_mmps")) <= 20 or
                np.linalg.norm(vector(row, "KF", "user_g_mmps")) > 20 or
                int(row["STATUS"]["seq"])-int(previous["STATUS"]["seq"]) != 20):
            continue
        window = []
        for r in rows[max(0, i-1):i+5]:
            window.append({"seq": int(r["STATUS"]["seq"]), "t_us": int(r["STATUS"]["t_us"]),
                           "omega_ref_mradps": int(r["WHEEL"]["omega_ref_mradps"]),
                           **{key: vector(r, "KF", f"{key}_g_mmps").tolist() for key in ("user", "ref", "hat", "dv", "cmd")},
                           "wheel_g_mmps": wheel_g(r, config).tolist(),
                           "translation_current_proxy_g": current_g_proxy(r, config).tolist(),
                           "send_i": vector(r, "WHEEL", "send_i").tolist()})
        out["release_windows"].append(window)
    return out


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--metadata", type=Path, help="采集配置 JSON，frame17 起必填")
    parser.add_argument("--spin-ref-mradps", type=int, help="只覆盖配置中的自旋参考筛选值")
    args = parser.parse_args()
    try:
        capture, metadata, raw_sha256 = read_capture_input(args.input, args.metadata)
        report = analyze(capture, args.spin_ref_mradps, metadata, raw_sha256)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    result = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(result+"\n", encoding="utf-8")
    print(result)
