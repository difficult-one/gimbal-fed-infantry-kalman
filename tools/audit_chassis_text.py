"""按版本严格对齐底盘文本日志，统计融合门控、非有限值和电机零输出。"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics

from chassis_text_parser import parse


def audit(text):
    rows, rigid, parsing = parse(text)
    if not rows:
        raise ValueError("没有符合对应frame版本的完整日志组")

    def vector(row, tag, key):
        return [int(x) for x in row[tag][key].split(",")]

    def counts(tag, key):
        return dict(Counter(r[tag][key] for r in rows))

    # 仅以记录到的目标/实测角速度选段，不能替代未记录的遥控运动模式。
    spin = [r for r in rows if abs(int(r["WHEEL"]["omega_ref_mradps"])) > 4000]
    steady = [r for r in spin if int(r["WHEEL"]["omega_ref_mradps"]) == 8162
              and abs(int(r["WHEEL"]["omega_mradps"]) - 8162) < 500]
    tracking = {}
    for i, name in enumerate(("LF", "RF", "LB", "RB")):
        error = [vector(r, "WHEEL", "dps")[i] - vector(r, "WHEEL", "req_dps")[i]
                 for r in steady]
        tracking[name] = {
            "send_zero_all": sum(vector(r, "WHEEL", "send_i")[i] == 0 for r in rows),
            "send_zero_high_rotation": sum(vector(r, "WHEEL", "send_i")[i] == 0 for r in spin),
            "steady_mean_error_dps": statistics.mean(error) if error else None,
            "steady_rmse_dps": math.sqrt(statistics.mean(x*x for x in error)) if error else None,
        }
    nonfinite = {}
    observed = [r for r in rows if "OBS" in r]
    observation = {"frames": len(observed)}
    if observed:
        for key in ("gate", "recovering", "clock_gen", "att_flags"):
            observation[key] = dict(Counter(r["OBS"][key] for r in observed))
        observation["last_rejected_windows"] = int(observed[-1]["OBS"]["rejects"])
        for key in ("q_1e6", "gravity_mmps2", "tilt_mrad", "chi_x1000", "innov_x1000", "r_x1e6", "dv_res_mmps"):
            nonfinite[f"OBS.{key}"] = sum(-2147483648 in vector(r, "OBS", key) for r in observed)
    for tag, key in (("IMU", "psi_bn_mrad"), ("IMU", "psi_gn_mrad"),
                     ("IMU", "nav_a_mmps2"), ("IMU", "center_a_mmps2"),
                     ("YAW", "ref_mdeg"), ("YAW", "fb_mdeg"), ("YAW", "out_raw")):
        nonfinite[f"{tag}.{key}"] = sum(-2147483648 in vector(r, tag, key) for r in rows)
    return {
        "complete_frames": len(rows), "incomplete_frames": parsing["incomplete_frames"],
        "parsing": parsing,
        "first_seq": int(rows[0]["STATUS"]["seq"]), "last_seq": int(rows[-1]["STATUS"]["seq"]),
        "first_t_us": int(rows[0]["STATUS"]["t_us"]), "last_t_us": int(rows[-1]["STATUS"]["t_us"]),
        "status_counts": {key: counts("STATUS", key) for key in ("fw", "frame", "mode", "valid", "exit", "accel", "obs", "wheel_mask")},
        "kp_counts": counts("KF", "kp_x1000"), "dv_counts": counts("KF", "dv_g_mmps"),
        "cmd_minus_ref_max_mmps": max(abs(a-b) for r in rows for a, b in
                                         zip(vector(r, "KF", "cmd_g_mmps"), vector(r, "KF", "ref_g_mmps"))),
        "zero_dt_seq": [int(r["STATUS"]["seq"]) for r in rows if int(r["STATUS"]["dt_us"]) == 0],
        "nonfinite_frame_counts": nonfinite,
        "observation": observation,
        "kf_velocity_counts": counts("KF", "v_n_mmps"),
        "wheel_nav_velocity_counts": counts("WHEEL", "vn_mmps"),
        "high_rotation_frames": len(spin), "steady_rotation_frames": len(steady),
        "wheel_tracking": tracking, "rigid_windows": len(rigid),
        "rigid_zero_good_windows": sum(int(r["good"]) == 0 for r in rigid),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = json.dumps(audit(args.input.read_text(encoding="utf-8-sig")), ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(result + "\n", encoding="utf-8")
    print(result)
