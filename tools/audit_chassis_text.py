"""按版本严格对齐底盘文本日志，统计融合门控、非有限值和电机零输出。"""
import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
import json
import math
from pathlib import Path
import statistics

from chassis_text_parser import has_speed_spin_command, parse


def _sequence_summary(rows, config_records, rejected_sequences):
    """Count gaps between accepted core frames and wholly absent records."""
    status = [row["STATUS"] for row in rows]
    versions = {(record["fw"].lower(), int(record["frame"])) for record in status}
    versions.update((record["fw"].lower(), int(record["frame"])) for record in config_records)
    cadences = {(int(record["decimation"]), int(record["period_us"]))
                for record in config_records}
    decimation, period_us = (0, 0)
    if len(versions) == 1 and len(cadences) <= 1:
        decimation, period_us = next(iter(cadences), (20, 5000))
    rejected = sorted(seq for seq in rejected_sequences if decimation > 0 and seq % decimation == 0)
    gaps = missing = unchecked = 0
    for before, after in zip(status, status[1:]):
        first_seq, last_seq = int(before["seq"]), int(after["seq"])
        step = last_seq-first_seq
        elapsed = int(after["t_us"])-int(before["t_us"])
        # Allow timing jitter down to half the nominal period. Backward clocks
        # or counters and implausible jumps are ambiguous across sessions.
        if (not 0 < decimation <= 65535 or period_us <= 0
                or not 0 <= first_seq < last_seq <= 0xffffffff
                or first_seq % decimation or last_seq % decimation
                or elapsed <= 0 or step*period_us > 2*elapsed
                or any(int(after[key]) < int(before[key]) for key in ("qdrop", "tdrop"))):
            unchecked += 1
            continue
        if step > decimation:
            gaps += 1
            incomplete = bisect_left(rejected, last_seq)-bisect_right(rejected, first_seq)
            missing += step//decimation-1-incomplete
    return {"sequence_gaps": gaps, "missing_core_frames": missing,
            "sequence_unchecked_transitions": unchecked}


def audit(text):
    rows, rigid, parsing = parse(text)
    if not rows:
        raise ValueError("没有符合对应frame版本的完整日志组")

    def vector(row, tag, key):
        return [int(x) for x in row[tag][key].split(",")]

    def counts(tag, key):
        return dict(Counter(r[tag][key] for r in rows))

    def high_rotation(row):
        target = int(row["WHEEL"]["omega_ref_mradps"])
        return abs(target) > 4000 and has_speed_spin_command(row)

    spin = [r for r in rows if high_rotation(r)]
    steady = [r for r in spin if r["STATUS"]["wheel_mask"] == "15"
              and abs(int(r["WHEEL"]["omega_mradps"])-int(r["WHEEL"]["omega_ref_mradps"])) < 500]
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
    fields = {
        "OBS": "q_1e6 gravity_mmps2 tilt_mrad chi_x1000 innov_x1000 r_x1e6 dv_res_mmps",
        "DETAIL": "q_1e6 gravity_mmps2 innov_x1000 r_x1e6",
        "IMU": "psi_bn_mrad psi_gn_mrad nav_a_mmps2 center_a_mmps2",
        "YAW": "ref_mdeg fb_mdeg out_raw",
        "KF": "p_x1e6",
    }
    for tag, names in fields.items():
        for key in names.split():
            present = [r for r in rows if key in r.get(tag, {})]
            if present:
                nonfinite[f"{tag}.{key}"] = sum(-2147483648 in vector(r, tag, key) for r in present)
    return {
        "complete_frames": len(rows), "incomplete_frames": parsing["incomplete_frames"],
        **_sequence_summary(rows, parsing["config_records"], parsing["rejected_sequences"]),
        "parsing": parsing,
        "first_seq": int(rows[0]["STATUS"]["seq"]), "last_seq": int(rows[-1]["STATUS"]["seq"]),
        "first_t_us": int(rows[0]["STATUS"]["t_us"]), "last_t_us": int(rows[-1]["STATUS"]["t_us"]),
        "status_counts": {key: counts("STATUS", key) for key in ("fw", "frame", "mode", "valid", "exit", "accel", "obs", "wheel_mask")},
        "kp_counts": counts("KF", "kp_x1000"), "dv_counts": counts("KF", "dv_g_mmps"),
        "cmd_minus_ref_max_mmps": max(abs(a-b) for r in rows for a, b in
                                         zip(vector(r, "KF", "cmd_g_mmps"), vector(r, "KF", "ref_g_mmps"))),
        "zero_dt_seq": [int(r["STATUS"]["seq"]) for r in rows if int(r["STATUS"]["dt_us"]) == 0],
        "nonfinite_frame_counts": nonfinite,
        "diagnostic_frames": {tag: sum(tag in r for r in rows) for tag in ("YAW", "DETAIL", "HEALTH", "CONFIG")},
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
