"""Audit frame17 ground-spin logs without treating encoder odometry as position truth."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from chassis_text_parser import parse
from analyze_imu_diagnostics import decode


def vector(row, tag, key, scale=1.0):
    return [int(value) * scale for value in row[tag][key].split(",")]


def rms(values):
    return math.sqrt(sum(value*value for value in values)/len(values)) if values else None


def percentile(values, ratio):
    ordered = sorted(values)
    if not ordered:
        return None
    x = (len(ordered)-1)*ratio
    lo = math.floor(x)
    return ordered[lo] + (ordered[math.ceil(x)]-ordered[lo])*(x-lo)


def direction(row):
    x, y = vector(row, "KF", "user_g_mmps")
    return ("x" if abs(x) >= abs(y) else "y") + ("+" if (x if abs(x) >= abs(y) else y) >= 0 else "-")


def continuous(rows, i):
    if i <= 0:
        return False
    a, b = rows[i-1], rows[i]
    return (int(b["STATUS"]["seq"])-int(a["STATUS"]["seq"]) == 20 and
            abs(int(b["STATUS"]["t_us"])-int(a["STATUS"]["t_us"])-100000) < 2000 and
            a["OBS"]["clock_gen"] == b["OBS"]["clock_gen"])


def summarize(rows, rigid, ids, times):
    selected = [rows[i] for i in ids]
    accel = [vector(r, "IMU", "center_a_mmps2", .001) for r in selected]
    raw = [vector(r, "IMU", "raw_a_mmps2", .001) for r in selected]
    gyro = [vector(r, "IMU", "gyro_mradps", .001) for r in selected]
    wheel_v = [vector(r, "WHEEL", "vn_mmps", .001) for r in selected]
    enc = [vector(r, "WHEEL", "dps") for r in selected]
    currents = [vector(r, "WHEEL", "send_i") for r in selected]
    gaps = 0
    displacement = [0.0, 0.0]
    absolute_displacement = [0.0, 0.0]
    for a, b in zip(ids, ids[1:]):
        if b != a+1 or not continuous(rows, b):
            gaps += 1
            continue
        dt = times[b]-times[a]
        for axis in range(2):
            ds = (wheel_v[ids.index(a)][axis]+wheel_v[ids.index(b)][axis])*.5*dt
            displacement[axis] += ds
            absolute_displacement[axis] += abs(ds)

    segments = []
    for i in ids:
        if segments and i == segments[-1][-1]+1 and continuous(rows, i):
            segments[-1].append(i)
        else:
            segments.append([i])
    windows = [w for w in rigid if w["good"] > 0 and w["bad"] == 0 and w["reject"] == 0 and
               any(w["start_t_us"] >= int(rows[seg[0]]["STATUS"]["t_us"]) and
                   w["end_t_us"] <= int(rows[seg[-1]]["STATUS"]["t_us"]) for seg in segments)]
    good_samples = sum(w["good"] for w in windows)
    fullrate = {"complete_windows": len(windows), "good_samples": good_samples}
    if good_samples:
        for field in ("before_rms_mmps2", "after_rms_mmps2", "corr_rms_mmps2"):
            fullrate[field.removesuffix("_mmps2")] = math.sqrt(
                sum(sum(x*x for x in w[field])*w["good"] for w in windows)/good_samples)*.001

    wheel_rates_mean = [sum(v[a] for v in enc)/len(enc) for a in range(4)]
    current_mean = [sum(v[a] for v in currents)/len(currents) for a in range(4)]
    center_speed = [math.hypot(*v) for v in wheel_v]
    raw_magnitude = [math.sqrt(sum(x*x for x in v)) for v in raw]
    heading_rate = []
    for a, b in zip(ids, ids[1:]):
        if b != a+1 or not continuous(rows, b):
            continue
        dpsi = math.atan2(math.sin((int(rows[b]["IMU"]["psi_gn_mrad"])-int(rows[a]["IMU"]["psi_gn_mrad"]))*0.001),
                          math.cos((int(rows[b]["IMU"]["psi_gn_mrad"])-int(rows[a]["IMU"]["psi_gn_mrad"]))*0.001))
        dtheta = math.atan2(math.sin((int(rows[b]["IMU"]["theta_mrad"])-int(rows[a]["IMU"]["theta_mrad"]))*0.001),
                            math.cos((int(rows[b]["IMU"]["theta_mrad"])-int(rows[a]["IMU"]["theta_mrad"]))*0.001))
        heading_rate.append((dpsi-dtheta)/(times[b]-times[a]))

    return {
        "frames": len(ids), "time_s": [times[ids[0]], times[ids[-1]]],
        "duration_s": times[ids[-1]]-times[ids[0]], "unobserved_pairs": gaps,
        "status_valid": dict(Counter(r["STATUS"]["valid"] for r in selected)),
        "gate": dict(Counter(r["OBS"]["gate"] for r in selected)),
        "accel_used": dict(Counter(r["STATUS"]["accel"] for r in selected)),
        "control_ready": dict(Counter(r["OBS"]["control_ready"] for r in selected)),
        "control_inhibit": dict(Counter(r["OBS"]["control_inhibit"] for r in selected)),
        "control_velocity_error_mmps": {
            "mean": sum(int(r["OBS"]["control_v_err_mmps"]) for r in selected)/len(selected),
            "p95": percentile([int(r["OBS"]["control_v_err_mmps"]) for r in selected], .95),
            "max": max(int(r["OBS"]["control_v_err_mmps"]) for r in selected)},
        "control_accel_residual_mmps2": {
            "mean": sum(int(r["OBS"]["control_a_res_mmps2"]) for r in selected)/len(selected),
            "max": max(int(r["OBS"]["control_a_res_mmps2"]) for r in selected)},
        "wheel_omega_mradps": {
            "mean": sum(int(r["WHEEL"]["omega_mradps"]) for r in selected)/len(selected),
            "reference_values": dict(Counter(r["WHEEL"]["omega_ref_mradps"] for r in selected))},
        "heading_encoder_omega_radps": {"mean": sum(heading_rate)/len(heading_rate), "pairs": len(heading_rate)},
        "wheel_odometry_n_velocity_mps": {
            axis: {"mean": sum(v[k] for v in wheel_v)/len(wheel_v), "rms": rms([v[k] for v in wheel_v])}
            for k, axis in enumerate(("x", "y"))},
        "wheel_odometry_integrated_candidate_m": displacement,
        "wheel_odometry_absolute_path_candidate_m": absolute_displacement,
        "wheel_encoder_no_slip_candidate_is_not_position_truth": True,
        "motor_speed_dps_mean_lf_rf_lb_rb": wheel_rates_mean,
        "motor_speed_dps_delta_from_lf": [x-wheel_rates_mean[0] for x in wheel_rates_mean],
        "motor_current_feedback_raw_mean_lf_rf_lb_rb": [
            sum(vector(r,"WHEEL","fb_i")[k] for r in selected)/len(selected) for k in range(4)],
        "motor_current_command_raw_mean_lf_rf_lb_rb": current_mean,
        "request_send_mismatch_rows": sum(vector(r,"WHEEL","req_i") != vector(r,"WHEEL","send_i") for r in selected),
        "current_command_abs_peak": max(abs(x) for row in currents for x in row),
        "center_accel_rms_mps2": [rms([a[k] for a in accel]) for k in range(2)],
        "raw_accel_magnitude_p05_p50_p95_mps2": [percentile(raw_magnitude, x) for x in (.05,.5,.95)],
        "gyro_xy_rms_radps": [rms([g[k] for g in gyro]) for k in range(2)],
        "rigid_full_rate": fullrate,
    }


def analyze(path, role):
    raw = path.read_bytes()
    text = raw.decode("utf-8-sig")
    rows, _, parsing = parse(text)
    aux = decode(text)
    rigid = aux["rigid"]
    if not rows:
        raise ValueError(f"No complete frames in {path}")
    if any(int(r["STATUS"]["frame"]) not in (17, 18) or r["STATUS"]["fw"] != "20261110" for r in rows):
        raise ValueError(f"Expected fw=20261110 frame=17 or 18 in {path}")
    times = [int(r["STATUS"]["t_us"])*1e-6 for r in rows]
    gaps = [{"from_seq": rows[i-1]["STATUS"]["seq"], "to_seq": r["STATUS"]["seq"],
             "seconds": times[i]-times[i-1]} for i,r in enumerate(rows[1:],1)
            if int(r["STATUS"]["seq"])-int(rows[i-1]["STATUS"]["seq"]) != 20]
    result = {"role": role, "path": str(path.resolve()), "sha256": hashlib.sha256(raw).hexdigest(),
              "parsing": parsing, "frames": len(rows),
              "aux_records": {key.upper(): len(value) for key,value in aux.items()},
              "time_s": [times[0], times[-1]], "gaps": gaps,
              "tdrop_first_last": [int(rows[0]["STATUS"]["tdrop"]), int(rows[-1]["STATUS"]["tdrop"])],
              "qdrop_first_last": [int(rows[0]["STATUS"]["qdrop"]), int(rows[-1]["STATUS"]["qdrop"])],
              "status_valid": dict(Counter(r["STATUS"]["valid"] for r in rows)),
              "estimate_gate": dict(Counter(r["OBS"]["gate"] for r in rows)),
              "control_ready": dict(Counter(r["OBS"]["control_ready"] for r in rows)),
              "control_inhibit": dict(Counter(r["OBS"]["control_inhibit"] for r in rows)),
              "shadow_max_abs_cmd_minus_ref_mmps": max(abs(a-b) for r in rows for a,b in
                   zip(vector(r,"KF","cmd_g_mmps"),vector(r,"KF","ref_g_mmps")))}
    rotation = [i for i,r in enumerate(rows) if int(r["WHEEL"]["omega_ref_mradps"]) > 4000 and
                r["STATUS"]["wheel_mask"] == "15" and int(r["STATUS"]["imu_age_us"]) <= 20000 and
                r["OBS"]["recovering"] == "0" and math.hypot(*vector(r,"KF","user_g_mmps")) < 5]
    if rotation:
        target = Counter(rows[i]["WHEEL"]["omega_ref_mradps"] for i in rotation).most_common(1)[0][0]
        plateau = [i for i in rotation if rows[i]["WHEEL"]["omega_ref_mradps"] == target and
                   abs(int(rows[i]["WHEEL"]["omega_mradps"])-int(target)) < 150]
        result["rotation_reference_target_mradps"] = int(target)
        result["rotation_plateau"] = summarize(rows, rigid, plateau, times) if plateau else {"frames":0}
    else:
        result["rotation_reference_target_mradps"] = None
        result["rotation_plateau"] = {"frames":0}
    if role == "basic movement":
        result["movement"] = {}
        thresholds = (50,100,300)
        for threshold in thresholds:
            ids=[i for i,r in enumerate(rows) if math.hypot(*vector(r,"KF","user_g_mmps"))>=threshold]
            if ids:
                sel=[rows[i] for i in ids]
                result["movement"][str(threshold)+"_mmps"]={
                    "frames":len(ids),"time_s":[times[ids[0]],times[ids[-1]]],
                    "command_direction":dict(Counter(direction(r) for r in sel)),
                    "valid":dict(Counter(r["STATUS"]["valid"] for r in sel)),
                    "control_ready":dict(Counter(r["OBS"]["control_ready"] for r in sel)),
                    "max_current_command_raw":max(abs(x) for r in sel for x in vector(r,"WHEEL","send_i")),
                    "max_wheel_reference_dps":max(abs(x) for r in sel for x in vector(r,"WHEEL","req_dps"))}
    return result


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--basic",type=Path,required=True)
    parser.add_argument("--spin-a",type=Path,required=True)
    parser.add_argument("--spin-b",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    result={"schema_version": 2, "runs":[analyze(args.basic,"basic movement"),analyze(args.spin_a,"ground spin 1"),analyze(args.spin_b,"ground spin 2")],
            "notes":{"frame17_default_mode":"MC_SHADOW","wheel_odometry":"no-slip kinematic candidate only; not ground position truth",
                     "current_command":"DJI raw command, not calibrated wheel force/torque",
                     "physical_drift":"requires independent floor mark/video measurement"}}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps([{ "role":r["role"],"frames":r["frames"],"time_s":r["time_s"],"gaps":r["gaps"],
       "rotation_plateau":r["rotation_plateau"],"movement":r.get("movement")} for r in result["runs"]],ensure_ascii=False,indent=2))
