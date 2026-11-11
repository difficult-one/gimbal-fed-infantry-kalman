"""Compare supported wheel spin with ground spin, without treating wheel speed as ground truth."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from chassis_text_parser import parse
from analyze_imu_diagnostics import decode


def vector(row, tag, key, scale=.001):
    return [int(x) * scale for x in row[tag][key].split(",")]


def norm(values):
    return math.sqrt(sum(x*x for x in values))


def percentile(values, fraction):
    values = sorted(values)
    index = (len(values)-1)*fraction
    low = math.floor(index)
    return values[low] + (values[math.ceil(index)]-values[low])*(index-low)


def analyze(path):
    raw = path.read_bytes()
    text = raw.decode("utf-8-sig")
    rows, _, parsing = parse(text)
    if not rows:
        raise ValueError("No complete frames")
    if any(int(r["STATUS"]["frame"]) not in (16, 17) for r in rows):
        raise ValueError("This comparison requires the known frame16/17 wheel calibration")
    aux = decode(text)
    times = [int(r["STATUS"]["t_us"])*1e-6 for r in rows]
    continuous = [False] + [
        int(b["STATUS"]["seq"])-int(a["STATUS"]["seq"]) == 20
        and abs(tb-ta-.1) < .002
        and a["OBS"]["clock_gen"] == b["OBS"]["clock_gen"]
        for a, b, ta, tb in zip(rows, rows[1:], times, times[1:])]
    fresh = [r["STATUS"]["wheel_mask"] == "15" and r["OBS"]["recovering"] == "0"
             and int(r["STATUS"]["imu_age_us"]) <= 20000 for r in rows]
    pure = [ok and abs(int(r["WHEEL"]["omega_ref_mradps"])-6575) <= 2
            and abs(int(r["WHEEL"]["omega_mradps"])-6575) < 100
            and norm(vector(r, "KF", "user_g_mmps")) < .005
            for r, ok in zip(rows, fresh)]
    stable = []
    start = 0
    for i, ok in enumerate(pure):
        if not ok or not continuous[i]:
            start = i if ok else i+1
        stable.append(ok and times[i]-times[start] >= 1.999)

    def summarize(ids):
        if not ids:
            return {"n": 0}
        selected = [rows[i] for i in ids]
        spans = []
        for i in ids:
            if spans and i == spans[-1][-1]+1 and continuous[i]:
                spans[-1].append(i)
            else:
                spans.append([i])
        windows = [w for w in aux["rigid"] if w["good"] and not w["bad"] and not w["reject"]
                   and any(w["start_t_us"]*1e-6 >= times[s[0]] and
                           w["end_t_us"]*1e-6 <= times[s[-1]] for s in spans)]
        count = sum(w["good"] for w in windows)
        fullrate = {"windows": len(windows), "samples": count, "ids": [w["win"] for w in windows]}
        if count:
            for key in ("before_rms_mmps2", "after_rms_mmps2", "corr_rms_mmps2"):
                fullrate[key] = math.sqrt(sum(sum(x*x for x in w[key])*w["good"]
                                               for w in windows)/count)*.001
        raw_norm = [norm(vector(r, "IMU", "raw_a_mmps2")) for r in selected]
        actual_pairs = []
        for span in spans:
            for a, b in zip(span, span[1:]):
                wrap = lambda x: math.atan2(math.sin(x), math.cos(x))
                dg = (int(rows[b]["IMU"]["psi_gn_mrad"])-int(rows[a]["IMU"]["psi_gn_mrad"]))*.001
                dr = (int(rows[b]["IMU"]["theta_mrad"])-int(rows[a]["IMU"]["theta_mrad"]))*.001
                actual_pairs.append(wrap(dg)-wrap(dr))
        span_time = sum(times[s[-1]]-times[s[0]] for s in spans)
        return {
            "n": len(ids), "spans_s": [[times[s[0]], times[s[-1]]] for s in spans],
            "valid": sum(r["STATUS"]["valid"] == "1" for r in selected),
            "accel_used": sum(r["STATUS"]["accel"] == "1" for r in selected),
            "exits": dict(Counter(r["STATUS"]["exit"] for r in selected)),
            "gates": dict(Counter(r["OBS"]["gate"] for r in selected)),
            "reject_reasons": dict(Counter(r["OBS"]["reject_reason"] for r in selected)),
            "center_rms_mps2": math.sqrt(sum(norm(vector(r, "IMU", "center_a_mmps2"))**2
                                             for r in selected)/len(ids)),
            "center_mean_mps2": [sum(vector(r, "IMU", "center_a_mmps2")[j]
                                        for r in selected)/len(ids) for j in range(2)],
            "raw_norm_p5_p50_p95_mps2": [percentile(raw_norm, p) for p in (.05,.5,.95)],
            "gyro_xy_rms_radps": math.sqrt(sum(norm(vector(r, "IMU", "gyro_mradps")[:2])**2
                                               for r in selected)/len(ids)),
            "estimated_speed_mean_max_mps": [sum(norm(vector(r, "KF", "v_n_mmps")) for r in selected)/len(ids),
                                              max(norm(vector(r, "KF", "v_n_mmps")) for r in selected)],
            "wheel_omega_mean_radps": sum(int(r["WHEEL"]["omega_mradps"])*.001 for r in selected)/len(ids),
            "heading_omega_mean_radps": sum(actual_pairs)/span_time if span_time else None,
            "gravity_update_rejected": sum(bool(int(r["OBS"]["att_flags"]) & 4) for r in selected),
            "fullrate": fullrate,
        }

    spin_ids = [i for i,r in enumerate(rows) if int(r["WHEEL"]["omega_ref_mradps"]) > 1000]
    if not spin_ids:
        raise ValueError("No rotation reference above 1 rad/s to compare")
    first, last = times[spin_ids[0]], times[spin_ids[-1]]
    stopped = [i for i,r in enumerate(rows) if fresh[i] and r["WHEEL"]["omega_ref_mradps"] == "0"
               and norm(vector(r,"WHEEL","vn_mmps")) < .005
               and abs(int(r["WHEEL"]["omega_mradps"])) < 50
               and norm(vector(r,"KF","user_g_mmps")) < .005]
    stop_time = next((times[i] for i in stopped if times[i] > last), None)
    return {
        "source": str(path.resolve()), "sha256": hashlib.sha256(raw).hexdigest(), "parsing": parsing,
        "versions": dict(Counter(f'{r["STATUS"]["fw"]}/{r["STATUS"]["frame"]}/{r["STATUS"]["mode"]}' for r in rows)),
        "gaps": [{"from_seq": rows[i-1]["STATUS"]["seq"], "to_seq": rows[i]["STATUS"]["seq"],
                  "seconds": times[i]-times[i-1]} for i in range(1,len(rows)) if not continuous[i]],
        "tdrop_first_last": [int(rows[i]["STATUS"]["tdrop"]) for i in (0,-1)],
        "qdrop_first_last": [int(rows[i]["STATUS"]["qdrop"]) for i in (0,-1)],
        "clock_generations": sorted({int(r["OBS"]["clock_gen"]) for r in rows}),
        "spin_reference_interval_s": [first,last],
        "all_constant_reference": summarize([i for i, ok in enumerate(pure) if ok]),
        "stable_pure_spin": summarize([i for i, ok in enumerate(stable) if ok]),
        "before_last_10s": summarize([i for i in stopped if first-10 <= times[i] < first]),
        "after_first_5s": summarize([i for i in stopped if stop_time is not None and stop_time <= times[i] < stop_time+5]),
        "tail_10s": summarize([i for i in stopped if times[i] >= times[-1]-10]),
        "shadow_cmd_minus_ref_max_mps": max(abs(a-b) for r in rows for a,b in
                                            zip(vector(r,"KF","cmd_g_mmps"),vector(r,"KF","ref_g_mmps"))),
        "reject_counter_first_last": [int(rows[i]["OBS"]["rejects"]) for i in (0,-1)],
        "aux_counts": {key:len(value) for key,value in aux.items()},
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = {"supported": analyze(args.input)}
    if args.baseline:
        result["ground"] = analyze(args.baseline)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    for name, item in result.items():
        print(name, json.dumps({key:item[key] for key in
              ("gaps","stable_pure_spin","before_last_10s","tail_10s")},ensure_ascii=False))
