"""严格对齐文本日志；乱码、混接、重复和缺字段的 seq 整组剔除。"""
from collections import Counter
import re


TAG = re.compile(r"\[(STATUS|WHEEL|IMU|KF|YAW|DIR|OBS|RIGID)\]\s+")
FIELD = re.compile(r"([A-Za-z_]\w*)=([+-]?\d+(?:,[+-]?\d+)*)\Z", re.ASCII)
FIRMWARE_FIELD = re.compile(r"(fw)=([0-9a-fA-F]{8})\Z")
SCHEMAS = {
    "STATUS": "seq t_us fw frame mode valid exit accel wheel_mask obs dt_us imu_age_us cmd_age_us qdrop tdrop",
    "WHEEL": "seq dps:4 req_dps:4 send_i:4 fb_i:4 vn_mmps:2 omega_mradps omega_ref_mradps",
    "IMU": "seq corr theta_mrad psi_bn_mrad psi_gn_mrad gyro_mradps:3 raw_a_mmps2:3 nav_a_mmps2:3 center_a_mmps2:2",
    "KF": "seq v_n_mmps:2 a_n_mmps2:2 user_g_mmps:2 ref_g_mmps:2 hat_g_mmps:2 err_g_mmps:2 dv_g_mmps:2 cmd_g_mmps:2 kp_x1000 flags sat",
    "YAW": "seq ref_mdeg fb_mdeg out_raw",
    "DIR": "seq theta_out_mrad rel_omega_mradps lead_mrad rate_valid",
    "OBS": "seq imu_t_us clock_gen recovering good_ms q_1e6:4 gravity_mmps2:3 tilt_mrad:2 chi_x1000 att_flags innov_x1000:4 r_x1e6:4 dv_res_mmps:2 window_ms gate rejects",
}
REQUIRED = {"STATUS", "WHEEL", "IMU", "KF", "YAW"}


def parse(text):
    frames, invalid, rigid, issues = {}, set(), [], []
    tag_counts = Counter()
    for number, line in enumerate(text.splitlines(), 1):
        matches = list(TAG.finditer(line))
        if not matches:
            continue
        if len(matches) > 1:
            # 后半行即使完整，也不与其他记录拼成一组。
            for match in matches:
                seq = re.match(r"seq=(\d+)\b", line[match.end():])
                if seq:
                    key = int(seq[1])
                    invalid.add(key)
                    frames.setdefault(key, {})
            issues.append({"line": number, "reason": "merged_tags"})
            continue
        match = matches[0]
        tag, body = match[1], line[match.end():]
        tag_counts[tag] += 1
        if tag == "RIGID":
            fields = dict(re.findall(r"(\w+)=(\S+)", body))
            if fields.get("good", "").isdigit():
                rigid.append(fields)
            continue
        seq = re.match(r"seq=(\d+)\b", body)
        if not seq:
            issues.append({"line": number, "reason": "missing_seq", "tag": tag})
            continue
        key = int(seq[1])
        row = frames.setdefault(key, {})
        fields, reason = {}, None
        for token in body.split():
            field = FIELD.fullmatch(token)
            if field is None and tag == "STATUS":
                field = FIRMWARE_FIELD.fullmatch(token)
            if not field or field[1] in fields:
                reason = "invalid_or_duplicate_field"
                break
            fields[field[1]] = field[2]
        for specification in SCHEMAS[tag].split():
            name, _, size = specification.partition(":")
            if name not in fields or len(fields[name].split(",")) != int(size or 1):
                reason = reason or "missing_field_or_wrong_dimension"
        if tag == "WHEEL" and "req_i" in fields and len(fields["req_i"].split(",")) != 4:
            reason = "wrong_req_i_dimension"
        if tag in row:
            reason = "duplicate_tag"
        if reason:
            invalid.add(key)
            issues.append({"line": number, "seq": key, "tag": tag, "reason": reason})
        else:
            row[tag] = fields
    rows = []
    for key, row in frames.items():
        required = REQUIRED | ({"DIR"} if int(row.get("STATUS", {}).get("frame", 0)) >= 12 else set())
        if int(row.get("STATUS", {}).get("frame", 0)) >= 14:
            required |= {"OBS"}
        if int(row.get("STATUS", {}).get("frame", 0)) >= 15:
            # Validate after gathering tags, including OBS before STATUS.
            observation = row.get("OBS", {})
            for specification in "dv_now_mmps:2 v_res_mmps:2 reject_reason".split():
                name, _, size = specification.partition(":")
                if name not in observation or len(observation[name].split(",")) != int(size or 1):
                    invalid.add(key)
                    issues.append({"seq": key, "tag": "OBS", "reason": "frame15_guard_diagnostic_missing_or_wrong_dimension"})
                    break
        if key not in invalid and required <= row.keys():
            if int(row["STATUS"].get("frame", 0)) >= 17:
                control_fields = ("control_ready control_inhibit control_good_ms control_blend_x1000 "
                                  "control_v_err_mmps control_a_res_mmps2").split()
                if any(name not in row["OBS"] or "," in row["OBS"][name] for name in control_fields):
                    issues.append({"seq": key, "tag": "OBS", "reason": "frame17_control_diagnostic_missing_or_wrong_dimension"})
                    continue
            rows.append(row)
    accepted = {int(r["STATUS"]["seq"]) for r in rows}
    return rows, rigid, {
        "observed_sequences": len(frames), "complete_frames": len(rows),
        "incomplete_frames": len(frames) - len(rows),
        "rejected_sequences": [seq for seq in frames if seq not in accepted],
        "issues": issues, "tag_counts": dict(tag_counts),
    }
