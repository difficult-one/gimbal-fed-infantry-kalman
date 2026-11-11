"""严格对齐文本日志；乱码、混接、重复和缺字段的 seq 整组剔除。"""
from collections import Counter
import re


TAG = re.compile(r"\[(STATUS|WHEEL|IMU|KF|YAW|DIR|OBS|DETAIL|HEALTH|CONFIG|RIGID)\]\s+")
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
    "DETAIL": "seq q_1e6:4 gravity_mmps2:3 innov_x1000:4 r_x1e6:4",
    "HEALTH": "seq t_us read_t_us attempt_seq accepted_seq status startup timeout error busy not_ready gaps",
    "CONFIG": "seq fw frame period_us decimation direct rotation_priority imu_on_gimbal imu_follows_pitch drive radius_um:4 gear_x1000:4 motor_sign:4 rows_x1e6:12 trim_dps:4 spin_ref_mradps max_wheel_dps:4 current_limit_raw algebra_y_sign center_y_sign",
}
REQUIRED = {"STATUS", "WHEEL", "IMU", "KF", "YAW"}
CORE_TAGS = {"STATUS", "WHEEL", "IMU", "KF", "DIR", "OBS"}
SLOW_OBSERVATION_FIELDS = {"q_1e6", "gravity_mmps2", "innov_x1000", "r_x1e6"}


def _schema(tag, frame):
    schema = SCHEMAS[tag].split()
    if frame >= 19:
        if tag == "STATUS":
            schema += "imu_seq chassis_mode control_mode output_enabled".split()
        elif tag == "WHEEL":
            schema += "req_i:4 raw_dps:4 age_us:4 wheel_seq:4 scale_x1000".split()
        elif tag == "KF":
            schema += ["p_x1e6:4"]
        elif tag == "YAW":
            schema += ("diag cmd_t_us src online gmode rc used enc_mdeg gz_mdps temp_mC cal_temp_mC "
                       "cal bias_z_urps trim_z_urps trim trim_updates enc_t_us ctrl_t_us spd_p_raw "
                       "spd_i_raw sent_raw i_limit_raw").split()
        elif tag == "OBS":
            schema = [item for item in schema if item.partition(":")[0] not in SLOW_OBSERVATION_FIELDS]
    if tag == "OBS" and frame >= 15:
        schema += "dv_now_mmps:2 v_res_mmps:2 reject_reason".split()
    if tag == "OBS" and frame >= 17:
        schema += ("control_ready control_inhibit control_good_ms control_blend_x1000 "
                   "control_v_err_mmps control_a_res_mmps2").split()
    return schema


def _valid_fields(tag, fields, frame):
    if tag == "CONFIG" or (tag == "STATUS" and frame >= 19):
        if not FIRMWARE_FIELD.fullmatch("fw="+fields.get("fw", "")):
            return False
    for specification in _schema(tag, frame):
        name, _, size = specification.partition(":")
        if name not in fields or len(fields[name].split(",")) != int(size or 1):
            return False
    return True


def has_speed_spin_command(row):
    """Check recorded drive context; legacy rows did not report these fields."""
    status = row["STATUS"]
    return (status.get("chassis_mode", "1") == "1"      # CHASSIS_ROTATE
            and status.get("control_mode", "0") == "0"  # CHASSIS_CONTROL_VELOCITY
            and status.get("output_enabled", "1") == "1")


def parse(text):
    frames, invalid, rigid, issues = {}, set(), [], []
    core_keys, config_records, damaged = set(), [], {}
    tag_counts = Counter()
    for number, line in enumerate(text.splitlines(), 1):
        matches = list(TAG.finditer(line))
        if not matches:
            continue
        if len(matches) > 1:
            # 后半行即使完整，也不与其他记录拼成一组。
            merged_core = any(match[1] in CORE_TAGS for match in matches)
            for match in matches:
                seq = re.match(r"seq=(\d+)\b", line[match.end():])
                if seq:
                    key = int(seq[1])
                    frames.setdefault(key, {})
                    if merged_core:
                        invalid.add(key)
                    else:
                        damaged.setdefault(key, set()).add(match[1])
                    if match[1] in CORE_TAGS:
                        core_keys.add(key)
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
        if tag in CORE_TAGS:
            core_keys.add(key)
        row = frames.setdefault(key, {})
        fields, reason = {}, None
        for token in body.split():
            field = FIELD.fullmatch(token)
            if field is None and tag in ("STATUS", "CONFIG"):
                field = FIRMWARE_FIELD.fullmatch(token)
            if not field or field[1] in fields:
                reason = "invalid_or_duplicate_field"
                break
            fields[field[1]] = field[2]
        if tag == "WHEEL" and "req_i" in fields and len(fields["req_i"].split(",")) != 4:
            reason = "wrong_req_i_dimension"
        if not reason and tag == "CONFIG" and _valid_fields(tag, fields, 19):
            # Keep calibration even when the matching core sample is missing.
            config_records.append(fields)
        if tag in row:
            reason = "duplicate_tag"
        if reason:
            damaged.setdefault(key, set()).add(tag)
            issues.append({"line": number, "seq": key, "tag": tag, "reason": reason})
        else:
            row[tag] = fields
    rows = []
    for key, row in frames.items():
        frame_field = row.get("STATUS", {}).get("frame", "0")
        frame = int(frame_field) if "," not in frame_field else 0
        required = (REQUIRED - {"YAW"} if frame >= 19 else REQUIRED) | ({"DIR"} if frame >= 12 else set())
        if frame >= 14:
            required |= {"OBS"}
        # Version-dependent validation happens after gathering tags so OBS may
        # precede STATUS, and optional packets do not become required core data.
        broken = damaged.get(key, set())
        for tag, fields in row.items():
            if not _valid_fields(tag, fields, frame):
                broken.add(tag)
                issues.append({"seq": key, "tag": tag, "reason": "missing_field_or_wrong_dimension"})
        if frame >= 19:
            for tag in broken - CORE_TAGS:
                row.pop(tag, None)
            broken &= CORE_TAGS
        if key not in invalid and not broken and required <= row.keys():
            rows.append(row)
    accepted = {int(r["STATUS"]["seq"]) for r in rows}
    return rows, rigid, {
        "observed_sequences": len(core_keys), "complete_frames": len(rows),
        "incomplete_frames": len(core_keys) - len(rows),
        "diagnostic_only_sequences": len(frames) - len(core_keys),
        "rejected_sequences": [seq for seq in frames if seq in core_keys and seq not in accepted],
        "config_records": config_records,
        "issues": issues, "tag_counts": dict(tag_counts),
    }
