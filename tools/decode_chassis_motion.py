"""将历史 MCL1 二进制流转为 CSV；当前通道 0 已改为文本，不适用此解码器。"""
import argparse
import csv
import struct
from pathlib import Path

VERSION = 0x20261111
HEADER = struct.Struct("<4sII")
# 字段顺序与 ChassisMotionState 一致；数组在 CSV 中展开为 name_0...。
FIELDS = [
    ("version", "I", 1), ("seq", "I", 1), ("time_us", "I", 1),
    ("cmd_age_us", "I", 1), ("log_drop", "I", 1),
    ("wheel_seq", "I", 4), ("wheel_age_us", "I", 4),
    ("imu_age_us", "I", 1), ("imu_seq", "I", 1),
    ("dt_s", "f", 1), ("theta_bg", "f", 1), ("psi_bn", "f", 1),
    ("psi_gn", "f", 1), ("omega_b", "f", 1), ("omega_imu", "f", 1),
    ("motor_dps", "f", 4), ("raw_motor_dps", "f", 4),
    ("wheel_mps", "f", 4), ("wheel_residual", "f", 4),
    ("current_fb", "f", 4), ("current_send", "f", 4),
    ("wheel_req_dps", "f", 4), ("wheel_eff_dps", "f", 4),
    ("current_request", "f", 4), ("imu_q", "f", 4),
    ("imu_accel_raw", "f", 3), ("imu_motion_b", "f", 3),
    ("imu_motion_n", "f", 3), ("imu_gyro_rad", "f", 3),
    ("wheel_b", "f", 2), ("wheel_n", "f", 2), ("accel_center_n", "f", 2),
    ("x", "f", 4), ("p_diag", "f", 4), ("innovation", "f", 4),
    ("hat_g", "f", 2), ("user_g", "f", 2), ("ref_g", "f", 2),
    ("error_g", "f", 2), ("dv_raw_g", "f", 2), ("dv_g", "f", 2), ("cmd_g", "f", 2),
    ("omega_ref", "f", 1), ("kp", "f", 1), ("wheel_scale", "f", 1),
    ("mode", "B", 1), ("estimate_valid", "B", 1),
    ("wheel_valid_mask", "B", 1), ("accel_used", "B", 1),
    ("observation_mask", "B", 1), ("exit_reason", "B", 1),
    ("limit_flags", "B", 1), ("power_limit_active", "B", 1),
    ("current_saturated_mask", "B", 1), ("accel_center_valid", "B", 1), ("reserved", "B", 2),
]
PAYLOAD = struct.Struct("<" + "".join(f"{count}{kind}" for _, kind, count in FIELDS))
COLUMNS = [name if count == 1 else f"{name}_{i}"
           for name, _, count in FIELDS for i in range(count)]


def decode(data):
    """跳过损坏/截断前缀；长度和版本同时匹配后才接受完整帧。"""
    pos = 0
    while True:
        pos = data.find(b"MCL1", pos)
        if pos < 0 or pos + HEADER.size > len(data):
            return
        _, size, dropped = HEADER.unpack_from(data, pos)
        if size != PAYLOAD.size:
            pos += 1
            continue
        end = pos + HEADER.size + size
        if end > len(data):
            return
        values = PAYLOAD.unpack_from(data, pos + HEADER.size)
        if values[0] not in (0x20261016, 0x20261026, 0x20261027, 0x20261028, 0x20261029, 0x20261030, 0x20261031, 0x20261105, 0x20261106, 0x20261107, 0x20261108, 0x20261110, VERSION):
            pos += 1
            continue
        yield [dropped, *values]
        pos = end


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    count = 0
    with args.output.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(["transport_drop", *COLUMNS])
        for row in decode(args.input.read_bytes()):
            writer.writerow(row)
            count += 1
    print(f"Decoded {count} frames; payload {PAYLOAD.size} bytes -> {args.output}")


if __name__ == "__main__":
    main()
