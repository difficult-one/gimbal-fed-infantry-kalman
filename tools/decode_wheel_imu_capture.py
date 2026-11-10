"""Decode one WIMU v1 capture and optionally export its replay metadata.

IMU vectors ending in _n retain native INS navigation axes. Control vectors
ending in _n use chassis navigation axes (INS x, -y, -z).
"""

import argparse
import hashlib
import json
import math
import struct
import sys
import zlib
from pathlib import Path


HEADER = struct.Struct("<4sHH8I8H31fbbBB")
CONFIG_OFFSET = struct.calcsize("<4sHH8I8H")
IMU = struct.Struct("<4I6h4B22f")
MOTOR = struct.Struct("<3IhHf")
CONTROL = struct.Struct("<8I19fH6B")
CRC = struct.Struct("<I")
F32 = struct.Struct("<f")
WHEELS = ("LF", "RF", "LB", "RB")
SOURCES = ("imu", *WHEELS, "yaw", "control")
RECORDS = (IMU, MOTOR, MOTOR, MOTOR, MOTOR, MOTOR, CONTROL)
LIMITS = (368, 368, 368, 368, 368, 368, 80)
MAX_CAPTURE_BYTES = HEADER.size + sum(record.size * limit for record, limit in zip(RECORDS, LIMITS)) + CRC.size


def finite_or_none(values):
    return [value if math.isfinite(value) else None for value in values]


def imu_record(row):
    values = finite_or_none(row[14:])
    return {
        "t_read_us": row[0], "accepted_seq": row[1], "attempt_seq": row[2], "generation": row[3],
        "accel_counts": list(row[4:7]), "gyro_counts": list(row[7:10]),
        "read_status": row[10], "valid_flags": row[11], "attitude_flags": row[12], "rigid_reject": row[13],
        "accel_mps2": values[0:3], "gyro_radps": values[3:6], "q": values[6:10],
        "gravity_b_mps2": values[10:13], "specific_force_after_lever_mps2": values[13:16],
        "motion_n_mps2": values[16:19], "center_n_mps2": values[19:22],
    }


def motor_record(row):
    return {
        "t_receive_us": row[0], "seq": row[1], "generation": row[2],
        "rpm": row[3], "encoder_count": row[4], "filtered_dps": finite_or_none(row[5:])[0],
    }


def control_record(row):
    values = finite_or_none(row[8:27])
    return {
        "time_us": row[0], "seq": row[1], "generation": row[2], "imu_seq": row[3], "wheel_seq": list(row[4:8]),
        "theta_bg_rad": values[0], "psi_bn_rad": values[1], "wheel_n_mps": values[2:4],
        "accel_center_n_mps2": values[4:6], "wheel_eff_dps": values[6:10], "omega_ref_radps": values[10],
        "ref_g_mps": values[11:13], "cmd_g_mps": values[13:15], "dv_now_mps": values[15:17],
        "dv_res_mps": values[17:19], "control_inhibit": row[27], "reject_reason": row[28],
        "exit_reason": row[29], "wheel_valid_mask": row[30], "observation_mask": row[31],
        "flags": row[32], "mode": row[33],
    }


def decode_capture(data, firmware_sha256=None):
    """Decode complete bytes and retain original unsigned timestamps and sequence IDs."""
    if len(data) < HEADER.size + CRC.size:
        raise ValueError("truncated WIMU header or CRC32")
    header = HEADER.unpack_from(data)
    magic, version, header_size = header[:3]
    if magic != b"WIMU":
        raise ValueError("invalid WIMU magic")
    if version != 1:
        raise ValueError(f"unsupported WIMU version {version}")
    if header_size != HEADER.size:
        raise ValueError(f"invalid WIMU header size {header_size}")

    fw, frame, config_version, token, start_us, end_us, stop_reason, overflow_source = header[3:11]
    counts = header[11:18]
    algebra_y_sign, center_y_sign, drive, reserved = header[50:]
    if header[18] or reserved:
        raise ValueError("nonzero WIMU reserved field")
    if any(count > limit for count, limit in zip(counts, LIMITS)):
        raise ValueError("WIMU record count exceeds capture bounds")
    payload_size = HEADER.size + sum(record.size * count for record, count in zip(RECORDS, counts))
    if len(data) != payload_size + CRC.size:
        raise ValueError("WIMU length does not match record counts")
    if zlib.crc32(memoryview(data)[:payload_size]) != CRC.unpack_from(data, payload_size)[0]:
        raise ValueError("WIMU CRC32 mismatch")
    if stop_reason not in (1, 2, 3, 4) or (stop_reason == 2 and not 1 <= overflow_source <= len(SOURCES)):
        raise ValueError("invalid WIMU stop reason or overflow source")
    if stop_reason != 2 and overflow_source:
        raise ValueError("unexpected WIMU overflow source")
    if (drive, algebra_y_sign, center_y_sign) != (1, -1, -1):
        raise ValueError("unsupported WIMU drive or coordinate signs")
    if not frame:
        raise ValueError("invalid WIMU frame revision")

    config = header[19:50]
    if not all(math.isfinite(value) for value in config):
        raise ValueError("non-finite WIMU capture configuration")
    radius, ratio, sign = config[0:4], config[4:8], config[8:12]
    if any(value <= 0 for value in radius):
        raise ValueError("invalid WIMU wheel radius")
    if any(value <= 0 for value in ratio):
        raise ValueError("invalid WIMU gear ratio")
    if any(value not in (-1, 1) for value in (*sign, config[29])):
        raise ValueError("invalid WIMU motor or yaw encoder sign")
    # The text diagnostics multiply in float32 before truncating to integer mrad/s.
    spin_ref_scaled = config[28] * 1000
    if abs(spin_ref_scaled) >= 2**31:
        raise ValueError("WIMU spin reference is outside the text log range")
    spin_ref_mradps = int(F32.unpack(F32.pack(spin_ref_scaled))[0])
    if not 0 < abs(spin_ref_mradps) < 2147483520:
        raise ValueError("invalid WIMU spin reference")
    raw_sha256 = hashlib.sha256(data).hexdigest()
    config_sha256 = hashlib.sha256(data[CONFIG_OFFSET:HEADER.size]).hexdigest()
    metadata = {
        "schema_version": 1,
        "capture_id": f"wimu-v1-{raw_sha256}",
        "fw": f"{fw:08x}",
        "frame": frame,
        "config_id": f"{config_version:08x}-{config_sha256}",
        "raw_sha256": raw_sha256,
        "drive": "OMNI_X",
        "wheel_order": list(WHEELS),
        "wheel_radius_m": list(radius),
        "gear_ratio": list(ratio),
        "motor_sign": list(sign),
        "kinematics": [list(config[i:i + 3]) for i in range(12, 24, 3)],
        "coordinates": "n_ins_x_neg_y_neg_z_legacy_y",
        "spin_wheel_trim_dps": list(config[24:28]),
        "spin_ref_mradps": spin_ref_mradps,
        "yaw_encoder_sign": config[29],
        "yaw_zero_deg": config[30],
        "algebra_y_sign": algebra_y_sign,
        "center_y_sign": center_y_sign,
    }
    if firmware_sha256 is not None:
        metadata["firmware_sha256"] = firmware_sha256

    streams = {}
    offset = HEADER.size
    decoders = (imu_record, motor_record, motor_record, motor_record, motor_record, motor_record, control_record)
    for source, count, record, decode in zip(SOURCES, counts, RECORDS, decoders):
        end = offset + count * record.size
        streams[source] = [decode(row) for row in record.iter_unpack(memoryview(data)[offset:end])]
        offset = end
    return {
        "metadata": metadata,
        "capture": {
            "wire_version": version, "token": token, "start_us": start_us, "end_us": end_us,
            "stop_reason": stop_reason, "overflow_source": overflow_source,
            "counts": dict(zip(SOURCES, counts)),
        },
        "imu": streams["imu"],
        "motors": {source: streams[source] for source in (*WHEELS, "yaw")},
        "control": streams["control"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="complete WIMU binary capture")
    parser.add_argument("--output", type=Path, help="decoded JSON path, defaults to stdout")
    parser.add_argument("--metadata-out", type=Path, help="also write the replay metadata object separately")
    parser.add_argument("--firmware-artifact", type=Path, help="add SHA-256 of the supplied firmware artifact")
    args = parser.parse_args()
    try:
        paths = [path.resolve() for path in (args.input, args.output, args.metadata_out, args.firmware_artifact)
                 if path is not None]
        if len(set(paths)) != len(paths):
            raise ValueError("input, output, metadata and firmware paths must be distinct")
        with args.input.open("rb") as stream:
            raw = stream.read(MAX_CAPTURE_BYTES + 1)
        if len(raw) > MAX_CAPTURE_BYTES:
            raise ValueError("WIMU length exceeds capture bounds")
        firmware_sha256 = None
        if args.firmware_artifact is not None:
            firmware_sha256 = hashlib.sha256(args.firmware_artifact.read_bytes()).hexdigest()
        capture = decode_capture(raw, firmware_sha256)
        decoded = json.dumps(capture, allow_nan=False) + "\n"
        if args.output is None:
            sys.stdout.write(decoded)
        else:
            args.output.write_text(decoded, encoding="utf-8")
        if args.metadata_out is not None:
            args.metadata_out.write_text(json.dumps(capture["metadata"], indent=2, allow_nan=False) + "\n",
                                         encoding="utf-8")
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
