"""Validate the per-capture configuration for omni wheel replay."""

import hashlib
import json
import math
import re
import sys

import numpy as np


WHEEL_ORDER = ("LF", "RF", "LB", "RB")
COORDINATES = "n_ins_x_neg_y_neg_z_legacy_y"
SHA256 = re.compile(r"[0-9a-f]{64}")
C45 = math.sqrt(0.5)
ARM_M = 0.325 * C45


def read_capture_input(path, metadata_path):
    raw = path.read_bytes()
    metadata = None
    if metadata_path is not None:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8-sig"))
        if not isinstance(metadata, dict):
            raise ValueError("metadata must be a JSON object")
    return raw.decode("utf-8-sig"), metadata, hashlib.sha256(raw).hexdigest()


def _finite(value):
    return type(value) in (int, float) and abs(value) <= sys.float_info.max


def _validate_vector(key, values, size, check):
    if not isinstance(values, list) or len(values) != size or not all(check(value) for value in values):
        raise ValueError(f"invalid metadata {key}: expected {size} valid numbers")


def _legacy_config(fw, frame, raw_sha256):
    if not 1 <= frame <= 16:
        raise ValueError(f"frame={frame} requires explicit capture --metadata")
    radius_m = 0.0725 if frame == 16 else 0.060
    return {
        "schema_version": 1,
        "source": "registered_legacy",
        "fw": fw,
        "frame": frame,
        "config_id": "legacy-145mm" if frame == 16 else "legacy-120mm",
        "raw_sha256": raw_sha256,
        "drive": "OMNI_X",
        "wheel_order": list(WHEEL_ORDER),
        "wheel_radius_m": [radius_m] * 4,
        "gear_ratio": [19] * 4,
        "motor_sign": [1] * 4,
        "kinematics": [[-C45, C45, ARM_M], [C45, C45, ARM_M],
                       [-C45, -C45, ARM_M], [C45, -C45, ARM_M]],
        "coordinates": COORDINATES,
        "spin_wheel_trim_dps": [0] * 4,
        "spin_ref_mradps": 6575 if frame == 16 else 8162,
    }


def motor_mps_per_dps(config):
    return (np.asarray(config["motor_sign"], dtype=float) * math.pi / 180
            * np.asarray(config["wheel_radius_m"], dtype=float) / np.asarray(config["gear_ratio"], dtype=float))


def _validate_geometry(metadata):
    if metadata.get("drive") != "OMNI_X" or metadata.get("wheel_order") != list(WHEEL_ORDER):
        raise ValueError("metadata drive/wheel_order must be OMNI_X with LF/RF/LB/RB order")
    if metadata.get("coordinates") != COORDINATES:
        raise ValueError("unsupported metadata coordinates convention")
    if type(metadata.get("spin_ref_mradps")) is not int or metadata["spin_ref_mradps"] == 0:
        raise ValueError("metadata spin_ref_mradps must be a nonzero integer")

    for key in ("wheel_radius_m", "gear_ratio", "loaded_rolling_radius_m"):
        if key != "loaded_rolling_radius_m" or key in metadata:
            _validate_vector(key, metadata.get(key), 4, lambda value: _finite(value) and value > 0)
    _validate_vector("motor_sign", metadata.get("motor_sign"), 4,
                     lambda value: _finite(value) and value in (-1, 1))
    _validate_vector("spin_wheel_trim_dps", metadata.get("spin_wheel_trim_dps"), 4, _finite)
    kinematics = metadata.get("kinematics")
    if not isinstance(kinematics, list) or len(kinematics) != 4:
        raise ValueError("metadata kinematics must have four [Jx, Jy, arm_m] rows")
    for row in kinematics:
        _validate_vector("kinematics", row, 3, _finite)
    if np.linalg.matrix_rank(np.asarray(kinematics, dtype=float)) != 3:
        raise ValueError("metadata kinematics cannot resolve a three-axis chassis twist")
    with np.errstate(over="ignore", under="ignore"):
        scale = motor_mps_per_dps(metadata)
    if not np.all(np.isfinite(scale) & (scale != 0)):
        raise ValueError("metadata wheel_radius_m/gear_ratio produce invalid speed scales")


def _embedded_config(records, fw, frame, raw_sha256):
    calibration = None
    for record in records:
        if (frame < 19 or record["fw"].lower() != fw or int(record["frame"]) != frame
                or not re.fullmatch(r"[0-9a-f]{8}", fw)):
            raise ValueError(f"embedded CONFIG fw/frame does not match STATUS fw={fw} frame={frame}")
        values = {key: value for key, value in record.items() if key != "seq"}
        values["fw"] = values["fw"].lower()
        if calibration is not None and values != calibration:
            raise ValueError("conflicting embedded CONFIG records in capture")
        calibration = values

    def number(key):
        return int(calibration[key])

    def vector(key, scale=1):
        values = [int(value) for value in calibration[key].split(",")]
        return [value / scale for value in values] if scale != 1 else values

    # Firmware's calibrated OMNI_X is CK_CUSTOM=3 with reflected algebra Y.
    # Existing analyzers cannot infer arbitrary custom or steered wheel axes.
    if (number("drive"), number("algebra_y_sign"), number("center_y_sign")) != (3, -1, -1):
        raise ValueError("unsupported embedded CONFIG drive or coordinates for OMNI_X replay")
    for key in ("direct", "rotation_priority", "imu_on_gimbal", "imu_follows_pitch"):
        if number(key) not in (0, 1):
            raise ValueError(f"invalid embedded CONFIG {key}: expected 0 or 1")
    for key in ("period_us", "decimation", "current_limit_raw"):
        if number(key) <= 0:
            raise ValueError(f"invalid embedded CONFIG {key}: expected a positive value")
    max_wheel_dps = vector("max_wheel_dps")
    _validate_vector("max_wheel_dps", max_wheel_dps, 4, lambda value: value > 0)
    kinematics = vector("rows_x1e6", 1_000_000)
    identifier = hashlib.sha256(json.dumps(calibration, sort_keys=True).encode()).hexdigest()
    config = {
        "schema_version": 1, "source": "embedded_config", "capture_id": "embedded-rtt",
        "config_id": f"{fw}-{identifier}", "raw_sha256": raw_sha256, "fw": fw, "frame": frame,
        "drive": "OMNI_X", "wheel_order": list(WHEEL_ORDER), "coordinates": COORDINATES,
        "wheel_radius_m": vector("radius_um", 1_000_000), "gear_ratio": vector("gear_x1000", 1000),
        "motor_sign": vector("motor_sign"), "kinematics": [kinematics[i:i+3] for i in range(0, 12, 3)],
        "spin_wheel_trim_dps": vector("trim_dps"), "spin_ref_mradps": number("spin_ref_mradps"),
        "period_us": number("period_us"), "decimation": number("decimation"),
        "direct_translation": number("direct"), "rotation_priority": number("rotation_priority"),
        "imu_on_gimbal": number("imu_on_gimbal"), "imu_follows_pitch": number("imu_follows_pitch"),
        "max_wheel_dps": max_wheel_dps, "current_limit_raw": number("current_limit_raw"),
    }
    _validate_geometry(config)
    return config


def _validate_embedded_overlap(metadata, embedded):
    # Firmware truncates scaled floats to integers. The sidecar may retain
    # more precision, but should agree within one wire unit.
    tolerances = {"wheel_radius_m": 1e-6, "gear_ratio": 1e-3, "kinematics": 1e-6,
                  "spin_wheel_trim_dps": 1, "spin_ref_mradps": 1,
                  "motor_sign": 0, "period_us": 0, "decimation": 0}
    for key, tolerance in tolerances.items():
        if key not in metadata:
            continue
        matches = (np.allclose(metadata[key], embedded[key], rtol=0, atol=tolerance+1e-12)
                   if tolerance else metadata[key] == embedded[key])
        if not matches:
            raise ValueError(f"metadata {key} disagrees with embedded CONFIG")


def resolve_capture_config(rows, metadata, raw_sha256, embedded_config_records=None):
    """Return one checked configuration for all complete STATUS rows.

    Pass parser-reported CONFIG records to include packets whose core was lost.
    Row-attached CONFIG is also accepted for existing three-argument callers.
    A capture-bound sidecar can add measured values or higher precision, but
    must agree with the firmware calibration when embedded records exist.
    """
    versions = {(row["STATUS"].get("fw", "").lower(), int(row["STATUS"]["frame"])) for row in rows}
    if len(versions) != 1:
        raise ValueError("capture must contain complete rows with a single STATUS fw/frame")
    fw, frame = versions.pop()
    records = embedded_config_records if embedded_config_records is not None else [
        row["CONFIG"] for row in rows if "CONFIG" in row]
    embedded = _embedded_config(records, fw, frame, raw_sha256) if records else None
    if metadata is None:
        return embedded if embedded is not None else _legacy_config(fw, frame, raw_sha256)
    if not isinstance(metadata, dict):
        raise ValueError("metadata must be a JSON object")

    if type(metadata.get("schema_version")) is not int or metadata["schema_version"] != 1:
        raise ValueError("unsupported metadata schema_version")
    for key in ("capture_id", "config_id"):
        if not isinstance(metadata.get(key), str) or not metadata[key].strip():
            raise ValueError(f"invalid metadata {key}")
    if not re.fullmatch(r"[0-9a-f]{8}", fw) or metadata.get("fw") != fw:
        raise ValueError(f"metadata fw does not match STATUS fw={fw}")
    if type(metadata.get("frame")) is not int or frame <= 0 or metadata["frame"] != frame:
        raise ValueError(f"metadata frame does not match STATUS frame={frame}")
    digest = metadata.get("raw_sha256")
    if not isinstance(digest, str) or not SHA256.fullmatch(digest) or digest != raw_sha256:
        raise ValueError("metadata raw_sha256 does not match the capture bytes")
    if "firmware_sha256" in metadata:
        digest = metadata["firmware_sha256"]
        if not isinstance(digest, str) or not SHA256.fullmatch(digest):
            raise ValueError("invalid metadata firmware_sha256")
    for key in ("period_us", "decimation"):
        if key in metadata and (type(metadata[key]) is not int or metadata[key] <= 0):
            raise ValueError(f"invalid metadata {key}: expected a positive integer")
    _validate_geometry(metadata)
    if embedded:
        _validate_embedded_overlap(metadata, embedded)
    sampling = {key: embedded[key] for key in ("period_us", "decimation")} if embedded else {}
    return {**metadata, **sampling, "source": "capture_metadata"}
