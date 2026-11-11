"""Replay captures using the calibration recorded at collection time."""

import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from analyze_chassis_run import wheel_g
from analyze_spin_phase import select_spin_rows, spin_reference, wheel_radius_m
from chassis_capture_config import resolve_capture_config
from chassis_text_parser import parse


C45 = math.sqrt(0.5)
ARM = 0.325 * C45


def row(frame):
    return {"STATUS": {"frame": str(frame), "fw": "20261110"},
            "WHEEL": {"dps": "-19000,19000,-19000,19000"},
            "IMU": {"theta_mrad": "0"}}


def log_text(*, pure_spin=False, fw="202611ab"):
    lines = []
    for i in range(1, 11):
        seq = i * 20
        t_us = i * 100000
        theta = (i - 5) * 300
        ref = 0 if pure_spin else 250
        # Independently project +Gx through a -theta rotation onto the omni directions.
        x = ref * 0.001 * math.cos(theta * 0.001)
        y = -ref * 0.001 * math.sin(theta * 0.001)
        directions = ((-1, 1), (1, 1), (-1, -1), (1, -1))
        rotation = 6.892 * ARM
        motor_dps = ",".join(str(round((rotation + (dx*x + dy*y)*C45)
                                         / scale))
                             for (dx, dy), scale in zip(directions, (0.00006, -0.00007,
                                                                    0.00008, 0.00009)))
        lines.extend((
            f"[STATUS] seq={seq} t_us={t_us} fw={fw} frame=17 mode=2 valid=1 exit=0 "
            "accel=1 wheel_mask=15 obs=15 dt_us=5000 imu_age_us=700 cmd_age_us=26 qdrop=0 tdrop=0",
            f"[WHEEL] seq={seq} dps={motor_dps} req_dps={motor_dps} req_i=1,1,1,1 send_i=1,1,1,1 fb_i=1,1,1,1 "
            "vn_mmps=120,-30 omega_mradps=6892 omega_ref_mradps=6892",
            f"[IMU] seq={seq} corr=1 theta_mrad={theta} psi_bn_mrad=0 psi_gn_mrad={theta} "
            "gyro_mradps=0,0,3000 raw_a_mmps2=0,0,9810 nav_a_mmps2=0,0,0 center_a_mmps2=0,0",
            f"[KF] seq={seq} v_n_mmps=0,0 a_n_mmps2=0,0 user_g_mmps={ref},0 ref_g_mmps={ref},0 "
            f"hat_g_mmps={ref},0 err_g_mmps=0,0 dv_g_mmps=0,0 cmd_g_mmps={ref},0 kp_x1000=0 flags=0 sat=0",
            f"[YAW] seq={seq} ref_mdeg=0 fb_mdeg=0 out_raw=0",
            f"[DIR] seq={seq} theta_out_mrad={theta} rel_omega_mradps=3000 lead_mrad=0 rate_valid=1",
            f"[OBS] seq={seq} imu_t_us={t_us-700} clock_gen=1 recovering=0 good_ms=500 "
            "q_1e6=1000000,0,0,0 gravity_mmps2=0,0,9810 tilt_mrad=0,0 chi_x1000=0 att_flags=3 "
            "innov_x1000=0,0,0,0 r_x1e6=2500,40000,2500,40000 dv_res_mmps=0,0 window_ms=250 "
            "gate=1 rejects=0 dv_now_mmps=0,0 v_res_mmps=0,0 reject_reason=0 control_ready=0 "
            "control_inhibit=0 control_good_ms=500 control_blend_x1000=0 control_v_err_mmps=0 control_a_res_mmps2=0",
        ))
    return ("\r\n".join(lines) + "\r\n").encode("utf-8")


def metadata_for(raw):
    radius = 0.076
    return {"schema_version": 1, "capture_id": "synthetic", "fw": "202611ab", "frame": 17,
            "config_id": "test-152", "raw_sha256": hashlib.sha256(raw).hexdigest(),
            "drive": "OMNI_X", "wheel_order": ["LF", "RF", "LB", "RB"],
            "wheel_radius_m": [radius]*4, "motor_sign": [1, -1, 1, 1],
            "gear_ratio": [math.radians(radius)/s for s in (0.00006, 0.00007, 0.00008, 0.00009)],
            "kinematics": [[-C45, C45, ARM], [C45, C45, ARM],
                           [-C45, -C45, ARM], [C45, -C45, ARM]],
            "coordinates": "n_ins_x_neg_y_neg_z_legacy_y", "spin_wheel_trim_dps": [66, -107, 29, 12],
            "spin_ref_mradps": 6892}


def embedded_log():
    raw = log_text(fw="20261111")
    lines = []
    for line in raw.decode().splitlines():
        seq = line.split("seq=", 1)[1].split()[0]
        if line.startswith("[YAW]"):
            continue
        if line.startswith("[STATUS]"):
            line = line.replace("frame=17", "frame=19")
            line += " imu_seq=100 chassis_mode=1 control_mode=0 output_enabled=1"
        elif line.startswith("[WHEEL]"):
            line += " raw_dps=0,0,0,0 age_us=1,2,3,4 wheel_seq=1,2,3,4 scale_x1000=1000"
        elif line.startswith("[KF]"):
            line += " p_x1e6=1,2,3,4"
        elif line.startswith("[OBS]"):
            line = " ".join(token for token in line.split() if not token.startswith(
                ("q_1e6=", "gravity_mmps2=", "innov_x1000=", "r_x1e6=")))
        lines.append(line)
        if seq == "20" and line.startswith("[STATUS]"):
            gear = ",".join(str(round(ratio*1000)) for ratio in metadata_for(raw)["gear_ratio"])
            rows = ",".join(str(round(value*1e6)) for row in metadata_for(raw)["kinematics"] for value in row)
            lines.append(f"[CONFIG] seq=20 fw=20261111 frame=19 period_us=5000 decimation=20 direct=1 "
                         f"rotation_priority=0 imu_on_gimbal=1 imu_follows_pitch=1 drive=3 radius_um=76000,76000,76000,76000 "
                         f"gear_x1000={gear} motor_sign=1,-1,1,1 rows_x1e6={rows} trim_dps=66,-107,29,12 "
                         "spin_ref_mradps=6892 max_wheel_dps=36000,36000,36000,36000 current_limit_raw=8000 "
                         "algebra_y_sign=-1 center_y_sign=-1")
    return ("\r\n".join(lines)+"\r\n").encode()


class CaptureCalibrationTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.capture = Path(directory.name) / "synthetic.txt"
        self.sidecar = Path(directory.name) / "synthetic.metadata.json"
        raw = log_text()
        self.capture.write_bytes(raw)
        self.metadata = metadata_for(raw)
        self.write_metadata()

    def write_metadata(self, **updates):
        self.sidecar.write_text(json.dumps({**self.metadata, **updates}), encoding="utf-8")

    def run_script(self, script, *args):
        return subprocess.run([sys.executable, "-B", "-X", "utf8", str(ROOT / "tools" / script),
                               *(str(arg) for arg in args)], capture_output=True, encoding="utf-8")

    def test_legacy_scale_and_default_reference(self):
        old, new = wheel_g(row(15)), wheel_g(row(16))
        self.assertAlmostEqual(old[0], 1480.9609794, places=6)
        self.assertAlmostEqual(new[0], 1789.4945168, places=6)
        self.assertAlmostEqual(old[1], 0.0)
        self.assertAlmostEqual(new[1], 0.0)
        self.assertEqual(spin_reference([row(15)]), 8162)
        self.assertEqual(spin_reference([row(16)]), 6575)
        self.assertEqual(spin_reference([row(16)], 9000), 9000)

    def test_frame19_embedded_configuration_replays_without_sidecar(self):
        raw = embedded_log()
        self.capture.write_bytes(raw)
        result = self.run_script("analyze_chassis_run.py", self.capture)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        config = report["capture_config"]
        self.assertEqual(config["source"], "embedded_config")
        self.assertEqual(config["frame"], 19)
        self.assertEqual(config["direct_translation"], 1)
        self.assertEqual(config["wheel_radius_m"], [.076]*4)
        self.assertEqual(config["raw_sha256"], hashlib.sha256(raw).hexdigest())
        self.assertAlmostEqual(report["directions"]["all"]["actual_mean_mmps"], 250, delta=.1)

    def test_embedded_configuration_rejects_stale_conflicting_or_unsupported_values(self):
        good = embedded_log().decode()
        config = next(line for line in good.splitlines() if line.startswith("[CONFIG]"))
        cases = (
            good.replace(config, config.replace("fw=20261111", "fw=20261110")),
            good.replace(config, config.replace("frame=19", "frame=18")),
            good.replace(config, config.replace("radius_um=76000,76000,76000,76000", "radius_um=76000,0,76000,76000")),
            good.replace(config, config.replace("drive=3", "drive=2")),
            good.replace(config, config.replace("algebra_y_sign=-1", "algebra_y_sign=1")),
            good + config.replace("seq=20", "seq=40").replace("direct=1", "direct=0")+"\n",
            good + config.replace("seq=20", "seq=300").replace("direct=1", "direct=0")+"\n",
        )
        for text in cases:
            with self.subTest(config=text[text.find("[CONFIG]"):text.find("[CONFIG]")+200]):
                rows, _, parsing = parse(text)
                with self.assertRaises(ValueError):
                    resolve_capture_config(rows, None, hashlib.sha256(text.encode()).hexdigest(), parsing["config_records"])
        # Repeated records with identical calibration, even without their core frame, are valid.
        repeated = good + config.replace("seq=20", "seq=40")+"\n"
        rows, _, parsing = parse(repeated)
        self.assertEqual(resolve_capture_config(rows, None, hashlib.sha256(repeated.encode()).hexdigest(),
                                                parsing["config_records"])["frame"], 19)
        orphaned = good.replace(config, config.replace("seq=20", "seq=300"))
        rows, _, parsing = parse(orphaned)
        self.assertEqual(parsing["complete_frames"], 10)
        self.assertEqual(parsing["diagnostic_only_sequences"], 1)
        self.assertEqual(resolve_capture_config(rows, None, None, parsing["config_records"])["frame"], 19)

    def test_frame19_spin_analysis_excludes_non_speed_spin_commands(self):
        text = embedded_log().decode()
        for original, substitute in (("chassis_mode=1", "chassis_mode=3"),
                                     ("control_mode=0", "control_mode=1"),
                                     ("output_enabled=1", "output_enabled=0")):
            with self.subTest(field=original):
                invalid = next(line for line in text.splitlines() if line.startswith("[STATUS] seq=40 "))
                altered = text.replace(invalid, invalid.replace(original, substitute))
                rows, _, parsing = parse(altered)
                config = resolve_capture_config(rows, None, None, parsing["config_records"])
                selected, rates = select_spin_rows(rows, config=config)
                self.assertEqual([row["STATUS"]["seq"] for row in selected], ["160", "180", "200"])
                self.assertEqual(len(rates), 7)
                self.capture.write_text(altered, encoding="utf-8")
                report = self.run_script("analyze_chassis_run.py", self.capture)
                self.assertEqual(report.returncode, 0, report.stderr)
                self.assertEqual(json.loads(report.stdout)["spin_execute"]["n"], 9)

    def test_sidecar_must_agree_with_logged_configuration_within_quantization(self):
        raw = embedded_log()
        digest = hashlib.sha256(raw).hexdigest()
        rows, _, parsing = parse(raw.decode())
        records = parsing["config_records"]
        config = resolve_capture_config(rows, None, digest, records)
        sidecar = {**config, "wheel_radius_m": [.0760005]*4, "loaded_rolling_radius_m": [.073]*4}
        resolved = resolve_capture_config(rows, sidecar, digest, records)
        self.assertEqual(resolved["loaded_rolling_radius_m"], [.073]*4)
        for key, stale in (("wheel_radius_m", [.0725]*4), ("gear_ratio", [19]*4),
                           ("motor_sign", [1]*4), ("decimation", 10), ("spin_ref_mradps", 8162)):
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, key):
                resolve_capture_config(rows, {**sidecar, key: stale}, digest, records)

    def test_unregistered_frames_and_mixed_firmware_require_explicit_configuration(self):
        for frame in (17, 18):
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                wheel_radius_m(row(frame))
        with self.assertRaises(ValueError):
            spin_reference([row(15), row(16)], 9000)

    def test_capture_config_scales_all_wheels_before_phase_and_velocity_fit(self):
        for script in ("analyze_spin_phase.py", "analyze_chassis_run.py"):
            with self.subTest(script=script):
                args = (self.capture, "--metadata", self.sidecar)
                result = self.run_script(script, *args)
                self.assertEqual(result.returncode, 0, result.stderr)
                report = json.loads(result.stdout)
                self.assertEqual(report["wheel_radius_m"], [0.076])
                self.assertEqual(report["spin_ref_mradps"], 6892)
                self.assertEqual(report["capture_config"]["raw_sha256"], self.metadata["raw_sha256"])
                self.assertEqual(result.stdout, self.run_script(script, *args).stdout)
                if script == "analyze_spin_phase.py":
                    self.assertAlmostEqual(report["mean_phase_deg"], 0.0, delta=0.08)
                    for fit in report["four_wheel_fits"].values():
                        self.assertAlmostEqual(fit["gain"], 1.0, delta=0.002)
                else:
                    self.assertAlmostEqual(report["directions"]["all"]["actual_mean_mmps"], 250, delta=0.08)

    def test_reference_override_cannot_bypass_missing_metadata(self):
        self.capture.write_bytes(log_text(fw="20261110"))
        result = self.run_script("analyze_chassis_run.py", self.capture, "--spin-ref-mradps", 6892)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("metadata", result.stderr)

    def test_invalid_metadata_cannot_produce_replay(self):
        cases = (
            ("raw_sha256", "0" * 64), ("schema_version", 2), ("fw", "202611ac"), ("frame", 18),
            ("wheel_radius_m", [0.076, 0.076, 0.0, 0.076]),
            ("gear_ratio", [19, float("nan"), 19, 19]), ("motor_sign", [True, -1, 1, 1]),
            ("kinematics", [[0, 0, 0]]*4), ("drive", "MECANUM"),
        )
        for key, value in cases:
            with self.subTest(key=key):
                self.write_metadata(**{key: value})
                result = self.run_script("analyze_chassis_run.py", self.capture, "--metadata", self.sidecar,
                                         "--spin-ref-mradps", 6892)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(key, result.stderr)

    def test_frame17_summary_uses_mps_and_a_versioned_report(self):
        self.capture.write_bytes(log_text(pure_spin=True, fw="20261110"))
        output = self.capture.with_suffix(".json")
        result = self.run_script("analyze_frame17_spin_drift.py", "--basic", self.capture,
                                 "--spin-a", self.capture, "--spin-b", self.capture, "--output", output)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(report.get("schema_version"), 2)
        plateau = report["runs"][0]["rotation_plateau"]
        self.assertNotIn("wheel_odometry_n_velocity_mmps", plateau)
        velocity = plateau["wheel_odometry_n_velocity_mps"]
        self.assertAlmostEqual(velocity["x"]["mean"], 0.120)
        self.assertAlmostEqual(velocity["y"]["rms"], 0.030)
        self.assertAlmostEqual(plateau["wheel_odometry_integrated_candidate_m"][0], 0.108)


if __name__ == "__main__":
    unittest.main()
