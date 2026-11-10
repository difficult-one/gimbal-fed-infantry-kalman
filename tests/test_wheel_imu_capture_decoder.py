import hashlib
import json
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DECODER = ROOT / "tools" / "decode_wheel_imu_capture.py"


def finish_capture(payload):
    return payload + struct.pack("<I", zlib.crc32(payload))


def sample_capture():
    header = struct.pack(
        "<4sHH8I8H31fbbBB",
        b"WIMU", 1, 184, 0x20261110, 18, 0x20261111, 9, 0xFFFFFFF0, 30, 1, 0,
        2, 1, 1, 1, 1, 1, 1, 0,
        .076, .077, .078, .079, 19, 20, 21, 22, 1, -1, 1, -1,
        -1, 1, .25, 1, 1, .25, -1, -1, .25, 1, -1, .25,
        66, -107, 29, 12, 6.892705, -1, 120.5, -1, -1, 1, 0,
    )
    imu = struct.pack(
        "<4I6h4B22f",
        0xFFFFFFFC, 12, 90, 4, 7, -8, 9, -10, 11, -12, 0, 7, 33, 0,
        1, 2, 3, 4, 5, 6, 1, 0, 0, 0, 0, 0, 9.81, 11, 12, 13,
        21, 22, 23, 31, 32, 33,
    )
    failed_imu = struct.pack(
        "<4I6h4B22f",
        5, 12, 91, 4, 0, 0, 0, 0, 0, 0, 3, 0, 16, 4,
        *([float("nan")] * 22),
    )
    motors = b"".join(
        struct.pack("<3IhHf", 6 + i, 80 + i, 4, -101 + i, 512 + i, 505.5 + i)
        for i in range(5)
    )
    control = struct.pack(
        "<8I19fH6B",
        20, 100, 4, 12, 80, 81, 82, 83, *range(19),
        0x0102, 4, 12, 15, 5, 9, 2,
    )
    return finish_capture(header + imu + failed_imu + motors + control)


class CaptureDecoderTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.raw_path = self.directory / "test.wimu"

    def run_decoder(self, raw, *options):
        self.raw_path.write_bytes(raw)
        return subprocess.run(
            [sys.executable, "-B", str(DECODER), str(self.raw_path), *map(str, options)],
            capture_output=True, text=True, encoding="utf-8", check=False,
        )

    def test_decodes_signed_values_distinct_streams_and_wrapping_times(self):
        raw = sample_capture()
        result = self.run_decoder(raw)
        self.assertEqual(result.returncode, 0, result.stderr)
        capture = json.loads(result.stdout)
        metadata = capture["metadata"]
        self.assertEqual(metadata["fw"], "20261110")
        self.assertEqual(metadata["frame"], 18)
        self.assertEqual(metadata["config_id"], "20261111-" + hashlib.sha256(raw[56:184]).hexdigest())
        self.assertEqual(metadata["raw_sha256"], hashlib.sha256(raw).hexdigest())
        self.assertEqual(metadata["wheel_order"], ["LF", "RF", "LB", "RB"])
        self.assertEqual(metadata["gear_ratio"], [19, 20, 21, 22])
        self.assertAlmostEqual(metadata["wheel_radius_m"][3], .079)
        self.assertEqual(metadata["kinematics"][2], [-1, -1, .25])
        self.assertEqual(metadata["spin_ref_mradps"], 6892)
        self.assertEqual(metadata["coordinates"], "n_ins_x_neg_y_neg_z_legacy_y")
        self.assertEqual(capture["capture"]["start_us"], 0xFFFFFFF0)
        self.assertEqual(capture["capture"]["end_us"], 30)
        fresh, failed = capture["imu"]
        self.assertEqual(fresh["t_read_us"], 0xFFFFFFFC)
        self.assertEqual(fresh["accel_counts"], [7, -8, 9])
        self.assertEqual(fresh["gyro_counts"], [-10, 11, -12])
        self.assertEqual(fresh["specific_force_after_lever_mps2"], [11, 12, 13])
        self.assertEqual(fresh["center_n_mps2"], [31, 32, 33])
        self.assertEqual(failed["t_read_us"], 5)
        self.assertEqual(failed["accepted_seq"], 12)
        self.assertEqual(failed["attempt_seq"], 91)
        self.assertEqual(failed["read_status"], 3)
        self.assertEqual(failed["accel_mps2"], [None, None, None])
        self.assertEqual(capture["motors"]["LF"][0]["rpm"], -101)
        self.assertEqual(capture["motors"]["RB"][0]["t_receive_us"], 9)
        self.assertEqual(capture["motors"]["yaw"][0]["encoder_count"], 516)
        control = capture["control"][0]
        self.assertEqual(control["wheel_seq"], [80, 81, 82, 83])
        self.assertEqual(control["wheel_eff_dps"], [6, 7, 8, 9])
        self.assertEqual(control["dv_now_mps"], [15, 16])
        self.assertEqual(control["dv_res_mps"], [17, 18])
        self.assertEqual(control["control_inhibit"], 0x0102)
        self.assertEqual(control["flags"], 9)

    def test_exports_matching_metadata_and_firmware_hash(self):
        firmware_path = self.directory / "firmware.bin"
        firmware_path.write_bytes(b"known firmware artifact")
        output = self.directory / "decoded.json"
        metadata_path = self.directory / "test.metadata.json"
        result = self.run_decoder(
            sample_capture(), "--output", output, "--metadata-out", metadata_path,
            "--firmware-artifact", firmware_path,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        self.assertEqual(json.loads(output.read_text(encoding="utf-8"))["metadata"], metadata)
        self.assertEqual(metadata["firmware_sha256"], hashlib.sha256(firmware_path.read_bytes()).hexdigest())
        self.assertEqual(result.stdout, "")

    def test_refuses_invalid_version_count_length_and_crc(self):
        raw = sample_capture()
        invalid = [
            (raw[:40], "truncated"),
            (raw[:-1], "length"),
            (raw + b"x", "length"),
            (raw[:-4] + bytes(4), "CRC32"),
        ]
        for offset, replacement, expected in (
            (0, b"BAD!", "magic"),
            (4, struct.pack("<H", 2), "version"),
            (6, struct.pack("<H", 180), "header size"),
            (40, struct.pack("<H", 369), "count"),
            (54, struct.pack("<H", 1), "reserved"),
            (56, struct.pack("<f", 0.0), "radius"),
        ):
            payload = raw[:-4]
            invalid.append((finish_capture(payload[:offset] + replacement + payload[offset + len(replacement):]),
                            expected))
        output = self.directory / "decoded.json"
        for bad_raw, expected in invalid:
            with self.subTest(expected=expected):
                result = self.run_decoder(bad_raw, "--output", output)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(expected, result.stderr)
                self.assertFalse(output.exists())

    def test_decodes_capture_frozen_when_leaving_shadow(self):
        raw = bytearray(sample_capture()[:-4])
        struct.pack_into("<I", raw, 32, 4)
        result = self.run_decoder(finish_capture(raw))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["capture"]["stop_reason"], 4)

    def test_spin_reference_matches_firmware_float_multiplication(self):
        raw = bytearray(sample_capture()[:-4])
        struct.pack_into("<f", raw, 168, 6.575)
        result = self.run_decoder(finish_capture(raw))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["metadata"]["spin_ref_mradps"], 6575)

    def test_refuses_to_overwrite_raw_capture(self):
        raw = sample_capture()
        result = self.run_decoder(raw, "--output", self.raw_path)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("distinct", result.stderr)
        self.assertEqual(self.raw_path.read_bytes(), raw)


if __name__ == "__main__":
    unittest.main()
