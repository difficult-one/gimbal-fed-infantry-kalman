"""Compile the capture producer with host I/O stubs and verify its wire output."""
import runpy
import struct
import subprocess
import tempfile
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class CaptureFirmwareTest(unittest.TestCase):
    def test_native_records_and_bounded_transport(self):
        with tempfile.TemporaryDirectory() as tmp:
            exe = Path(tmp) / "wheel_imu_capture_test.exe"
            fixture = Path(tmp) / "capture.wimu"
            subprocess.run([
                "gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-DWHEEL_IMU_CAPTURE=1",
                "-Itests/host/dwt", "-Ibsp/dwt", "-Iconfig", "-Imodules/diagnostics", "-Imodules/chassis_motion",
                "-IMiddlewares/Third_Party/SEGGER/RTT",
                "tests/wheel_imu_capture_test.c", "modules/diagnostics/wheel_imu_capture.c",
                "-lm", "-o", str(exe),
            ], cwd=ROOT, check=True)
            subprocess.run([str(exe), str(fixture)], check=True)
            data = fixture.read_bytes()
        self.assertEqual(struct.unpack_from("<I", data, len(data) - 4)[0], zlib.crc32(data[:-4]))
        self.assertEqual(struct.unpack_from("<4I6h4B", data, 184),
                         (30, 8, 10, 7, 2, -3, 4, -1, 0, 6, 0, 7, 0, 0))
        self.assertEqual(struct.unpack_from("<3IhHf", data, 424),
                         (90, 0xFFFFFFFF, 7, -50, 370, -296.5))
        decode = runpy.run_path(str(ROOT / "tools/decode_wheel_imu_capture.py"))["decode_capture"]
        capture = decode(data)
        self.assertEqual(capture["metadata"]["spin_ref_mradps"], 6892)
        self.assertEqual(capture["imu"][1]["read_status"], 3)
        self.assertEqual(capture["motors"]["LF"][1]["seq"], 0)
        self.assertEqual(capture["control"][0]["wheel_eff_dps"], [123, 234, 345, 456])


if __name__ == "__main__":
    unittest.main()
