"""Fault injection against the production MPU6500 SPI transport."""
import subprocess
import tempfile
import unittest
from pathlib import Path


class MpuSpiStatusTest(unittest.TestCase):
    def test_transport_status_and_recovery(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            exe = Path(tmp) / "mpu_spi_status.exe"
            subprocess.run([
                "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
                "-Itests/host/mpu_spi", "-Itests/host/dwt", "-Ibsp/dwt", "-Imodules/mpu6500",
                "tests/mpu_spi_status_test.c", "modules/mpu6500/mpu6500driver.c",
                "-o", str(exe),
            ], cwd=root, check=True)
            subprocess.run([str(exe)], cwd=tmp, check=True)


if __name__ == "__main__":
    unittest.main()
