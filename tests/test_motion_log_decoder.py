"""校验 MCU 日志的 C/Python 布局，以及噪声前缀、坏长度、截断和连续帧。"""
import runpy
import struct
from pathlib import Path

root = Path(__file__).resolve().parents[1]
decoder = runpy.run_path(str(root / "tools/decode_chassis_motion.py"))
for variant in (0, 1):
    data = (root / f"build/host_motion/sample_{variant}.bin").read_bytes()
    assert len(data) == decoder["HEADER"].size + decoder["PAYLOAD"].size
    rows = list(decoder["decode"](b"noiseMCL1\0\0\0\0\0\0\0\0" + data + data + data[:20]))
    assert len(rows) == 2
    row = dict(zip(["transport_drop", *decoder["COLUMNS"]], rows[0]))
    assert row["version"] == decoder["VERSION"]
    assert row["seq"] == 123456 and row["transport_drop"] == 7
    assert row["imu_q_2"] == 0.125 and row["kp"] == 0.0625
    assert row["current_saturated_mask"] == 9
    # 原保留字节用于新增有效位，旧配置版本的相同布局也须保持兼容。
    for version in (0x20261016, 0x20261026, 0x20261027, 0x20261028, 0x20261029, 0x20261030, 0x20261031, 0x20261105, 0x20261106, 0x20261107, 0x20261108):
        old = bytearray(data)
        struct.pack_into("<I", old, decoder["HEADER"].size, version)
        assert len(list(decoder["decode"](old))) == 1
print("PASS: C/Python log layout, field offsets, stream resync, truncated frames")
