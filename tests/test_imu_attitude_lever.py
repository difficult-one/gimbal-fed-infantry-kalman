"""实际 INS/EKF 的处理顺序回归；硬件替身不复制被测算法。"""
import json
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[1]
output = root / "build/host_motion"
output.mkdir(parents=True, exist_ok=True)
source = (root / "modules/imu/ins.c").read_text(encoding="utf-8")
task = "void INS_Task(void)" + source.split("void INS_Task(void)", 1)[1].split("/**", 1)[0]
transforms = "void BodyFrameToEarthFrame(" + source.rsplit("void BodyFrameToEarthFrame(", 1)[1]
initial = "static uint8_t InitQuaternion(" + source.split("static uint8_t InitQuaternion(", 1)[1].split("/**", 1)[0]
calibration_signature = "static void mpu_offset_call(void)\n{"
calibration = calibration_signature + source.split(calibration_signature, 1)[1].split("void INS_Task", 1)[0]
# 导入生产任务、启动标定和坐标变换，SPI/时钟/中断/温控由 C 用例替换。
(output / "ins_task_under_test.h").write_text(task + "\n" + initial + "\n" + calibration + "\n" + transforms, encoding="utf-8")
exe = output / "imu_attitude_lever.exe"
subprocess.run([
    "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", "-fno-strict-aliasing",
    "-Itests/host/dwt", "-Ibuild/host_motion", "-Ibsp/dwt",
    "-Itests/host", "-Itests/host/can", "-Iconfig", "-Imodules/chassis_motion",
    "-Imodules/algorithm", "-Imodules/imu", "-Imodules/motor/dji", "-Imodules/motor",
    "-Imodules/daemon", "-Imodules/jam_detector", "-Imodules/diagnostics",
    "tests/imu_attitude_lever_test.c", "config/config.c", "tests/host/matrix.c", "modules/algorithm/kalman_filter.c",
    "modules/algorithm/quaternion_ekf.c", "modules/imu/imu_compensation.c", "-lm", "-o", str(exe),
], cwd=root, check=True)
subprocess.run([str(exe), "0", "5"], cwd=root, check=True)
subprocess.run([str(exe), "0", "6"], cwd=root, check=True)
subprocess.run([str(exe), "0", "7"], cwd=root, check=True)
subprocess.run([str(exe), "0", "8"], cwd=root, check=True)
results = []
for scenario, name in enumerate(("slow_yaw", "steady_yaw", "yaw_pitch", "translation", "gyro_noise")):
    pair = []
    for legacy in (0, 1):
        line = subprocess.check_output([str(exe), str(legacy), str(scenario)], cwd=root, text=True)
        _, _, before, after = line.split()
        pair.append({"legacy": bool(legacy), "before": float(before), "after": float(after)})
    new, old = pair
    if scenario == 0:
        assert old["after"] > old["before"] * 1.2, (name, pair)
    if scenario < 3:
        # 同一已归一化姿态链中，仅比较观测顺序；耦合运动允许差分截断误差。
        assert new["after"] < old["after"] * .25, (name, pair)
        assert new["after"] < .001, (name, pair)
    elif scenario == 3:
        assert abs(new["after"] - old["after"]) < .0001, pair
        assert new["after"] > .04, "real translation disappeared"
    else:
        # 这次修复针对重力观测顺序，不宣称消除陀螺差分噪声。
        assert new["after"] <= old["after"] * 1.05, pair
    results.append({"scenario": name, "results": pair})
    print(f"PASS: actual INS/EKF {name}: legacy={old['after']:.7f}, corrected={new['after']:.7f} m/s^2")
(output / "imu_attitude_lever_results.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
