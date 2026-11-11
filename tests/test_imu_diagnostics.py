"""低频日志必须保留一次短暂门槛失败，并识别完整/截断诊断。"""
import runpy
import re
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[1]
module = runpy.run_path(str(root / "tools/analyze_imu_diagnostics.py"))
decode = module["decode"]
hits = [0] * 31


def aid(win, counters, now=0, last=0):
    return (f"00> [AID] win={win} dver=1 t_us={win*1000000} now=0x{now:08x} last=0x{last:08x} "
            f"last_t_us=5000 resets=1 settle_ms=100 window_ms=0 hits=" + ",".join(map(str, counters)))


first = aid(1, hits)
hits[6] = 1
second = aid(2, hits, last=1 << 6)
rows = decode(first + "\n" + second)["aid"]
assert rows[0]["hits_delta"] is None
assert rows[1]["now_names"] == []
assert rows[1]["hits_delta"] == {"车轮原始速度超限": 1}
assert rows[1]["last_names"] == ["车轮原始速度超限"]
assert "车轮原始速度超限: 1" in module["describe"]({"rigid": [], "aid": rows})
hits[6] = 0xFFFFFFFF
wrapped = decode(aid(3, hits) + "\n" + aid(4, [0]*31))["aid"]
assert wrapped[-1]["hits_delta"] == {"车轮原始速度超限": 1}
try:
    decode(second.rsplit(",", 1)[0])
    raise AssertionError("truncated counters accepted")
except ValueError:
    pass
assert decode("00> [STATUS] seq=20 fw=20261028") == {"rigid": [], "aid": [], "clock": []}
clock = "[CLOCK] dver=1 t_us=9000000 hal_ms=9100 cyccnt=123 demcr=0 ctrl=0 gen=1 flags=7 last_t_us=8999000"
assert len(decode(clock)["clock"][0]["flag_names"]) == 3
try:
    decode(clock.replace(" ctrl=0", ""))
    raise AssertionError("incomplete clock record accepted")
except ValueError:
    pass

# 使用 MCU 的实际整数格式化代码，在主机模拟 RTT 满、同窗重读与漏窗。
# 包含最大计数/饱和值，检查整个包可容纳且不会输出截断文本。
source = (root / "application/chassis/chassis_log.c").read_text(encoding="utf-8")
helpers = source.split("static int32_t MotionLogScaled", 1)[1].split("#if WHEEL_IMU_CAPTURE", 1)[0]
prefix = r'''
#include "imu_compensation.h"
#include "bsp_dwt.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "config.h"
static IMURigidStats r;
static IMUYawBiasDebug a;
static char packet[2560];
void DWT_GetClockDebug(DWT_ClockDebug *out) { memset(out,0,sizeof(*out)); }
uint64_t DWT_GetTimeline_us(void) { return 1000000; }
static unsigned calls, fail;
static void INS_GetCompensationDebug(IMURigidStats *x, IMUYawBiasDebug *y) { *x=r; *y=a; }
static unsigned SEGGER_RTT_Write(unsigned channel, const char *text, unsigned size) {
    assert(channel==0 && size<sizeof(packet)); calls++;
    if (fail) return 0;
    memcpy(packet,text,size); packet[size]=0; return size;
}
'''
main = r'''
int main(void) {
    r.seq=1; r.start_us=UINT32_MAX; r.end_us=UINT32_MAX;
    r.samples=r.good=UINT32_MAX; r.dt_min_us=r.dt_max_us=UINT32_MAX;
    for(unsigned i=0;i<3;++i) {
        r.alpha2[i]=r.alpha_peak[i]=r.tangent2[i]=r.centripetal2[i]=1e30f;
        r.tangent_peak[i]=r.centripetal_peak[i]=1e30f;
        if(i<2) r.before2[i]=r.after2[i]=r.before_sum[i]=r.after_sum[i]=r.correction2[i]=r.cross[i]=1e30f;
    }
    for(unsigned i=0;i<IMU_AID_CAUSE_COUNT;++i) a.hits[i]=UINT32_MAX;
    a.time_us=a.last_block_us=a.resets=UINT32_MAX;
    a.reason=a.last_reason=0x7fffffffu; a.settle_s=a.window_s=1e20f;
    MotionDiagnosticLog(); assert(calls==1);
    assert(strstr(packet,"[RIGID] win=1") && strstr(packet,"[AID] win=1"));
    assert(strlen(packet)>1000 && !strcmp(packet+strlen(packet)-2,"\r\n"));
    MotionDiagnosticLog(); assert(calls==1);
    r.seq=2; fail=1; MotionDiagnosticLog(); assert(calls==2);
    MotionDiagnosticLog(); assert(calls==2); /* 不因 RTT 满忙等/重发同一窗。 */
    r.seq=3; fail=0; MotionDiagnosticLog(); assert(strstr(packet,"ddrop=1"));
    r.seq=5; MotionDiagnosticLog(); assert(strstr(packet,"ddrop=2"));
    puts(packet);
    return 0;
}
'''
test_dir = root / "build/host_motion"
test_dir.mkdir(parents=True, exist_ok=True)
test_c = test_dir / "imu_diagnostic_log.c"
test_exe = test_dir / "imu_diagnostic_log.exe"
test_c.write_text(prefix + "static int32_t MotionLogScaled" + helpers + main, encoding="utf-8")
subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
                "-Itests/host/dwt", "-Ibsp/dwt", "-Itests/host", "-Itests/host/can", "-Iconfig", "-Imodules/chassis_motion",
    "-Imodules/algorithm", "-Imodules/imu", "-Imodules/motor/dji", "-Imodules/motor",
    "-Imodules/daemon", "-Imodules/jam_detector", "-Imodules/diagnostics", "config/config.c", str(test_c), "-lm", "-o", str(test_exe)], cwd=root, check=True)
packet = subprocess.run([str(test_exe)], cwd=root, check=True, capture_output=True, text=True).stdout
decoded = decode(packet)
assert len(decoded["clock"]) == 1
assert len(decoded["aid"]) == len(decoded["rigid"]) == 1
assert len(decoded["aid"][0]["hits"]) == 31
assert decoded["rigid"][0]["ddrop"] == 2
assert decoded["rigid"][0]["win"] == 5
print("PASS: transient reasons, counter wrap, actual MCU formatter bounds, RTT skip/drop and decoder")

# Compile the actual seven-line formatter as well. Unrelated yaw hardware is
# stubbed; production state/OBS types, formatting and strict parser are used.
task = "void ChassisMotionLogTask(void)" + source.split("void ChassisMotionLogTask(void)", 1)[1]
extra = r'''
#include "chassis_motion.h"
#include "ins.h"
typedef struct { uint32_t time_us; unsigned source,rc_online,gimbal_mode;
    int rc_yaw_raw,rc_yaw_used; float yaw_ref,yaw_feedback,gyro_z; } RobotCMDYawDebug;
typedef struct { uint32_t control_time_us; float speed_p_raw,speed_i_raw,output_raw,sent_raw; } GimbalYawFeedback;
static unsigned motion_log_ready=1, motion_transport_drop, ready=1, pattern;
static void RobotCMDGetYawDebug(RobotCMDYawDebug *x) { memset(x,0,sizeof(*x)); }
void INS_GetCalibrationDebug(INS_CalibrationDebug *x) { memset(x,0,sizeof(*x)); }
static void GimbalGetYawFeedback(GimbalYawFeedback *x) { memset(x,0,sizeof(*x)); }
uint8_t ChassisMotionPopDiagnosticLog(ChassisMotionState *s,ChassisMotionDirectionState *d,
                                     ChassisMotionObservationState *o) {
    if (!ready) return 0;
    ready=0;
    memset(s,pattern ? 0x7f : 0,sizeof(*s)); memset(d,pattern ? 0x7f : 0,sizeof(*d));
    memset(o,pattern ? 0x7f : 0,sizeof(*o));
    s->seq=pattern ? 40 : 20; s->version=@CONFIG_VERSION@; s->mode=MC_SHADOW;
    if (!pattern) {
        s->imu_q[0]=.125f; o->gravity_b[2]=9.81f; o->attitude_chi_ratio=.123f;
        o->clock_generation=3; o->imu_recovering=1; o->gate=2;
        o->r_diag[0]=.0025f; s->innovation[0]=.1f;
        o->delta_v_partial[0]=.06f; o->delta_v_partial[1]=-.033f;
        o->velocity_residual[0]=.10f; o->velocity_residual[1]=.01f;
        o->reject_reason=MC_REJECT_VELOCITY;
        o->control_ready=0; o->control_inhibit=MC_CONTROL_VELOCITY|MC_CONTROL_ACCEL;
        o->control_good_s=.4f; o->control_blend=.25f;
        o->control_velocity_error=.029f; o->control_accel_residual=.12f;
    }
    return 1;
}
'''
log_main = r'''
int main(void) {
    r.seq=0; /* No independent RIGID packet may overwrite this test's payload. */
    ChassisMotionLogTask();
    assert(calls==1 && !motion_transport_drop && strstr(packet,"[OBS] seq=20"));
    puts(packet);
    pattern=1; ready=1;
    ChassisMotionLogTask();
    assert(calls==2 && !motion_transport_drop && strlen(packet)<3072);
    puts(packet);
    return 0;
}
'''
revision = re.search(r"^#define MC_FRAME_REVISION (\d+u)",
                     (root / "config/config.c").read_text(encoding="utf-8"), re.M)[1]
config_version = re.search(r"^#define MC_CONFIG_VERSION (0x[0-9a-fA-F]+u)",
                          (root / "config/config.c").read_text(encoding="utf-8"), re.M)[1]
extra = extra.replace("@CONFIG_VERSION@", config_version)
test_c = test_dir / "motion_observation_log.c"
test_exe = test_dir / "motion_observation_log.exe"
test_c.write_text(prefix.replace("packet[2560]", "packet[3072]") + extra +
                  "static int32_t MotionLogScaled" + helpers + task + log_main, encoding="utf-8")
subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
                "-Itests/host/dwt", "-Ibsp/dwt", "-Itests/host", "-Itests/host/can", "-Iconfig", "-Imodules/chassis_motion",
    "-Imodules/algorithm", "-Imodules/imu", "-Imodules/motor/dji", "-Imodules/motor",
    "-Imodules/daemon", "-Imodules/jam_detector", "-Imodules/diagnostics",
                "config/config.c", str(test_c), "-lm", "-o", str(test_exe)], cwd=root, check=True)
packet = subprocess.check_output([str(test_exe)], cwd=root, text=True)
parser = runpy.run_path(str(root / "tools/chassis_text_parser.py"))
frames, _, metadata = parser["parse"](packet)
assert metadata["complete_frames"] == 2 and not metadata["incomplete_frames"]
assert frames[0]["STATUS"]["frame"] == revision.rstrip("u")
assert frames[0]["STATUS"]["fw"] == f"{int(config_version.rstrip('u'), 0):08x}"
assert frames[0]["OBS"]["chi_x1000"] == "123"
assert frames[0]["OBS"]["q_1e6"] == "125000,0,0,0"
assert frames[0]["OBS"]["r_x1e6"] == "2500,0,0,0"
assert frames[0]["OBS"]["recovering"] == "1" and frames[0]["OBS"]["gate"] == "2"
assert frames[0]["OBS"]["dv_now_mmps"] == "60,-33"
assert frames[0]["OBS"]["v_res_mmps"] == "100,10"
assert frames[0]["OBS"]["reject_reason"] == "4"
assert frames[0]["OBS"]["control_ready"] == "0"
assert frames[0]["OBS"]["control_inhibit"] == "12"
assert frames[0]["OBS"]["control_good_ms"] == "400"
assert frames[0]["OBS"]["control_blend_x1000"] == "250"
assert frames[0]["OBS"]["control_v_err_mmps"] == "29"
assert frames[0]["OBS"]["control_a_res_mmps2"] == "120"
print(f"PASS: actual frame{revision.rstrip('u')} formatter, OBS fields, bounds and strict seven-line parsing")
