"""回归真实文本日志中的缺字段、混接、重复和截断，避免拼出虚假完整组。"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from chassis_text_parser import parse
from audit_chassis_text import audit


SAMPLE = """00> [STATUS] seq=20 t_us=4656972 fw=20261106 frame=12 mode=2 valid=0 exit=8 accel=1 wheel_mask=15 obs=15 dt_us=4981 imu_age_us=795 cmd_age_us=26 qdrop=0 tdrop=0
[WHEEL] seq=20 dps=0,0,0,0 req_dps=0,0,0,0 req_i=8,-10,-7,10 send_i=8,-10,-7,10 fb_i=4,-22,-16,4 vn_mmps=0,0 omega_mradps=0 omega_ref_mradps=0
[IMU] seq=20 corr=1 theta_mrad=-67 psi_bn_mrad=72 psi_gn_mrad=5 gyro_mradps=22,-873,-13 raw_a_mmps2=-5752,-152,7929 nav_a_mmps2=-876,17,-45 center_a_mmps2=133,9
[KF] seq=20 v_n_mmps=1,0 a_n_mmps2=22,12 user_g_mmps=0,0 ref_g_mmps=0,0 hat_g_mmps=1,0 err_g_mmps=-1,0 dv_g_mmps=0,0 cmd_g_mmps=0,0 kp_x1000=0 flags=0 sat=0
[YAW] seq=20 ref_mdeg=0 fb_mdeg=-306 out_raw=215
[DIR] seq=20 theta_out_mrad=-67 rel_omega_mradps=0 lead_mrad=0 rate_valid=1
"""


def frame19_sample():
    lines = SAMPLE.replace("frame=12", "frame=19").splitlines()
    lines = [line for line in lines if "[YAW]" not in line]
    lines[0] += " imu_seq=4812 chassis_mode=1 control_mode=0 output_enabled=1"
    lines[1] += " raw_dps=0,0,0,0 age_us=1,2,3,4 wheel_seq=100,101,102,103 scale_x1000=1000"
    lines[3] += " p_x1e6=10,20,30,40"
    lines.append("[OBS] seq=20 imu_t_us=4656100 clock_gen=1 recovering=0 good_ms=500 "
                 "tilt_mrad=0,0 chi_x1000=10 att_flags=3 dv_res_mmps=0,0 window_ms=250 "
                 "gate=1 rejects=0 dv_now_mmps=0,0 v_res_mmps=0,0 reject_reason=0 control_ready=0 "
                 "control_inhibit=128 control_good_ms=200 control_blend_x1000=0 "
                 "control_v_err_mmps=5 control_a_res_mmps2=30")
    return "\n".join(lines) + "\n"


class TextLogTests(unittest.TestCase):
    def test_frame19_core_does_not_require_slow_diagnostics(self):
        sample = frame19_sample()
        rows, _, report = parse(sample)
        self.assertEqual(len(rows), 1)
        self.assertNotIn("YAW", rows[0])
        self.assertNotIn("q_1e6", rows[0]["OBS"])
        self.assertEqual(report["complete_frames"], 1)
        for tag in ("STATUS", "WHEEL", "IMU", "KF", "DIR", "OBS"):
            damaged = "\n".join(line for line in sample.splitlines() if f"[{tag}]" not in line)
            with self.subTest(tag=tag):
                self.assertEqual(parse(damaged)[0], [])

    def test_frame19_requires_new_same_sample_fields(self):
        sample = frame19_sample()
        for token in (" imu_seq=4812", " chassis_mode=1", " control_mode=0", " output_enabled=1",
                      " raw_dps=0,0,0,0", " age_us=1,2,3,4", " wheel_seq=100,101,102,103",
                      " scale_x1000=1000", " p_x1e6=10,20,30,40"):
            with self.subTest(token=token):
                self.assertEqual(parse(sample.replace(token, ""))[0], [])
        self.assertEqual(parse(sample.replace("age_us=1,2,3,4", "age_us=1,2,3"))[0], [])
        self.assertEqual(parse(sample.replace("fw=20261106", "fw=123"))[0], [])

    def test_frame19_damaged_optional_diagnostics_do_not_lose_core_data(self):
        detail = "[DETAIL] seq=20 q_1e6=1000000,0,0,0 gravity_mmps2=0,0,9810 innov_x1000=0,0,0,0 r_x1e6=1,2,3,4\n"
        health = "[HEALTH] seq=20 t_us=4656972 read_t_us=4656100 attempt_seq=4813 accepted_seq=4812 " \
                 "status=0 startup=0 timeout=0 error=0 busy=0 not_ready=1 gaps=0\n"
        rows, _, report = parse(frame19_sample()+detail+health)
        self.assertEqual(len(rows), 1)
        self.assertIn("DETAIL", rows[0])
        self.assertIn("HEALTH", rows[0])
        self.assertEqual(report["diagnostic_only_sequences"], 0)
        cases = ((detail.replace("1,2,3,4", "1,2,3"), "DETAIL"),
                 (health.replace(" gaps=0", ""), "HEALTH"),
                 (detail+detail, "DETAIL"),
                 ("[YAW] seq=20 ref_mdeg=1 fb_mdeg=2 out_raw=3\n", "YAW"))
        for damaged, tag in cases:
            with self.subTest(damaged=damaged):
                rows, _, report = parse(frame19_sample()+damaged)
                self.assertEqual(len(rows), 1)
                self.assertNotIn(tag, rows[0])
                self.assertTrue(any(issue.get("tag") == tag for issue in report["issues"]))
        merged = detail.rstrip()+health
        rows, _, report = parse(frame19_sample()+merged)
        self.assertEqual(len(rows), 1)
        self.assertEqual(report["issues"][-1]["reason"], "merged_tags")
        lines = frame19_sample().splitlines()
        lines[0] += detail.strip()
        self.assertEqual(parse("\n".join(lines))[0], [])
        orphan = detail.replace("seq=20", "seq=40")
        self.assertEqual(parse(frame19_sample()+orphan)[2]["observed_sequences"], 1)
        self.assertEqual(parse(frame19_sample()+orphan)[2]["diagnostic_only_sequences"], 1)

    def test_legacy_corrupt_yaw_is_still_rejected(self):
        self.assertEqual(parse(SAMPLE.replace("fb_mdeg=-306", "fb_mdeg=bad"))[0], [])

    def test_audit_uses_recorded_signed_spin_target_and_optional_diagnostics(self):
        first = frame19_sample().replace("omega_mradps=0 omega_ref_mradps=0", "omega_mradps=6892 omega_ref_mradps=6892")
        second = first.replace("seq=20", "seq=40").replace("omega_mradps=6892 omega_ref_mradps=6892",
                                                                       "omega_mradps=-6892 omega_ref_mradps=-6892")
        disabled = first.replace("seq=20", "seq=60").replace("output_enabled=1", "output_enabled=0")
        following = first.replace("seq=20", "seq=80").replace("chassis_mode=1", "chassis_mode=3")
        force = first.replace("seq=20", "seq=100").replace("control_mode=0", "control_mode=1")
        wheel_missing = first.replace("seq=20", "seq=120").replace("wheel_mask=15", "wheel_mask=7")
        result = audit(first+second+disabled+following+force+wheel_missing)
        self.assertEqual(result["high_rotation_frames"], 3)
        self.assertEqual(result["steady_rotation_frames"], 2)
        self.assertEqual(result["diagnostic_frames"]["YAW"], 0)

    def test_frame17_control_permission_diagnostics_required(self):
        old = "[OBS] seq=20 imu_t_us=4656100 clock_gen=1 recovering=0 good_ms=500 q_1e6=1000000,0,0,0 gravity_mmps2=0,0,9810 tilt_mrad=0,0 chi_x1000=10 att_flags=3 innov_x1000=0,0,0,0 r_x1e6=2500,40000,2500,40000 dv_res_mmps=0,0 window_ms=20 gate=1 rejects=0 dv_now_mmps=0,0 v_res_mmps=0,0 reject_reason=0"
        extra = " control_ready=0 control_inhibit=128 control_good_ms=200 control_blend_x1000=0 control_v_err_mmps=5 control_a_res_mmps2=30"
        latest = SAMPLE.replace("frame=12", "frame=17")
        for prefix in (True, False):
            def joined(obs):
                return obs + "\n" + latest if prefix else latest + obs + "\n"
            self.assertEqual(len(parse(joined(old+extra))[0]), 1)
            for token in extra.split():
                self.assertEqual(len(parse(joined((old+extra).replace(token, "")))[0]), 0)
            self.assertEqual(len(parse(joined(old+extra.replace("control_ready=0", "control_ready=0,1")))[0]), 0)

    def test_frame15_guard_diagnostics_required_in_any_tag_order(self):
        observation = "[OBS] seq=20 imu_t_us=4656100 clock_gen=1 recovering=0 good_ms=500 q_1e6=1000000,0,0,0 gravity_mmps2=0,0,9810 tilt_mrad=0,0 chi_x1000=10 att_flags=3 innov_x1000=0,0,0,0 r_x1e6=2500,40000,2500,40000 dv_res_mmps=60,-33 window_ms=20 gate=2 rejects=1 dv_now_mmps=60,-33 v_res_mmps=100,10 reject_reason=4\n"
        latest = SAMPLE.replace("frame=12", "frame=15")
        for prefix in (True, False):
            def joined(obs):
                return obs + latest if prefix else latest + obs
            self.assertEqual(len(parse(joined(observation))[0]), 1)
            for bad in (" dv_now_mmps=60,-33", " v_res_mmps=100,10", " reject_reason=4"):
                self.assertEqual(len(parse(joined(observation.replace(bad, "")))[0]), 0)
            self.assertEqual(len(parse(joined(observation.replace("v_res_mmps=100,10", "v_res_mmps=100")))[0]), 0)

    def test_frame14_requires_complete_same_seq_observation(self):
        observation = "[OBS] seq=20 imu_t_us=4656100 clock_gen=1 recovering=1 good_ms=0 q_1e6=1000000,0,0,0 gravity_mmps2=0,0,9810 tilt_mrad=0,0 chi_x1000=10 att_flags=3 innov_x1000=0,0,0,0 r_x1e6=2500,40000,2500,40000 dv_res_mmps=60,-33 window_ms=200 gate=2 rejects=1\n"
        latest = SAMPLE.replace("frame=12", "frame=14")
        self.assertEqual(len(parse(latest + observation)[0]), 1)
        self.assertEqual(parse(latest)[2]["rejected_sequences"], [20])
        for broken in (observation.replace("seq=20", "seq=40"),
                       observation.replace("q_1e6=1000000,0,0,0", "q_1e6=1000000,0,0"),
                       observation.replace("gate=2", "gate=bad"), observation + observation):
            self.assertEqual(len(parse(latest + broken)[0]), 0)

    def test_complete_and_legacy(self):
        self.assertEqual(len(parse(SAMPLE)[0]), 1)
        legacy = SAMPLE.replace("frame=12", "frame=11").split("[DIR]")[0]
        self.assertEqual(audit(legacy)["complete_frames"], 1)

    def test_damaged_frame_cannot_contaminate_next_frame(self):
        lines = SAMPLE.splitlines()
        merged = "\n".join(lines[:-2] + [lines[-2] + "\xc4\x0f" + lines[0], lines[-1]])
        cases = [
            SAMPLE.replace("exit=8", "\xc5\xf0\xd2t=8"),
            SAMPLE.replace("nav_a_mmps2=-876,17,-45", "nav_a_mmps2=-876,17"),
            SAMPLE.replace("center_a_mmps2=133,9", "center_a_mmps2=133,"),
            SAMPLE + lines[0] + "\n",
            SAMPLE.split("[DIR]")[0],
            merged,
        ]
        for damaged in cases:
            with self.subTest(damaged=damaged):
                rows, _, result = parse(damaged + "\n" + SAMPLE.replace("seq=20", "seq=40"))
                self.assertEqual([r["STATUS"]["seq"] for r in rows], ["40"])
                self.assertEqual(result["rejected_sequences"], [20])


if __name__ == "__main__":
    unittest.main()
