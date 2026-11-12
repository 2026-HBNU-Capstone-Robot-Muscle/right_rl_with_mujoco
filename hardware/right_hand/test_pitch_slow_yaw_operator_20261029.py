"""Meaningful queue/guard/default-I/O checks; all device constructors blocked."""
import contextlib,copy,io,json,sys,unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pitch_slow_yaw_plan_20261029 as plan
import pitch_slow_yaw_gate_20261029 as gate
from retained_red_yaw_plan_20261029 import PATH as YAW_PATH,ORIGINAL
from pitch_handoff_plan_20261028 import PITCH_PHASE,PITCH_HOLD,TRANSFER_HOLD

class Checks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p=plan.PitchSlowYawPlan(plan.REQUEST);cls.request=json.loads(plan.REQUEST.read_bytes())
        cls.folder=plan.OUT/'unit_test_requests';cls.folder.mkdir(exist_ok=True)
    def request_file(self,label,change):
        r=copy.deepcopy(self.request);change(r);file=self.folder/(label+'.json');file.write_text(json.dumps(r),encoding='utf-8');return file
    def test_complete_queue_joins_once_and_is_bounded(self):
        rows=self.p.plan(self.p.source_position());values=np.vstack([self.p.source_position(),[q for p,q in rows]])
        self.assertLessEqual(np.max(abs(np.diff(values,axis=0))),2)
        self.assertLessEqual((len(rows)+40)*.075+20,480)
        self.assertGreater(len(rows)*.075,300)
    def test_only_yaw_timing_changes(self):
        old=np.load(ORIGINAL);new=np.load(YAW_PATH)
        indices=np.repeat(np.arange(len(old['goals'])),np.where(old['phases']=='yaw',2,1))
        np.testing.assert_array_equal(new['source_indices'],indices)
        np.testing.assert_array_equal(new['goals'],old['goals'][indices])
        self.assertEqual(np.count_nonzero(new['phases']=='yaw'),1276)
    def test_one_place_before_pitch_and_final_open(self):
        rows=self.p.plan(self.p.source_position());i=next(i for i,(p,q) in enumerate(rows) if p==PITCH_PHASE)
        np.testing.assert_array_equal(rows[i-1][1],plan.pitch_start())
        np.testing.assert_array_equal(rows[-1][1],np.load(YAW_PATH)['start'])
        self.assertEqual(self.p.CAPTURE_BEFORE_PHASE,PITCH_PHASE)
    def test_old_limits_remain_separate(self):
        from pitch_handoff_plan_20261028 import PitchHandoffPlan
        from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan
        self.assertEqual(PitchHandoffPlan.ON_TIMEOUT,300);self.assertEqual(HorizontalQuarterPlan.ON_TIMEOUT,300)
        self.assertEqual(self.p.ON_TIMEOUT,480)
        self.assertEqual(self.p.MAX_THUMB_TRAVEL,PitchHandoffPlan.MAX_THUMB_TRAVEL)
    def test_four_tick_noise_reuses_center(self):
        file=self.request_file('noise4',lambda r:r['off_state']['position'].__setitem__(1,r['off_state']['position'][1]+4))
        p=plan.PitchSlowYawPlan(file);self.assertTrue(p.review_center_reused)
        np.testing.assert_array_equal(p.source_position(),self.p.source_position());p.validate_start(p.raw_off_ticks)
    def test_five_ticks_need_fresh_center(self):
        file=self.request_file('noise5',lambda r:r['off_state']['position'].__setitem__(1,r['off_state']['position'][1]+5))
        p=plan.PitchSlowYawPlan(file);self.assertFalse(p.review_center_reused)
    def test_health_precedes_center_reuse(self):
        file=self.request_file('bad_health',lambda r:r['off_state']['torque'].__setitem__(0,1))
        with self.assertRaises(ValueError):plan.PitchSlowYawPlan(file)
    def test_enable_radius_not_widened(self):
        raw=self.p.source_position();raw[1]+=5
        with self.assertRaises(ValueError):self.p.validate_start(raw)
    def test_nominal_failure_rejected_even_when_accepted_flag_true(self):
        target=plan.OUT/'loaded_cube_evidence.json';original=gate.read
        def replaced(path):
            r=original(path)
            if path==target:
                r=copy.deepcopy(r)
                next(v for v in r['cases'] if (v['gain_scale'],v['friction'])==(1.,.3))['motion_health_passed']=False
            return r
        with patch.object(gate,'read',side_effect=replaced):
            with self.assertRaises(RuntimeError):gate.validate_loaded()
    def test_current_review_explicitly_exposes_broad_failures(self):
        r=gate.validate_review(self.p,plan.PROOF)
        self.assertTrue(r['loaded_evidence']['supervised_experiment_only'])
        self.assertFalse(r['loaded_evidence']['all27_parameter_conditions_passed'])
        self.assertFalse(r['physical_combined_rotation_verified'])
    def test_tampered_time_limit_review_rejected(self):
        r=json.loads(plan.PROOF.read_bytes());r['active_motion_limit_s']=900
        file=self.folder/'tampered_time.json';file.write_text(json.dumps(r),encoding='utf-8')
        with self.assertRaises(RuntimeError):gate.validate_review(self.p,file)
    def test_pitch_and_transfer_callbacks_do_not_collide(self):
        import leap_pitch_slow_yaw_20261029 as entry
        events=[];protocol=SimpleNamespace(pitch_frame=lambda *a:events.append('pitch'),transfer_frame=lambda *a:events.append('transfer'))
        with patch.object(entry,'precise_sample',return_value=({'position':np.zeros(16)},np.zeros(16))):
            bus=SimpleNamespace(grasp_protocol=protocol)
            entry.sample(bus,np.zeros(16),{},PITCH_HOLD);entry.sample(bus,np.zeros(16),{},TRANSFER_HOLD)
        self.assertEqual(events,['pitch','transfer'])
    def test_default_entry_never_opens_devices(self):
        import cv2,dynamixel_sdk
        import leap_pitch_slow_yaw_20261029 as entry
        with patch.object(sys,'argv',['leap_pitch_slow_yaw_20261029.py']),contextlib.redirect_stdout(io.StringIO()), \
             patch.object(cv2,'VideoCapture',side_effect=AssertionError('Camera opened')), \
             patch.object(dynamixel_sdk,'PortHandler',side_effect=AssertionError('Motor port opened')):
            self.assertEqual(entry.main(),0)

if __name__=='__main__':unittest.main(verbosity=2)
