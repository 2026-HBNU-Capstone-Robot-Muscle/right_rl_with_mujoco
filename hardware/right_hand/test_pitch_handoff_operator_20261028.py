"""Queue, bounded-cache, capture dispatch and no-device entry regressions."""
import contextlib,copy,io,json,sys,unittest
from unittest.mock import patch
import numpy as np
import pitch_handoff_plan_20261028 as plan
import pitch_handoff_gate_20261028 as gate

class OperatorChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p=plan.PitchHandoffPlan(plan.REQUEST)
        cls.request=json.loads(plan.REQUEST.read_bytes())
        cls.folder=plan.OUT/'unit_test_requests';cls.folder.mkdir(exist_ok=True)
    def request_file(self,name,change):
        r=copy.deepcopy(self.request);change(r)
        path=self.folder/(name+'.json');path.write_text(json.dumps(r),encoding='utf-8');return path
    def test_recorded_pitch_has_only_two_scoped_differences(self):
        old=np.load(plan.PITCH);new=np.asarray([q for _,q in plan.loaded_rows()])[:len(old['goals'])]
        difference=new-old['goals'];expected=np.zeros_like(difference);expected[:,5]=-8;expected[:,13]=8
        np.testing.assert_array_equal(difference,expected)
    def test_queue_is_bounded_and_has_no_yaw(self):
        rows=self.p.plan(self.p.source_position());values=np.vstack([self.p.source_position(),[q for _,q in rows]])
        self.assertLessEqual(np.max(abs(np.diff(values,axis=0))),2)
        self.assertLessEqual((len(rows)+40)*.075+20,300)
        self.assertFalse(any('horizontal_yaw' in p for p,_ in rows))
    def test_place_checkpoint_precedes_known_pitch(self):
        rows=self.p.plan(self.p.source_position());at=next(i for i,(p,q) in enumerate(rows) if p==self.p.CAPTURE_BEFORE_PHASE)
        np.testing.assert_array_equal(rows[at-1][1],plan.pitch_start())
        self.assertEqual(rows[at][0],plan.PITCH_PHASE)
    def test_final_pose_is_existing_open_goal(self):
        np.testing.assert_array_equal(self.p.plan(self.p.source_position())[-1][1],np.load(plan.OPEN_PATH)['open_goal'])
    def test_exact_reference_is_not_recentered(self):
        self.assertFalse(self.p.review_center_reused)
    def test_one_tick_noise_retains_review_center(self):
        path=self.request_file('noise1',lambda r:r['off_state']['position'].__setitem__(1,r['off_state']['position'][1]+1))
        p=plan.PitchHandoffPlan(path);self.assertTrue(p.review_center_reused)
        np.testing.assert_array_equal(p.source_position(),self.p.source_position())
        self.assertEqual(p.raw_off_ticks[1],self.p.source_position()[1]+1)
    def test_four_tick_boundary_is_existing_radius(self):
        path=self.request_file('noise4',lambda r:r['off_state']['position'].__setitem__(1,r['off_state']['position'][1]+4))
        p=plan.PitchHandoffPlan(path);self.assertTrue(p.review_center_reused)
        p.validate_start(p.raw_off_ticks)
    def test_five_ticks_requires_fresh_screen_center(self):
        path=self.request_file('noise5',lambda r:r['off_state']['position'].__setitem__(1,r['off_state']['position'][1]+5))
        p=plan.PitchHandoffPlan(path);self.assertFalse(p.review_center_reused)
        self.assertEqual(p.source_position()[1],self.p.source_position()[1]+5)
    def test_health_not_hidden_by_center_cache(self):
        path=self.request_file('torque_on',lambda r:r['off_state']['torque'].__setitem__(0,1))
        with self.assertRaises(ValueError):plan.PitchHandoffPlan(path)
    def test_outside_enable_radius_rejected(self):
        raw=self.p.source_position();raw[1]+=5
        with self.assertRaises(ValueError):self.p.validate_start(raw)
    def test_new_hold_callbacks_dispatch_once(self):
        import leap_pitch_handoff_20261028 as entry
        from types import SimpleNamespace
        events=[];protocol=SimpleNamespace(hold_frame=lambda *a:events.append('pitch'),rotation_frame=lambda *a:events.append('transfer'))
        bus=SimpleNamespace(grasp_protocol=protocol)
        with patch.object(entry,'precise_sample',return_value=({'position':np.zeros(16)},np.zeros(16))):
            entry.sample(bus,np.zeros(16),{},plan.PITCH_PHASE)
            entry.sample(bus,np.zeros(16),{},plan.PITCH_HOLD)
            entry.sample(bus,np.zeros(16),{},plan.TRANSFER_HOLD)
        self.assertEqual(events,['pitch','transfer'])
    def test_tampered_next_turn_proof_rejected(self):
        r=json.loads(plan.PROOF.read_bytes());r['next_yaw_queued']=True
        path=self.folder/'tampered_next_turn.json';path.write_text(json.dumps(r),encoding='utf-8')
        with self.assertRaises(RuntimeError):gate.validate_review(self.p,path)
    def test_default_entry_opens_no_devices(self):
        import cv2,dynamixel_sdk
        import leap_pitch_handoff_20261028 as entry
        with patch.object(sys,'argv',['leap_pitch_handoff_20261028.py']),contextlib.redirect_stdout(io.StringIO()), \
             patch.object(cv2,'VideoCapture',side_effect=AssertionError('Camera opened')), \
             patch.object(dynamixel_sdk,'PortHandler',side_effect=AssertionError('Motor port opened')):
            self.assertEqual(entry.main(),0)

if __name__=='__main__':unittest.main(verbosity=2)
