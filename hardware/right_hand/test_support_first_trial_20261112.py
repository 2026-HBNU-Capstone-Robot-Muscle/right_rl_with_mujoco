"""Source, safety-contract and user-start integration tests without hardware."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import support_first_trial_plan_20261112 as plan
import support_first_trial_gate_20261112 as gate
import leap_cycle_support_first_transfer_20261112 as entry
from support_first_trial_capture_20261112 import SupportFirstTrialCapture,NOTICES

class Tests(unittest.TestCase):
    def test_first_route_and_phase_contract(self):
        z=plan.candidate();first=np.load(plan.ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz',allow_pickle=False)
        self.assertEqual(len(z['goals']),2868);self.assertTrue(np.array_equal(z['goals'][:1417],first['goals']))
        self.assertEqual(list(dict.fromkeys(z['phases'][1417:].tolist())),list(NOTICES))
        for phase in plan.HOLDS:self.assertEqual(np.count_nonzero(z['phases']==phase),54)
    def test_native_transport_and_finite_models_bound(self):
        r=gate.validate_review(plan.SupportFirstTrialPlan(plan.REQUEST),plan.PROOF)
        self.assertEqual((r['current_ma'],r['goal_step_ticks'],r['sample_period_s'],r['hardware_tracking_stop_rad']),(450,2,.075,.18))
        self.assertEqual(r['active_motion_limit_s'],500);self.assertTrue(r['existing500s_approval_retained'])
        self.assertEqual(r['support_first_models']['loaded_finite_cases'],6);self.assertEqual(r['support_first_models']['hand_only_cases'],3)
        for flag in ('physical_transfer_success','physical_upper_clearance_verified','full_task_success_certified','final_left_alignment_verified','second_rotation_included','all_condition_safety_certified'):
            self.assertFalse(r[flag])
    def test_source_tamper_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'candidate.npz';z=plan.candidate();z['goals'][1700,7]+=1
            np.savez_compressed(path,**z)
            with patch.object(plan,'CANDIDATE',path),self.assertRaises(RuntimeError):plan.candidate()
    def test_old_v3_review_cannot_approve_new_route(self):
        old=plan.ROOT/'right_hand_rotation_20261112/nominal_cycle_trial_preparation/review.json'
        with self.assertRaises(RuntimeError):gate.validate_review(plan.SupportFirstTrialPlan(plan.REQUEST),old)
    def test_default_and_mocked_start_no_devices_or_new_approval_prompt(self):
        with patch('dynamixel_sdk.PortHandler',side_effect=AssertionError('motor access')) as port, \
             patch('camera_stream.CameraStream',side_effect=AssertionError('camera access')) as camera, \
             patch('builtins.input',side_effect=AssertionError('repeat scope prompt')),contextlib.redirect_stdout(io.StringIO()) as log:
            with patch('sys.argv',['trial']):self.assertEqual(entry.main(),0)
            self.assertIn('CHECK ONLY',log.getvalue())
            with patch('sys.argv',['trial','--execute']),patch.object(entry.runtime,'main',return_value=0) as start:
                self.assertEqual(entry.main(),0);self.assertEqual(start.call_count,1)
            self.assertEqual(port.call_count,0);self.assertEqual(camera.call_count,0)
    def test_missing_phases_or_pending_goals_cannot_finish(self):
        c=SupportFirstTrialCapture.__new__(SupportFirstTrialCapture);q=plan.candidate()['goals'][-1]
        for report,pending in (({'cube_placement_confirmed':True},[]),
            ({'cube_placement_confirmed':True,'support_first_phases_seen':list(NOTICES)},[1])):
            with self.assertRaises(RuntimeError):c.finish(SimpleNamespace(pending=pending),q,report)
    def test_finish_keeps_support_until_user_SPACE_and_does_not_claim_success(self):
        c=SupportFirstTrialCapture.__new__(SupportFirstTrialCapture);c.guard=object();c._observer=lambda *args:object()
        c._photo=lambda *args:(None,{'image':'mock'});wait=[]
        def space(name,observe):wait.append(name);return {'position':np.arange(16)}
        c._space=space;r={'cube_placement_confirmed':True,'support_first_phases_seen':list(NOTICES)}
        with contextlib.redirect_stdout(io.StringIO()):c.finish(SimpleNamespace(pending=[]),plan.candidate()['goals'][-1],r)
        self.assertEqual(wait,['support_cube_for_off']);self.assertTrue(r['cube_support_confirmed_before_off'])
        self.assertFalse(r['physical_transfer_success']);self.assertFalse(r['physical_upper_clearance_verified']);self.assertFalse(r['cube_removal_confirmed'])
    def test_stage_notice_adds_no_photo_or_motor_write(self):
        c=SupportFirstTrialCapture.__new__(SupportFirstTrialCapture)
        c._photo=lambda *args:(_ for _ in ()).throw(AssertionError('blocking phase photo'))
        bus=SimpleNamespace(write=lambda *args:(_ for _ in ()).throw(AssertionError('motor write')))
        r={}
        with contextlib.redirect_stdout(io.StringIO()):
            for phase in NOTICES:c.transfer_frame(bus,None,r,phase)
        self.assertEqual(r['support_first_phases_seen'],list(NOTICES))
    def test_frozen_execution_sources_unchanged(self):
        frozen=json.loads((plan.ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json').read_bytes())['code_hashes']
        self.assertEqual(len(frozen),170)
        for name,digest in frozen.items():self.assertEqual(plan.sha(plan.ROOT/name),digest,name)

if __name__=='__main__':
    r=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
    report=dict(tests=r.testsRun,failures=len(r.failures),errors=len(r.errors),passed=r.wasSuccessful(),devices_opened=0,assistant_motor_writes=0)
    (plan.OUT/'software_verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report));raise SystemExit(not r.wasSuccessful())
