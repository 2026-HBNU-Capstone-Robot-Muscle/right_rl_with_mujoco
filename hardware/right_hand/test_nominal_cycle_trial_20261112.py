"""Meaningful operator authorization, queue and source-boundary checks."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import nominal_cycle_plan_20261112 as plan
import nominal_cycle_gate_20261112 as gate
from nominal_cycle_capture_20261112 import NominalCycleCapture
import leap_cycle_nominal_two_turns_20261112 as entry

class Tests(unittest.TestCase):
    def test_source_and_phases(self):
        z=plan.candidate(); self.assertEqual(len(z['goals']),5089)
        self.assertEqual(set(z['phases']),{plan.FIRST,plan.FIRST_HOLD,plan.SECOND,plan.FINAL_HOLD})
        self.assertTrue(np.array_equal(z['goals'],np.load(plan.SOURCE,allow_pickle=False)['goals']))
    def test_no_scope_approval_was_granted(self): self.assertFalse(plan.scope_approved())
    def test_approval_must_bind_candidate_and_scope(self):
        with tempfile.TemporaryDirectory() as folder:
            f=Path(folder)/'approval.json'
            for value in ({'user_explicitly_approved':True},
                          {'user_explicitly_approved':True,'maximum_on_seconds':500,'source_goals_sha256':'wrong','acknowledges_nominal_only_and_hand_only_failures':True}):
                f.write_text(json.dumps(value),encoding='utf-8')
                with patch.object(plan,'APPROVAL',f): self.assertFalse(plan.scope_approved())
    def test_default_and_cancel_open_no_devices(self):
        before=plan.APPROVAL.read_bytes()
        for args in (['trial'],['trial','--execute']):
            with patch('sys.argv',args),patch('builtins.input',return_value=''), \
                 patch('dynamixel_sdk.PortHandler',side_effect=AssertionError('motor access')) as port, \
                 patch('camera_stream.CameraStream',side_effect=AssertionError('camera access')) as camera, \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertIn(entry.main(),(0,1)); self.assertEqual(port.call_count,0); self.assertEqual(camera.call_count,0)
        self.assertEqual(before,plan.APPROVAL.read_bytes())
    def test_unapproved_bus_cannot_approve(self):
        with self.assertRaises(RuntimeError): entry.NominalCycleBus.approve(SimpleNamespace(),None,None)
    def test_proof_matches_current_source_and_keeps_failures(self):
        r=gate.validate_review(plan.NominalCyclePlan(plan.REQUEST),plan.PROOF)
        self.assertFalse(r['accepted_for_operator_trial']); self.assertFalse(r['hand_only_all_passed'])
        self.assertFalse(r['all_condition_safety_certified']); self.assertFalse(r['physical_two_turn_success'])
        self.assertTrue(all(not h['passed'] and h['stop'] for h in r['hand_only_comparisons']))
    def test_stale_approval_proof_rejected(self):
        with patch.object(gate,'scope_approved',return_value=True),self.assertRaises(RuntimeError):
            gate.validate_review(plan.NominalCyclePlan(plan.REQUEST),plan.PROOF)
    def test_final_queue_and_second_rotation_required(self):
        c=NominalCycleCapture.__new__(NominalCycleCapture)
        q=plan.candidate()['goals'][-1]
        for report,pending in (({'cube_placement_confirmed':True},[]),
                               ({'cube_placement_confirmed':True,'second_rotation_attempted':True},[1])):
            with self.assertRaises(RuntimeError): c.finish(SimpleNamespace(pending=pending),q,report)
    def test_completion_does_not_claim_success_or_removal(self):
        c=NominalCycleCapture.__new__(NominalCycleCapture); c.guard=object()
        c._observer=lambda *a: object(); c._photo=lambda *a: (None,{'image':'mock'})
        c._space=lambda *a: {'position':np.arange(16)}
        r={'cube_placement_confirmed':True,'second_rotation_attempted':True}
        with contextlib.redirect_stdout(io.StringIO()): c.finish(SimpleNamespace(pending=[]),plan.candidate()['goals'][-1],r)
        self.assertFalse(r['physical_two_turn_success']); self.assertFalse(r['cube_removal_confirmed'])
        self.assertTrue(r['cube_support_confirmed_before_off'])

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
    (plan.OUT/'software_verification.json').write_text(json.dumps({'tests':result.testsRun,'failures':len(result.failures),
        'errors':len(result.errors),'passed':result.wasSuccessful(),'real_devices_opened':0},indent=2)+'\n',encoding='utf-8')
    raise SystemExit(not result.wasSuccessful())
