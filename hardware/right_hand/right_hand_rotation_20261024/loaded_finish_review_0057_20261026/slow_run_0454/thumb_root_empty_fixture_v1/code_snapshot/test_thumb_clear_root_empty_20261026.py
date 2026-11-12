"""Meaningful device-free checks for the new EMPTY motion and shutdown."""
import contextlib,copy,io,itertools,json,subprocess,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import quarter_motion_runner_20261025 as runner
from thumb_clear_root_empty_plan_20261026 import (ThumbRootEmptyPlan,ROOT,FIXTURE,TURN_PHASE,FINISH_PHASE,THUMB_CLEAR,ROOT_UP,END_HOLD,tail_goals,final_goal,finish_goals,motion_goals)
from thumb_clear_root_empty_gate_20261026 import validate_proof
from test_quarter_support_photo_20261025 import FakeBus

class MotionChecks(unittest.TestCase):
    def setUp(self):self.plan=ThumbRootEmptyPlan(FIXTURE/'request.json')
    def run_path(self,fault=None):
        r={};b=FakeBus(self.plan,r);initial=b.snapshot();clock=itertools.count()
        def sample(bus,c,report,phase,snapshot_check):
            if fault=='health' and phase==ROOT_UP:raise RuntimeError('Synthetic failed fresh read before new root lift')
            s=bus.snapshot();snapshot_check(s);return s,np.zeros(16)
        def cancel():
            if fault=='cancel' and r.get('phase')==THUMB_CLEAR and not np.array_equal(b.state['goal'],finish_goals()[-1]):
                raise KeyboardInterrupt('Synthetic cancel during new thumb movement')
        with patch.object(runner,'check_off'),patch.object(runner.checked,'health'),patch.object(runner.checked,'sample',side_effect=sample),contextlib.redirect_stdout(io.StringIO()):
            if fault:
                with self.assertRaises(RuntimeError if fault=='health' else KeyboardInterrupt):runner.run(b,initial,r,cancel=cancel,sleep=lambda _:None,now=lambda:next(clock)*.002)
            else:runner.run(b,initial,r,cancel=cancel,sleep=lambda _:None,now=lambda:next(clock)*.002)
        return r,b,initial
    def test_new_stages_only_after_complete_reference_and_never_loaded(self):
        p=self.plan;rows=p.plan(p.source_position());phases=[s for s,q in rows]
        self.assertFalse(p.CAPTURE_PLACEMENT);self.assertIsNone(p.CAPTURE_BEFORE_PHASE);self.assertIsNone(p.CONTACT_RELEASE_BEFORE_PHASE);self.assertFalse(p.ROTATION_INCLUDED)
        np.testing.assert_array_equal([q for s,q in rows if s==TURN_PHASE],motion_goals()[1:])
        np.testing.assert_array_equal([q for s,q in rows if s==FINISH_PHASE],finish_goals()[1:])
        self.assertEqual(phases.count(THUMB_CLEAR),128);self.assertEqual(phases.count(ROOT_UP),192)
        self.assertLess(max(i for i,s in enumerate(phases) if s==THUMB_CLEAR),phases.index(ROOT_UP))
        self.assertLess(max(i for i,s in enumerate(phases) if s==FINISH_PHASE),phases.index(THUMB_CLEAR))
        delta=final_goal()-finish_goals()[-1];self.assertEqual(delta.tolist(),[0,-128,256,-128,0,0,0,384,0,0,0,384,0,0,0,0])
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([p.source_position()]+[q for s,q in rows]),axis=0))),2)
        self.assertEqual(p.SAMPLE_PERIOD,.075);self.assertLess((len(rows)+40)*.085+2,p.ON_TIMEOUT)
    def test_complete_runner_exact_targets_and_off_restore(self):
        r,b,initial=self.run_path();self.assertEqual(r['status'],self.plan.STATUS)
        self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored']);self.assertEqual(b.off_count,1)
        self.assertNotIn('cube_placement_confirmed',r);self.assertNotEqual(r.get('physical_flip_verified'),True)
        self.assertEqual(r['last_issued_command'],final_goal().tolist())
        np.testing.assert_array_equal([q for addr,q in b.writes if addr==116][2:],[q for p,q in self.plan.plan(self.plan.source_position())])
        for addr,key in ((102,'current_cap'),(108,'acceleration'),(112,'velocity'),(98,'watchdog'),(80,'d_gain'),(82,'i_gain'),(84,'p_gain')):
            np.testing.assert_array_equal(next(q for a,q in reversed(b.writes) if a==addr),initial[key])
    def test_bad_read_before_extra_root_lift_prevents_that_target(self):
        r,b,_=self.run_path('health');self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
        expected=[q for p,q in tail_goals() if p==THUMB_CLEAR][-1]
        self.assertEqual(r['last_issued_command'],expected.tolist());self.assertNotEqual(r.get('status'),self.plan.STATUS)
    def test_cancel_in_new_thumb_motion_stops_and_restores(self):
        r,b,_=self.run_path('cancel');self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
        self.assertNotEqual(r.get('status'),self.plan.STATUS);self.assertEqual(r['last_issued_command'],tail_goals()[0][1].tolist())
    def test_default_no_devices_and_correct_instructions(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_thumb_clear_root_empty_20261026.py')],capture_output=True,text=True,encoding='utf-8',timeout=8)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('NO PLACE step',r.stdout)
        self.assertIn('75ms cycles',r.stdout);self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)

class ProofChecks(unittest.TestCase):
    def test_complete_proof_and_rejection_of_loaded_permission_or_other_proposal(self):
        p=ThumbRootEmptyPlan(FIXTURE/'request.json');r=validate_proof(p,FIXTURE/'screen.json')
        self.assertTrue(r['empty_hand_only']);self.assertFalse(r['placement_checkpoint_required']);self.assertFalse(r['experimental_loaded_trial'])
        wrong=copy.deepcopy(r);wrong['thumb_root_proposal']['additional_root_ticks']=512
        for bad in (wrong,{**r,'empty_hand_only':False},{**r,'native_placement_hold_seconds':4.},{**r,'control_cycle_seconds':.05}):
            with tempfile.TemporaryDirectory() as d:
                f=Path(d)/'bad.json';f.write_text(json.dumps(bad),encoding='utf-8')
                with self.assertRaises(RuntimeError):validate_proof(p,f)
if __name__=='__main__':unittest.main()
