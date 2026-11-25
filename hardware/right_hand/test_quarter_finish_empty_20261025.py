"""Device-free checks for the EMPTY continuation and failure shutdown."""
import contextlib,copy,io,itertools,json,subprocess,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import quarter_motion_runner_20261025 as runner
from quarter_finish_empty_plan_20261025 import (QuarterFinishEmptyPlan,ROOT,FIXTURE,TURN_PHASE,FINISH_PHASE,END_HOLD,finish_goals,motion_goals)
from quarter_finish_empty_gate_20261025 import validate_proof
from test_quarter_support_photo_20261025 import FakeBus
class EmptyFinishChecks(unittest.TestCase):
    def setUp(self):self.plan=QuarterFinishEmptyPlan(FIXTURE/'request.json')
    def run_path(self,fault=None):
        report={};bus=FakeBus(self.plan,report);initial=bus.snapshot();clock=itertools.count()
        def sample(b,c,r,phase,snapshot_check):
            if fault=='health' and phase==FINISH_PHASE:raise RuntimeError('Synthetic unhealthy read')
            s=b.snapshot();snapshot_check(s);return s,np.zeros(16)
        def cancel():
            if fault=='cancel' and report.get('phase')==FINISH_PHASE and not np.array_equal(bus.state['goal'],motion_goals()[-1]):raise KeyboardInterrupt('Synthetic stop during finish')
        error=RuntimeError if fault=='health' else KeyboardInterrupt
        with patch.object(runner,'check_off'),patch.object(runner.checked,'health'),patch.object(runner.checked,'sample',side_effect=sample),contextlib.redirect_stdout(io.StringIO()):
            if fault:
                with self.assertRaises(error):runner.run(bus,initial,report,cancel=cancel,sleep=lambda x:None,now=lambda:next(clock)*.002)
            else:runner.run(bus,initial,report,cancel=cancel,sleep=lambda x:None,now=lambda:next(clock)*.002)
        return report,bus,initial
    def test_plan_has_no_cube_checkpoint_and_only_two_changed_finish_motors(self):
        self.assertFalse(self.plan.CAPTURE_PLACEMENT);self.assertIsNone(self.plan.CAPTURE_BEFORE_PHASE);self.assertIsNone(self.plan.CONTACT_RELEASE_BEFORE_PHASE)
        rows=self.plan.plan(self.plan.source_position());np.testing.assert_array_equal([q for p,q in rows if p==TURN_PHASE],motion_goals()[1:])
        np.testing.assert_array_equal([q for p,q in rows if p==FINISH_PHASE],finish_goals()[1:]);self.assertEqual(sum(p==END_HOLD for p,q in rows),60)
        delta=finish_goals()[-1]-motion_goals()[-1];self.assertEqual(np.flatnonzero(delta).tolist(),[7,11]);self.assertEqual(delta[[7,11]].tolist(),[128,128])
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([self.plan.source_position()]+[q for p,q in rows]),axis=0))),2)
    def test_runner_finishes_without_place_remove_callbacks_and_restores(self):
        report,bus,initial=self.run_path();self.assertEqual(report['status'],self.plan.STATUS);self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored']);self.assertEqual(bus.off_count,1)
        self.assertNotIn('cube_placement_confirmed',report);self.assertEqual(report['last_issued_command'],finish_goals()[-1].tolist())
        writes=[q for addr,q in bus.writes if addr==116];np.testing.assert_array_equal(writes[2:],[q for p,q in self.plan.plan(self.plan.source_position())])
        for addr,size,key in ((102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),(98,1,'watchdog'),(80,2,'d_gain'),(82,2,'i_gain'),(84,2,'p_gain')):np.testing.assert_array_equal(next(q for a,q in reversed(bus.writes) if a==addr),initial[key])
    def test_cancel_during_finish_shuts_down_and_does_not_claim_completion(self):
        r,bus,_=self.run_path('cancel');self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored']);self.assertNotEqual(r.get('status'),self.plan.STATUS);self.assertEqual(r['last_issued_command'][7],finish_goals()[1,7])
    def test_unhealthy_finish_read_sends_no_new_finish_goal(self):
        r,bus,_=self.run_path('health');self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored']);self.assertEqual(r['last_issued_command'],motion_goals()[-1].tolist())
    def test_complete_proof_matches_empty_path(self):
        r=validate_proof(self.plan,FIXTURE/'screen.json');self.assertTrue(r['empty_hand_only']);self.assertFalse(r['placement_checkpoint_required']);self.assertFalse(r['experimental_loaded_trial']);self.assertTrue(r['cube_disabled_entire_screen'])
    def test_proof_rejects_cube_enabled_or_loaded_permission(self):
        r=json.loads((FIXTURE/'screen.json').read_bytes())
        for key,value in (('cube_disabled_entire_screen',False),('empty_hand_only',False),('placement_checkpoint_required',True),('empty_root_finish_goal_count',63)):
            v=copy.deepcopy(r);v[key]=value
            with tempfile.TemporaryDirectory() as d:
                p=Path(d)/'proof.json';p.write_text(json.dumps(v),encoding='utf-8')
                with self.assertRaises(RuntimeError):validate_proof(self.plan,p)
    def test_default_does_not_open_devices(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_quarter_finish_empty_20261025.py')],capture_output=True,text=True,encoding='utf-8',timeout=8)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)
if __name__=='__main__':unittest.main()
