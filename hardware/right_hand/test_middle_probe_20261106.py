"""Reject unsafe changes and keep image capture downstream of health checks."""
import copy,json,unittest,tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import middle_probe_plan_20261106 as plan
class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.z={k:v.copy() for k,v in np.load(plan.CANDIDATE).items()};self.old={k:v.copy() for k,v in plan.natural_candidate().items()};self.meta=json.loads((plan.O/'proposal.json').read_bytes());self.alias={k:v.copy() for k,v in np.load(plan.ALIAS).items()}
    def check(self):plan.validate_arrays(self.z['start'],self.z['goals'],self.z['phases'],self.old,self.meta,self.alias)
    def test_source_candidate_accepted(self):plan.validate_candidate();self.check()
    def test_fixed_thumb_change_rejected(self):
        i=np.flatnonzero(self.z['phases']==plan.FOLD)[0];self.z['goals'][i,0]+=1
        with self.assertRaises(RuntimeError):self.check()
    def test_native_excursion_rejected(self):
        self.z['goals'][950,8]=5000
        with self.assertRaises(RuntimeError):self.check()
    def test_extra_joint_step_rejected(self):
        self.z['goals'][950,8]+=8
        with self.assertRaises(RuntimeError):self.check()
    def test_changed_original_grip_rejected(self):
        self.z['goals'][20,0]+=1
        with self.assertRaises(RuntimeError):self.check()
    def test_missing_return_rejected(self):
        self.z['goals'][np.flatnonzero(self.z['phases']==plan.RETURN)[-1],8]+=1
        with self.assertRaises(RuntimeError):self.check()
    def test_shortened_observation_rejected(self):
        self.z['phases'][np.flatnonzero(self.z['phases']==plan.HOLD)[0]]=plan.FOLD
        with self.assertRaises(RuntimeError):self.check()
    def test_evaluator_cannot_substitute_goals(self):
        self.alias['goals'][950,8]+=1
        with self.assertRaises(RuntimeError):self.check()
    def test_probe_holds_other_twelve_joints(self):
        p=plan.MiddleProbePlan.__new__(plan.MiddleProbePlan)
        self.assertEqual(p.frozen_for_phase(plan.FOLD).tolist(),[0,1,2,3,4,5,6,7,12,13,14,15]);self.assertEqual(p.frozen_for_phase(plan.HOLD).tolist(),list(range(16)))
class SamplingTests(unittest.TestCase):
    def test_probe_photo_waits_for_settling(self):
        from middle_probe_capture_20261106 import MiddleProbeCapture
        capture=MiddleProbeCapture.__new__(MiddleProbeCapture);clock=[10.];capture.now=lambda:clock[0];camera=SimpleNamespace(capture_named=lambda name,after: name if clock[0]>=after else None);capture.camera=camera;report={};state={'position':np.zeros(16,dtype=int),'goal':np.zeros(16,dtype=int)}
        capture.middle_frame(None,np.zeros(16,dtype=int),report,state);self.assertNotIn('middle_probe_photo',report)
        clock[0]=11.2;capture.middle_frame(None,np.zeros(16,dtype=int),report,state);self.assertEqual(report['middle_probe_photo'],'middle_probe_hold.jpg');self.assertEqual(report['middle_probe_position_ticks'],[0]*16)
    def test_photo_only_after_parent_health_succeeds(self):
        import leap_middle_clearance_probe_20261106 as entry
        callback=__import__('unittest').mock.Mock();bus=SimpleNamespace(grasp_protocol=SimpleNamespace(middle_frame=callback));command=np.zeros(16);report={};state={'position':np.zeros(16)}
        with patch.object(entry,'BASE_SAMPLE',return_value=(state,np.zeros(16))) as health:
            got=entry.MiddleProbeBus.checked_sample(bus,command,report,plan.HOLD,snapshot_check='sentinel')
            health.assert_called_once_with(bus,command,report,plan.HOLD,snapshot_check='sentinel');self.assertIs(got[0],state);callback.assert_called_once_with(bus,command,report,state)
        callback.reset_mock()
        with patch.object(entry,'BASE_SAMPLE',side_effect=RuntimeError('health fault')):
            with self.assertRaises(RuntimeError):entry.MiddleProbeBus.checked_sample(bus,command,report,plan.HOLD)
        callback.assert_not_called()
class CenteringTests(unittest.TestCase):
    def request(self,delta):
        r=json.loads(plan.REQUEST.read_bytes());r['off_state']['position'][0]+=delta
        f=tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',suffix='.json',dir=plan.O,delete=False);json.dump(r,f);f.close();p=Path(f.name);p.resolve().relative_to(plan.R);return p
    def test_one_tick_drift_reuses_center(self):
        source=self.request(1)
        try:
            p=plan.MiddleProbePlan(source);ref=plan.MiddleProbePlan(plan.REQUEST);self.assertTrue(p.review_center_reused);np.testing.assert_array_equal(p.source_position(),ref.source_position());self.assertEqual(int(p.raw_off_ticks[0]-p.source_position()[0]),1);np.testing.assert_array_equal(p.validate_start(p.raw_off_ticks),p.raw_off_ticks)
        finally:source.unlink()
    def test_five_tick_drift_requires_new_screen(self):
        source=self.request(5)
        try:
            p=plan.MiddleProbePlan(source);ref=plan.MiddleProbePlan(plan.REQUEST);self.assertFalse(p.review_center_reused);self.assertNotEqual(p.source_position()[0],ref.source_position()[0])
        finally:source.unlink()
    def test_one_tick_runtime_reuses_frozen_review_without_process(self):
        import middle_probe_gate_20261106 as gate
        source=self.request(1);file=tempfile.NamedTemporaryFile(mode='w',suffix='.json',dir=plan.O,delete=False);file.close();output=Path(file.name);output.resolve().relative_to(plan.R)
        try:
            with patch.object(gate,'process_screen',side_effect=AssertionError('Unnecessary simulator process')) as process:
                r=gate.run_screen(source,output);self.assertTrue(r['review_center_reused']);process.assert_not_called();self.assertEqual(r['raw_off_ticks'][0]-r['empty_recovery']['start_ticks'][0],1)
            p=plan.MiddleProbePlan(source);rows=p.plan(p.raw_off_ticks);self.assertLessEqual(int(np.max(abs(rows[0][1]-p.raw_off_ticks))),2)
        finally:source.unlink();output.unlink()
    def test_nonprobe_phase_does_not_capture_probe_photo(self):
        import leap_middle_clearance_probe_20261106 as entry
        callback=__import__('unittest').mock.Mock();bus=SimpleNamespace(grasp_protocol=SimpleNamespace(middle_frame=callback))
        with patch.object(entry,'BASE_SAMPLE',return_value=({},np.zeros(16))):entry.MiddleProbeBus.checked_sample(bus,np.zeros(16),{},plan.RETURN)
        callback.assert_not_called()
if __name__=='__main__':unittest.main(verbosity=2)
