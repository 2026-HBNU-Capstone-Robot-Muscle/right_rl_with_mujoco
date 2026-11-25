"""Queue continuity, health-first admission, fixed review and no default I/O."""
import contextlib,copy,io,json,sys,unittest
from unittest.mock import patch
import numpy as np
import retained_slow_yaw_plan_20261029 as plan
import retained_slow_yaw_gate_20261029 as gate
class Checks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p=plan.RetainedSlowYawPlan(plan.REQUEST);cls.request=json.loads(plan.REQUEST.read_bytes())
        cls.folder=plan.OUT/'unit_test_requests';cls.folder.mkdir(exist_ok=True)
    def request_file(self,label,change):
        r=copy.deepcopy(self.request);change(r);p=self.folder/(label+'.json');p.write_text(json.dumps(r),encoding='utf-8');return p
    def test_continuous_queue_stays_inside_original300s_and2ticks(self):
        rows=self.p.plan(self.p.source_position());values=np.vstack([self.p.source_position(),[q for p,q in rows]])
        self.assertLessEqual(int(np.max(abs(np.diff(values,axis=0)))),2)
        self.assertLessEqual((len(rows)+40)*.075+20,300)
        np.testing.assert_array_equal(rows[-1][1],np.load(plan.LOADED_PATH)['start'])
    def test_loaded_suffix_matches_reviewed_combined_angles_and_timing(self):
        from pitch_slow_yaw_plan_20261029 import loaded_rows
        own=plan.loaded_rows();combined=loaded_rows()
        self.assertEqual([(p,q.tolist()) for p,q in own],[(p,q.tolist()) for p,q in combined[-len(own):]])
        self.assertFalse(any('pitch' in p or 'transfer' in p for p,q in self.p.plan(self.p.source_position())))
    def test_noise4_cached_but5_needs_fresh_screen(self):
        for amount,reuse in ((4,True),(5,False)):
            path=self.request_file('noise'+str(amount),lambda r:r['off_state']['position'].__setitem__(1,r['off_state']['position'][1]+amount))
            p=plan.RetainedSlowYawPlan(path);self.assertEqual(p.review_center_reused,reuse)
    def test_bad_health_cannot_be_hidden_by_cached_center(self):
        path=self.request_file('torque_on',lambda r:r['off_state']['torque'].__setitem__(0,1))
        with self.assertRaises(ValueError):plan.RetainedSlowYawPlan(path)
    def test_corrupt_review_does_not_authorize_operator(self):
        r=json.loads(plan.PROOF.read_bytes());r['read_resynchronization_policy']['total_read_budget_s']=1.
        path=self.folder/'tampered_budget.json';path.write_text(json.dumps(r),encoding='utf-8')
        with self.assertRaises(RuntimeError):gate.validate_review(self.p,path)
    def test_rejected_hand_case_not_overridden_by_all_passed_flag(self):
        original=gate.read
        def replaced(path):
            r=original(path)
            if path==plan.OUT/'loaded_hand_dynamics/review.json':r=copy.deepcopy(r);r['cases'][0]['max_self_mm']=.2
            return r
        with patch.object(gate,'read',side_effect=replaced):
            with self.assertRaises(RuntimeError):gate.validate_loaded()
    def test_scope_preserves_unverified_real_result_and_broad_failures(self):
        r=gate.validate_review(self.p,plan.PROOF)
        self.assertFalse(r['physical_retained_red_yaw_verified'])
        self.assertTrue(r['loaded_evidence']['supervised_experiment_only'])
        self.assertFalse(r['loaded_evidence']['combined_reference']['all27_parameter_conditions_passed'])
    def test_bus_dispatches_scoped_transport_and_keeps_native_health_sampler(self):
        import leap_retained_slow_yaw_20261029 as entry
        from calibrated_two_full_health_20261028 import sample
        b=object.__new__(entry.RetainedBus);b.report={}
        request=dict(kind='sync_read',address=64,length=83)
        with patch.object(entry,'receive',return_value='fresh') as transport:
            self.assertEqual(b._receive('operation','READ',request),'fresh');transport.assert_called_once_with(b,'operation','READ',request)
        self.assertIs(entry.RetainedBus.checked_sample,sample)
        self.assertEqual(b.report['read_resynchronization_policy']['total_read_budget_s'],.2)
    def test_default_entry_does_not_open_devices(self):
        import cv2,dynamixel_sdk
        import leap_retained_slow_yaw_20261029 as entry
        with patch.object(sys,'argv',['leap_retained_slow_yaw_20261029.py']),contextlib.redirect_stdout(io.StringIO()), \
             patch.object(cv2,'VideoCapture',side_effect=AssertionError('Camera opened')), \
             patch.object(dynamixel_sdk,'PortHandler',side_effect=AssertionError('Motor port opened')):
            self.assertEqual(entry.main(),0)
if __name__=='__main__':unittest.main(verbosity=2)
