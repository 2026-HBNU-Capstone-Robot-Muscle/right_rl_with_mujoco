"""No-device tests for the scoped initial-pair escape and actual-angle interlock."""
import contextlib,copy,io,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import horizontal_quarter_plan_20261027 as p
import horizontal_quarter_gate_20261027 as gate
import leap_horizontal_quarter_20261027 as live
import leap_new_pair_grasp_20261027 as base
from rewired_initial_pair_scope_20261027 import InitialPairScope,PAIR

class ScopeTests(unittest.TestCase):
    def test_original_pair_is_allowed_only_in_initial_ring_segment(self):
        scope=InitialPairScope({PAIR:1.2},True)
        self.assertTrue(scope.valid);self.assertEqual(scope.filtered(p.RING_CLEAR,{PAIR:.7}),{})
        self.assertEqual(scope.filtered(p.ROOTS,{PAIR:.7}),{PAIR:.7})
        self.assertEqual(scope.filtered(p.OPEN,{PAIR:.7}),{PAIR:.7})
    def test_more_than_two_mm_is_never_accepted(self):
        scope=InitialPairScope({PAIR:2.001},True);self.assertFalse(scope.valid)
        self.assertEqual(scope.filtered(p.RING_CLEAR,{PAIR:1.9}),{PAIR:1.9})
    def test_deeper_pair_is_not_masked(self):
        scope=InitialPairScope({PAIR:1.},True)
        self.assertEqual(scope.filtered(p.RING_CLEAR,{PAIR:1.01}),{PAIR:1.01})
    def test_any_other_initial_pair_invalidates_the_exception(self):
        scope=InitialPairScope({PAIR:1.,'other_pair':.001},True);self.assertFalse(scope.valid)
        self.assertEqual(scope.filtered(p.RING_CLEAR,{PAIR:.5,'other_pair':.001}),{PAIR:.5,'other_pair':.001})
    def test_no_new_contact_is_allowed_when_the_initial_pair_is_absent(self):
        scope=InitialPairScope({},True)
        self.assertEqual(scope.filtered(p.RING_CLEAR,{PAIR:.001}),{PAIR:.001})
    def test_no_exception_when_the_ring_prefix_is_disabled(self):
        scope=InitialPairScope({PAIR:.1},False);self.assertFalse(scope.valid)
        self.assertEqual(scope.filtered(p.RING_CLEAR,{PAIR:.1}),{PAIR:.1})
    def test_only_ring_side_goal_changes_before_root_recovery(self):
        request=json.loads((p.ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v5_rewired/request.json').read_bytes())
        request['plan_version']=p.HorizontalQuarterPlan.PLAN_VERSION
        with tempfile.TemporaryDirectory(prefix='legacy_ring_request_',dir=p.ROOT) as temp:
            path=Path(temp)/'request.json';path.write_text(json.dumps(request),encoding='utf-8')
            plan=p.HorizontalQuarterPlan(path);a=plan.source_position();rows=plan.plan(a)
        ring=[q for phase,q in rows if phase==p.RING_CLEAR]
        self.assertEqual(len(ring),48);self.assertEqual(ring[-1][13]-a[13],96)
        self.assertTrue(all(np.array_equal(q[np.arange(16)!=13],a[np.arange(16)!=13]) for q in ring))
        phases=[phase for phase,q in rows];self.assertLess(phases.index(p.RING_CLEAR),phases.index(p.ROOTS))
    def test_root_targets_cannot_advance_before_actual_ring_clearance(self):
        bus=live.HorizontalQuarterBus.__new__(live.HorizontalQuarterBus);bus.report={'phase':p.ROOTS};bus.screen_approved=True;bus.adopted=True;bus.initial_pair_checked=False
        bus.proof={'initial_self_separation':{'required_actual_ring_rotation_ticks':2058}};a=p.HorizontalQuarterPlan(p.REQUEST).source_position();bus.last_reviewed_state={'position':a.copy()};bus.last_reviewed_state['position'][13]=2057
        with patch.object(base.NewPairBus,'write') as write:
            with self.assertRaisesRegex(RuntimeError,'Measured ring'):bus.write(116,4,a)
            write.assert_not_called();bus.last_reviewed_state['position'][13]=2058;bus.write(116,4,a);write.assert_called_once();self.assertTrue(bus.initial_pair_checked)

class ScopeProofTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not p.PROOF.exists():raise unittest.SkipTest('model review pending')
        cls.plan=p.HorizontalQuarterPlan(p.REQUEST);cls.review=gate.validate_review(cls.plan,p.PROOF)
    def reject(self,fn):
        row=copy.deepcopy(self.review);fn(row)
        with tempfile.TemporaryDirectory(prefix='initial_separation_gate_',dir=p.ROOT) as temp:
            path=Path(temp)/'review.json';path.write_text(json.dumps(row),encoding='utf-8')
            with self.assertRaises((RuntimeError,ValueError,KeyError,TypeError)):gate.validate_review(self.plan,path)
    def test_relaxed_initial_bound_rejected(self):self.reject(lambda r:r['initial_self_separation']['policy'].__setitem__('maximum_initial_depth_mm',3.))
    def test_deeper_dynamic_initial_contact_rejected(self):self.reject(lambda r:r['empty_recovery']['dynamic_cases'][0].__setitem__('initial_pair_increase_mm',.001))
    def test_static_recontact_in_ring_prefix_rejected(self):self.reject(lambda r:r['empty_recovery']['static_cases'][0].__setitem__('initial_pair_step_increase_mm',.001))
    def test_uncleared_end_of_ring_prefix_rejected(self):self.reject(lambda r:r['empty_recovery']['dynamic_cases'][0].__setitem__('last_ring_pair_mm',.001))
if __name__=='__main__':unittest.main(verbosity=2)
