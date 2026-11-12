"""Device-free tests for the operator large-turn transaction and proof gate."""
import contextlib,copy,io,json,runpy,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import horizontal_quarter_plan_20261027 as p
import horizontal_quarter_gate_20261027 as gate
import leap_horizontal_quarter_20261027 as live
import leap_new_pair_grasp_20261027 as old_live
from horizontal_quarter_capture_20261027 import HorizontalQuarterCapture
from loaded_quarter_reentry_20261025 import limits,cmc_native_min
import sequence_flip_checked as checked

class TransactionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan=p.HorizontalQuarterPlan(p.REQUEST);cls.a=cls.plan.source_position();cls.rows=cls.plan.plan(cls.a)
    def test_default_opens_no_devices(self):
        with patch.object(sys,'argv',['leap_horizontal_quarter_20261027.py']),patch('quarter_finish_empty_camera_20261025.QuarterFinishEmptyCamera.__init__',side_effect=AssertionError('camera opened')),patch('subprocess.Popen',side_effect=AssertionError('process opened')),contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as exit:runpy.run_path(str(p.ROOT/'leap_horizontal_quarter_20261027.py'),run_name='__main__')
        self.assertIn(exit.exception.code,(0,2))
    def test_execute_missing_review_stops_before_devices(self):
        with patch.object(sys,'argv',['leap_horizontal_quarter_20261027.py','--execute']),patch.object(gate,'validate_review',side_effect=RuntimeError('missing review')),patch('quarter_finish_empty_camera_20261025.QuarterFinishEmptyCamera.__init__',side_effect=AssertionError('camera opened')),contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as exit:runpy.run_path(str(p.ROOT/'leap_horizontal_quarter_20261027.py'),run_name='__main__')
        self.assertEqual(exit.exception.code,2)
    def test_native_goals_two_ticks_and_monotonic_off_return(self):
        low,high=limits();previous=self.a
        for phase,q in self.rows:
            self.assertLessEqual(int(max(abs(q-previous))),2);self.assertTrue(np.all(q<=high));admitted=low.copy()
            if phase==p.ENTRY:admitted[3]=self.a[3];self.assertGreaterEqual(q[3],previous[3])
            self.assertTrue(np.all(q>=admitted));previous=q
    def test_only_existing_passive_cmc_exception(self):
        low,_=limits();bad=self.a.copy();bad[2]=int(np.floor(low[2]))-1
        with self.assertRaises(ValueError):self.plan.validate_admission(bad)
        bad=self.a.copy();bad[3]=1600
        with self.assertRaises(ValueError):self.plan.validate_admission(bad)
    def test_transaction_order_and_removal_after_open(self):
        phases=[v[0] for v in self.rows];ordered=[p.SETTLE,p.GRASP,p.HOLD,p.YAW,p.YAW_HOLD,p.REOPEN,p.REMOVE]
        self.assertEqual([phases.index(v) for v in ordered],sorted(phases.index(v) for v in ordered));self.assertEqual(phases.count(p.YAW_HOLD),40)
        self.assertTrue(np.array_equal(self.rows[-1][1],np.load(p.OPEN_PATH)['open_goal']))
    def test_loaded_queue_is_exact_reviewed_data(self):
        data=np.load(p.LOADED_PATH);actual=[q for phase,q in self.rows if phase not in p.PREFIX_PHASES]
        self.assertTrue(np.array_equal(actual,data['goals']))
    def test_budget_matches_longer_path(self):
        self.assertLess((len(self.rows)+40)*.075+20,self.plan.ON_TIMEOUT);self.assertEqual(self.plan.ON_TIMEOUT,300)
    def test_invalid_loaded_target_rejected(self):
        rows=p.loaded_rows();rows[1]=(rows[1][0],rows[1][1]+2000)
        with patch.object(p,'loaded_rows',return_value=rows):
            with self.assertRaises(ValueError):self.plan.plan(self.a)
    def test_no_writes_before_review_and_emergency_off_allowed(self):
        bus=live.HorizontalQuarterBus.__new__(live.HorizontalQuarterBus);bus.report={}
        with patch.object(checked.Bus,'write') as write:
            for addr,size,value in [(84,2,np.full(16,600)),(64,1,np.ones(16)),(116,4,self.a)]:
                with self.assertRaises(RuntimeError):bus.write(addr,size,value)
            write.assert_not_called();bus.write(64,1,np.zeros(16));write.assert_called_once()
    def test_no_target_resend(self):
        bus=live.HorizontalQuarterBus.__new__(live.HorizontalQuarterBus);bus.report={};bus.screen_approved=True;bus.plan_module=self.plan;bus.bind(self.a)
        with patch.object(checked.Bus,'write') as write:
            bus.write(116,4,self.a);bus.adopt(self.a);bus.write(116,4,self.a);bus.write(116,4,self.rows[0][1]);count=write.call_count
            with self.assertRaises(ValueError):bus.write(116,4,self.rows[0][1])
            self.assertEqual(write.call_count,count)
    def test_actual_cmc_before_roots(self):
        bus=live.HorizontalQuarterBus.__new__(live.HorizontalQuarterBus);bus.report={'phase':p.ROOTS};bus.screen_approved=True;bus.adopted=True;bus.last_reviewed_state={'position':self.a.copy()}
        bus.last_reviewed_state['position'][3]=cmc_native_min()-1
        with patch.object(old_live.GuardedBus,'write') as write:
            with self.assertRaises(RuntimeError):bus.write(116,4,self.a)
            write.assert_not_called();bus.last_reviewed_state['position'][3]=cmc_native_min()+4;bus.write(116,4,self.a);write.assert_called_once()
    def test_health_failure_propagates(self):
        with patch.object(live,'base_sample',side_effect=RuntimeError('undervoltage')):
            with self.assertRaisesRegex(RuntimeError,'undervoltage'):live.sample(SimpleNamespace(),self.a,{},p.YAW)
    def test_remove_requires_finished_open_queue(self):
        capture=HorizontalQuarterCapture(None,None,cancel=lambda:None);opened=np.load(p.OPEN_PATH)['open_goal'];bus=SimpleNamespace(pending=[self.a])
        with self.assertRaises(RuntimeError):capture.finish(bus,opened,{})
        with self.assertRaisesRegex(RuntimeError,'Open endpoint'):capture.finish(bus,self.a,{'cube_placement_confirmed':True})
        with self.assertRaisesRegex(RuntimeError,'queue incomplete'):capture.finish(bus,opened,{'cube_placement_confirmed':True})
    def test_rotation_observation_does_not_claim_success(self):
        capture=HorizontalQuarterCapture(SimpleNamespace(capture_named=lambda *a,**kw:'turn.jpg'),None,cancel=lambda:None,now=lambda:12.);report={}
        with contextlib.redirect_stdout(io.StringIO()):capture.rotation_frame(None,self.a,report,{'position':self.a})
        self.assertFalse(report['physical_horizontal_90_verified']);self.assertEqual(report['rotation_observation_photo'],'turn.jpg')
    def test_inherited_camera_accepts_cube_after_place(self):self.assertFalse(self.plan.CAMERA_TYPE.EMPTY_ONLY)
    def test_operator_uses_dedicated_runner_and_restores_shared_runtime(self):
        previous=live.runtime.run
        def inspect(*args,**kwargs):self.assertIs(live.runtime.run,live.confirmed_run);return 17
        with patch.object(live.runtime,'main',side_effect=inspect):self.assertEqual(live.operator_main(),17)
        self.assertIs(live.runtime.run,previous)

class ProofTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not p.PROOF.exists():raise unittest.SkipTest('complete model proof pending')
        cls.plan=p.HorizontalQuarterPlan(p.REQUEST);cls.review=gate.validate_review(cls.plan,p.PROOF)
    def mutated(self,fn):
        report=copy.deepcopy(self.review);fn(report)
        with tempfile.TemporaryDirectory(prefix='quarter_gate_test_',dir=p.ROOT) as temp:
            path=Path(temp)/'review.json';path.write_text(json.dumps(report),encoding='utf-8')
            with self.assertRaises((RuntimeError,ValueError,KeyError,TypeError)):gate.validate_review(self.plan,path)
    def test_valid_review_matches(self):self.assertTrue(self.review['accepted_for_operator_trial'])
    def test_reject_incomplete(self):self.mutated(lambda r:r['empty_recovery']['static_cases'].pop())
    def test_reject_duplicate_case(self):self.mutated(lambda r:r['empty_recovery']['dynamic_cases'].__setitem__(0,r['empty_recovery']['dynamic_cases'][1]))
    def test_reject_nan(self):self.mutated(lambda r:r['empty_recovery']['dynamic_cases'][0].__setitem__('max_tracking_rad',float('nan')))
    def test_reject_native_excess(self):self.mutated(lambda r:r['empty_recovery']['dynamic_cases'][0].__setitem__('unapproved_range_excess_rad',.001))
    def test_reject_code_change(self):self.mutated(lambda r:r['code_hashes'].__setitem__('hand_motor_mapping.py','0'*64))
    def test_reject_queue_change(self):self.mutated(lambda r:r.__setitem__('plan_sha256','0'*64))
    def test_reject_relaxed_health_stop(self):self.mutated(lambda r:r.__setitem__('hardware_tracking_stop_rad',.3))
    def test_reject_false_physical_success(self):self.mutated(lambda r:r.__setitem__('physical_horizontal_90_verified',True))
    def test_reject_changed_preload_policy(self):self.mutated(lambda r:r['off_preload_confirmation_policy'].__setitem__('position_span_ticks',20))
    def test_reject_changed_goal_confirmation_policy(self):self.mutated(lambda r:r['goal_readback_confirmation_policy'].__setitem__('exact_current_reads',1))
    def test_reject_changed_root_clearance_policy(self):self.mutated(lambda r:r['empty_root_motion_policy'].__setitem__('initial_parallel_clearance_goal_ticks',1900))
    def test_reject_changed_placement_tolerance(self):self.mutated(lambda r:r['stable_placement_review']['policy']['error_limits_by_motor_id_1_to_16'].__setitem__(11,96))
    def test_reject_changed_placement_stability(self):self.mutated(lambda r:r['stable_placement_review']['policy'].__setitem__('maximum_position_span_ticks',20))
    def test_reject_changed_receive_deadline(self):self.mutated(lambda r:r['read_resynchronization_policy'].__setitem__('total_read_budget_s',1.))
    def test_reject_unbounded_receive_requests(self):self.mutated(lambda r:r['read_resynchronization_policy'].__setitem__('maximum_requests',99))
    def test_reject_extended_packet_window_over80ms(self):self.mutated(lambda r:r['sync_receive_window_policy'].__setitem__('packet_window_ms',200.))
    def test_reject_extending_unrelated_read_scope(self):self.mutated(lambda r:r['sync_receive_window_policy']['scopes'].append([98,1]))

if __name__=='__main__':unittest.main(verbosity=2)
