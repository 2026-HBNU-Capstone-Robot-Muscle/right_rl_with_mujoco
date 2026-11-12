"""No-device tests of the new PLACE/grasp/OPEN/removal transaction."""
import contextlib,copy,io,json,runpy,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import new_pair_grasp_plan_20261027 as p
import new_pair_grasp_gate_20261027 as gate
import leap_new_pair_grasp_20261027 as live
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from loaded_quarter_reentry_20261025 import limits,cmc_native_min
import sequence_flip_checked as checked

class CycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan=p.NewPairGraspPlan(p.REQUEST);cls.a=cls.plan.source_position();cls.rows=cls.plan.plan(cls.a)
    def test_default_cannot_open_devices(self):
        with patch.object(sys,'argv',['leap_new_pair_grasp_20261027.py']),patch('quarter_finish_empty_camera_20261025.QuarterFinishEmptyCamera.__init__',side_effect=AssertionError('camera opened')),patch('subprocess.Popen',side_effect=AssertionError('process opened')),contextlib.redirect_stdout(io.StringIO()) as output:
            with self.assertRaises(SystemExit) as exit:runpy.run_path(str(p.ROOT/'leap_new_pair_grasp_20261027.py'),run_name='__main__')
        self.assertEqual(exit.exception.code,0);self.assertIn('no camera, motor port',output.getvalue())
    def test_execute_without_proof_stops_before_devices(self):
        with patch.object(sys,'argv',['leap_new_pair_grasp_20261027.py','--execute']),patch.object(gate,'validate_proof',side_effect=RuntimeError('test missing review')),patch('quarter_finish_empty_camera_20261025.QuarterFinishEmptyCamera.__init__',side_effect=AssertionError('camera opened')),contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as exit:runpy.run_path(str(p.ROOT/'leap_new_pair_grasp_20261027.py'),run_name='__main__')
        self.assertEqual(exit.exception.code,2)
    def test_camera_allows_loaded_checkpoint(self):self.assertFalse(live.PairGraspCamera.EMPTY_ONLY)
    def test_queue_native_except_monotone_cmc_reentry(self):
        lo,hi=limits();previous=self.a.copy()
        for phase,q in self.rows:
            self.assertLessEqual(int(max(abs(q-previous))),2)
            self.assertTrue(np.all(q<=hi));allowed=lo.copy()
            if phase==p.ENTRY:allowed[3]=min(lo[3],self.a[3]);self.assertGreaterEqual(q[3],previous[3])
            self.assertTrue(np.all(q>=allowed));previous=q
    def test_only_measured_cmc_gets_off_exception(self):
        lo,hi=limits();bad=self.a.copy();bad[2]=int(np.floor(lo[2]))-1
        with self.assertRaises(ValueError):p.admission(bad)
        bad=self.a.copy();bad[3]=int(np.floor(lo[3]-32))-1
        with self.assertRaises(ValueError):p.admission(bad)
    def test_loaded_queue_closes_then_opens_before_removal(self):
        phases=[phase for phase,q in self.rows]
        self.assertLess(phases.index(p.SETTLE),phases.index(p.GRASP));self.assertLess(phases.index(p.HOLD),phases.index(p.REOPEN));self.assertLess(phases.index(p.REOPEN),phases.index(p.REMOVE))
        self.assertEqual(phases.count(p.HOLD),40);self.assertTrue(np.array_equal(self.rows[-1][1],np.load(p.OPEN_PATH)['open_goal']))
        before=np.load(p.GRASP_PATH)['grasp_goal'].copy();before[7]=max(before[7],2048)
        self.assertTrue(np.array_equal(next(q for phase,q in self.rows if phase==p.HOLD),before))
    def test_timeout_posture_can_recover_within_motion_budget(self):
        self.assertLess((len(self.rows)+40)*self.plan.SAMPLE_PERIOD+10,self.plan.ON_TIMEOUT)
        prefix=[q for phase,q in self.rows if phase in (p.ENTRY,p.ROOTS,p.JOIN,p.OPEN)]
        self.assertLessEqual(max(q[0] for q in prefix),max(self.a[0],np.load(p.OPEN_PATH)['open_goal'][0]))
    def test_no_writes_without_review_but_emergency_off_remains(self):
        b=live.NewPairBus.__new__(live.NewPairBus);b.report={}
        with patch.object(checked.Bus,'write',return_value=None) as write:
            for addr,size,value in [(84,2,np.full(16,600)),(64,1,np.ones(16)),(116,4,self.a)]:
                with self.assertRaises(RuntimeError):b.write(addr,size,value)
            write.assert_not_called();b.write(64,1,np.zeros(16));write.assert_called_once()
    def test_exact_queue_consumed_once_no_resend(self):
        b=live.NewPairBus.__new__(live.NewPairBus);b.report={};b.screen_approved=True;b.plan_module=self.plan;b.bind(self.a)
        with patch.object(checked.Bus,'write',return_value=None) as write:
            b.write(116,4,self.a);b.adopt(self.a);b.write(116,4,self.a);target=self.rows[0][1];b.write(116,4,target)
            count=write.call_count
            with self.assertRaises(ValueError):b.write(116,4,target)
            self.assertEqual(write.call_count,count)
    def test_actual_cmc_needed_before_roots(self):
        b=live.NewPairBus.__new__(live.NewPairBus);b.report={'phase':p.ROOTS};b.screen_approved=True;b.adopted=True;b.last_reviewed_state={'position':self.a.copy()}
        with patch.object(live.GuardedBus,'write') as write:
            with self.assertRaises(RuntimeError):b.write(116,4,self.a)
            write.assert_not_called();b.last_reviewed_state['position'][3]=cmc_native_min()+4;b.write(116,4,self.a);write.assert_called_once()
    def test_actual_support_clearance_needed_before_open(self):
        b=live.NewPairBus.__new__(live.NewPairBus);b.report={'phase':p.OPEN};b.screen_approved=True;b.adopted=True;b.support_checked=False;b.root_release_threshold=np.array([2000,2100,2000]);b.last_reviewed_state={'position':self.a.copy()}
        b.last_reviewed_state['position'][[7,11,15]]=b.root_release_threshold-1
        with patch.object(live.GuardedBus,'write') as write:
            with self.assertRaises(RuntimeError):b.write(116,4,self.a)
            write.assert_not_called();b.last_reviewed_state['position'][[7,11,15]]=b.root_release_threshold;b.write(116,4,self.a);write.assert_called_once()
    def test_healthy_sample_failure_propagates(self):
        b=SimpleNamespace(plan_module=self.plan)
        with patch.object(live,'_healthy',side_effect=RuntimeError('undervoltage')):
            with self.assertRaisesRegex(RuntimeError,'undervoltage'):live.sample(b,self.a,{},p.GRASP)
    def test_checkpoints_use_plan_health_sampler_and_hold_guard(self):
        state={'position':self.a.copy()};guard=SimpleNamespace(check=lambda *args:None);report={};seen=[]
        def sample(bus,command,report,phase,**kw):seen.append(phase);kw['snapshot_check'](state);return state,np.zeros(16)
        b=SimpleNamespace(checked_sample=sample);protocol=NewPairGraspCapture(None,None,cancel=lambda:None)
        protocol._observer(b,self.a,report,guard)('place_cube');self.assertEqual(seen,['place_cube'])
    def test_remove_needs_placement_and_finished_open_queue(self):
        protocol=NewPairGraspCapture(None,None,cancel=lambda:None);opened=np.load(p.OPEN_PATH)['open_goal'];b=SimpleNamespace(pending=[self.a])
        with self.assertRaises(RuntimeError):protocol.finish(b,opened,{})
        with self.assertRaisesRegex(RuntimeError,'Open endpoint'):protocol.finish(b,self.a,{'cube_placement_confirmed':True})
        with self.assertRaisesRegex(RuntimeError,'queue incomplete'):protocol.finish(b,opened,{'cube_placement_confirmed':True})
    def test_no_place_prompt_at_unopened_endpoint(self):
        protocol=NewPairGraspCapture(None,None,cancel=lambda:None)
        with self.assertRaisesRegex(RuntimeError,'Open endpoint'):protocol.place(None,self.a,{},None)
    def test_placement_space_after_90_seconds_is_accepted_with_health_reads(self):
        clock=[0.];seen=[];keys=SimpleNamespace(arm=lambda:None,consume_space=lambda:clock[0]>=120)
        protocol=NewPairGraspCapture(None,keys,cancel=lambda:None,now=lambda:clock[0],sleep=lambda seconds:clock.__setitem__(0,clock[0]+seconds))
        with contextlib.redirect_stdout(io.StringIO()):state=protocol._space('place_cube',lambda phase:seen.append(phase) or {'position':self.a})
        self.assertGreaterEqual(clock[0],120);self.assertGreater(len(seen),5900);self.assertTrue(np.array_equal(state['position'],self.a));self.assertGreaterEqual(self.plan.PLACEMENT_PAUSE_LIMIT,NewPairGraspCapture.PLACEMENT_TIMEOUT+10)
    def test_longer_wait_still_stops_for_unhealthy_motor(self):
        protocol=NewPairGraspCapture(None,SimpleNamespace(arm=lambda:None),cancel=lambda:None)
        def unhealthy(phase):raise RuntimeError('low voltage')
        with self.assertRaisesRegex(RuntimeError,'low voltage'):protocol._space('place_cube',unhealthy)

class ReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not p.PROOF.exists():raise unittest.SkipTest('full model review pending')
        cls.plan=p.NewPairGraspPlan(p.REQUEST);cls.review=gate.validate_proof(cls.plan,p.PROOF)
    def mutated(self,fn):
        r=copy.deepcopy(self.review);fn(r)
        with tempfile.TemporaryDirectory(prefix='pair_gate_test_',dir=p.ROOT) as temp:
            directory=Path(temp).resolve();self.assertIn(p.ROOT,directory.parents);path=directory/'review.json';path.write_text(json.dumps(r),encoding='utf-8')
            with self.assertRaises((RuntimeError,ValueError,KeyError,TypeError)):gate.validate_proof(self.plan,path)
    def test_valid_review_matches(self):self.assertTrue(self.review['accepted_for_operator_grasp_trial'])
    def test_reject_incomplete(self):self.mutated(lambda r:r['loaded_physics'].pop())
    def test_reject_duplicate_case(self):self.mutated(lambda r:r['loaded_physics'].__setitem__(0,r['loaded_physics'][1]))
    def test_reject_nan(self):self.mutated(lambda r:r['loaded_physics'][0].__setitem__('max_cube_mm',float('nan')))
    def test_reject_native_loaded_excess(self):self.mutated(lambda r:r['loaded_physics'][0].__setitem__('unapproved_range_excess_rad',.01))
    def test_reject_code_change(self):self.mutated(lambda r:r['code_hashes'].__setitem__('hand_motor_mapping.py','0'*64))
    def test_reject_different_loaded_cycle(self):self.mutated(lambda r:r.__setitem__('loaded_cycle_sha256','0'*64))
    def test_cache_only_reuses_identical_loaded_cycle(self):
        self.assertEqual(len(gate.reusable_loaded_review(self.plan)),27)
        with patch.object(gate,'loaded_digest',return_value='bad'):
            with self.assertRaises(RuntimeError):gate.reusable_loaded_review(self.plan)

if __name__=='__main__':unittest.main(verbosity=2)
