"""Device-free PLACE/REMOVE, accepted targets and fresh release checks."""
import contextlib,copy,io,itertools,json,subprocess,sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np
import quarter_motion_runner_20261025 as runner
from loaded_quarter_finish_plan_20261025 import (LoadedQuarterFinishPlan,ROOT,FIXTURE,TURN_PHASE,FINISH_PHASE,END_HOLD,motion_goals,finish_goals)
from loaded_quarter_finish_protocol_20261025 import LoadedQuarterFinishCapture
from loaded_quarter_finish_gate_20261025 import validate_proof
from leap_loaded_quarter_finish_trial_20261025 import LoadedFinishBus,EmptyFinishBus
from test_quarter_support_photo_20261025 import FakeBus
class Bus(FakeBus):
    def __init__(self,plan,report):super().__init__(plan,report);self.state['current']=np.zeros(16,dtype=int);self.loaded_goals=[]
    def write(self,addr,size,v,verify=True):
        if addr==116 and self.report.get('phase') in (TURN_PHASE,FINISH_PHASE):
            if not self.report.get('cube_placement_confirmed'):raise AssertionError('Loaded target before PLACE confirmed')
            self.loaded_goals.append(np.asarray(v).copy())
        return super().write(addr,size,v,verify=verify)
def protocol(bus,fail):
    p=LoadedQuarterFinishCapture.__new__(LoadedQuarterFinishCapture);p._observer=Mock(return_value=Mock());clock=itertools.count();p.now=lambda:next(clock)*.25;p.sleep=lambda _:None
    def photo(name,phase,observe):return bus.snapshot(),name
    def space(phase,observe):
        if phase=='place_cube':
            if bus.loaded_goals:raise AssertionError('Goals advanced while waiting at PLACE')
            np.testing.assert_array_equal(bus.state['goal'],motion_goals()[0])
        if fail==phase:raise KeyboardInterrupt('Synthetic checkpoint cancellation')
        return bus.snapshot()
    p._photo=Mock(side_effect=photo);p._space=Mock(side_effect=space);return p
class LoadedFinishChecks(unittest.TestCase):
    def setUp(self):self.plan=LoadedQuarterFinishPlan(FIXTURE/'request.json')
    def run_path(self,fail=None):
        report={};bus=Bus(self.plan,report);initial=bus.snapshot();p=protocol(bus,fail);clock=itertools.count()
        def sample(b,c,r,phase,snapshot_check):
            if fail=='health' and phase==FINISH_PHASE:raise RuntimeError('Synthetic unhealthy read before new finish')
            s=b.snapshot();snapshot_check(s);return s,np.zeros(16)
        with patch.object(runner,'check_off'),patch.object(runner.checked,'health'),patch.object(runner.checked,'sample',side_effect=sample),contextlib.redirect_stdout(io.StringIO()):
            if fail:
                with self.assertRaises(RuntimeError if fail=='health' else KeyboardInterrupt):runner.run(bus,initial,report,cancel=lambda:None,sleep=lambda _:None,now=lambda:next(clock)*.002,phase_callback=p.place,prepared_callback=p.finish)
            else:runner.run(bus,initial,report,cancel=lambda:None,sleep=lambda _:None,now=lambda:next(clock)*.002,phase_callback=p.place,prepared_callback=p.finish)
        return report,bus,p,initial
    def test_targets_match_completed_empty_reference_and_finish(self):
        rows=self.plan.plan(self.plan.source_position());np.testing.assert_array_equal([q for p,q in rows if p==TURN_PHASE],motion_goals()[1:]);np.testing.assert_array_equal([q for p,q in rows if p==FINISH_PHASE],finish_goals()[1:])
        self.assertTrue(self.plan.CAPTURE_PLACEMENT);self.assertEqual(self.plan.CAPTURE_BEFORE_PHASE,TURN_PHASE);self.assertEqual(sum(p==END_HOLD for p,q in rows),60);self.assertLessEqual(np.max(abs(np.diff(np.vstack([self.plan.source_position()]+[q for p,q in rows]),axis=0))),2)
    def test_place_precedes_every_loaded_goal_and_removal_precedes_completion(self):
        r,bus,p,initial=self.run_path();np.testing.assert_array_equal(bus.loaded_goals,np.vstack([motion_goals()[1:],finish_goals()[1:]]));self.assertEqual([c.args[0] for c in p._space.call_args_list],['place_cube','remove_cube'])
        self.assertTrue(r['cube_removal_confirmed']);self.assertFalse(r['cube_present']);self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored']);self.assertEqual(r['status'],self.plan.STATUS);self.assertFalse(r['physical_flip_verified'])
        for addr,size,key in ((102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),(98,1,'watchdog'),(80,2,'d_gain'),(82,2,'i_gain'),(84,2,'p_gain')):np.testing.assert_array_equal(next(q for a,q in reversed(bus.writes) if a==addr),initial[key])
    def test_cancel_at_place_sends_no_loaded_goal_and_restores(self):
        r,bus,_,_=self.run_path('place_cube');self.assertEqual(len(bus.loaded_goals),0);self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored']);self.assertNotIn('cube_placement_confirmed',r)
    def test_failed_health_read_sends_no_new_finish_goal(self):
        r,bus,_,_=self.run_path('health');self.assertEqual(len(bus.loaded_goals),760);self.assertEqual(r['last_issued_command'],motion_goals()[-1].tolist());self.assertTrue(r['settings_restored']);self.assertNotIn('cube_removal_confirmed',r)
    def test_remove_not_confirmed_is_not_recorded_as_removed(self):
        r,bus,_,_=self.run_path('remove_cube');self.assertEqual(len(bus.loaded_goals),824);self.assertTrue(r['all_torque_off_verified']);self.assertNotIn('cube_removal_confirmed',r);self.assertNotEqual(r.get('status'),self.plan.STATUS)
    def test_native_proof_requires_place_and_the_verified_finish(self):
        r=validate_proof(self.plan,FIXTURE/'screen.json');self.assertTrue(r['placement_checkpoint_required']);self.assertTrue(r['cube_disabled_entire_screen']);self.assertFalse(r['cube_contact_verified'])
        for key,value in (('placement_checkpoint_required',False),('cube_disabled_entire_screen',False),('empty_root_finish_goal_count',63)):
            wrong=copy.deepcopy(r);wrong[key]=value
            with tempfile.TemporaryDirectory() as d:
                p=Path(d)/'proof.json';p.write_text(json.dumps(wrong),encoding='utf-8')
                with self.assertRaises(RuntimeError):validate_proof(self.plan,p)
    def test_stalled_thumb_mcp_cannot_advance_the_next_target(self):
        b=LoadedFinishBus.__new__(LoadedFinishBus);b._ring_release_writes=32;b._goal_writes=34;b.last_goal_command=np.zeros(16,dtype=int);b.latest_release_state={'position':np.zeros(16,dtype=int)};b.latest_release_state['position'][1]=21;b.latest_release_read_at=time.monotonic();b.report={}
        with patch.object(EmptyFinishBus,'write') as parent:
            with self.assertRaisesRegex(RuntimeError,'MCP ID2'):b.write(116,4,np.zeros(16,dtype=int))
            parent.assert_not_called()
    def test_fresh_followed_thumb_mcp_can_reach_parent_guards(self):
        b=LoadedFinishBus.__new__(LoadedFinishBus);b._ring_release_writes=32;b._goal_writes=34;b.last_goal_command=np.zeros(16,dtype=int);b.latest_release_state={'position':np.zeros(16,dtype=int)};b.latest_release_read_at=time.monotonic();b.report={}
        with patch.object(EmptyFinishBus,'write') as parent:b.write(116,4,np.zeros(16,dtype=int));parent.assert_called_once()
        self.assertTrue(b.report['thumb_mcp_release_measured_verified'])
    def test_default_does_not_open_devices(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_loaded_quarter_finish_trial_20261025.py')],capture_output=True,text=True,encoding='utf-8',timeout=8);self.assertEqual(r.returncode,0,r.stderr);self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)
if __name__=='__main__':unittest.main()
