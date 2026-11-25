"""Device-free tests of the experimental loaded transition and its evidence."""
import contextlib,copy,io,itertools,json,subprocess,sys,tempfile,time,unittest
from unittest.mock import Mock,patch
from pathlib import Path
import numpy as np
from loaded_quarter_trial_plan_20261025 import LoadedQuarterTrialPlan,ROOT,RETURN_PHASE,START_HOLD,TURN_PHASE,END_HOLD,motion_goals,empty_return_goals,validate_experiment_basis
from loaded_quarter_trial_gate_20261025 import validate_proof
from loaded_quarter_trial_protocol_20261025 import LoadedQuarterCapture
from leap_loaded_quarter_trial_20261025 import LoadedQuarterBus
from leap_book_cube_grasp_trial import Bus as BaseBus
from book_grasp_trial_plan import RING_RELEASE
import quarter_motion_runner_20261025 as runner
FIXTURE=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/fixture_v3'

class Checks(unittest.TestCase):
    def setUp(self):
        self.plan=LoadedQuarterTrialPlan(FIXTURE/'request.json');self.proof=json.loads((FIXTURE/'screen.json').read_bytes())
    def rejected(self,mutation):
        mutation(self.proof)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'proof.json';path.write_text(json.dumps(self.proof),encoding='utf-8')
            with self.assertRaises(RuntimeError):validate_proof(self.plan,path)
    def test_complete_current_fixture(self):
        r=validate_proof(self.plan,FIXTURE/'screen.json')
        self.assertTrue(r['experimental_loaded_trial']);self.assertTrue(r['cube_disabled_entire_screen'])
    def test_same_full_loaded_goals_bounded_empty_return_and_timing(self):
        raw=self.plan.source_position();rows=self.plan.plan(raw);original=motion_goals()
        np.testing.assert_array_equal(np.array([q for p,q in rows if p==TURN_PHASE]),original[1:])
        self.assertEqual(sum(p==TURN_PHASE for p,q in rows),760)
        self.assertTrue(self.plan.CAPTURE_PLACEMENT);self.assertEqual(self.plan.CAPTURE_BEFORE_PHASE,TURN_PHASE)
        self.assertEqual(sum(p==START_HOLD for p,q in rows),40);self.assertEqual(sum(p==END_HOLD for p,q in rows),60)
        anchor=int(np.argmin(np.max(abs(original-raw),axis=1)));back=empty_return_goals(anchor)
        np.testing.assert_array_equal(np.array([q for p,q in rows if p==RETURN_PHASE]),back[1:])
        np.testing.assert_array_equal(back[-1],original[0]);self.assertLessEqual(abs(np.diff(back,axis=0)).max(),2)
        # Other joints are the known reverse path; the thumb clearance is <=16 ticks.
        for q in back:
            same=np.flatnonzero(np.all(original[:anchor+1,1:]==q[1:],axis=1))
            self.assertTrue(len(same));self.assertTrue(any(0<=original[i,0]-q[0]<=16 for i in same))
        values=np.vstack([raw]+[q for p,q in rows]);limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
        self.assertLessEqual(abs(np.diff(values,axis=0)).max(),2)
        self.assertTrue(np.all(values>=limits['lower_ticks']));self.assertTrue(np.all(values<=limits['upper_ticks']))
        self.assertLess((len(rows)+40)*.06+2,self.plan.ON_TIMEOUT)
    def test_model_is_conditional_not_physical_or_robust_success(self):
        r=validate_experiment_basis();self.assertFalse(r['physical_cube_rotation_verified'])
        self.assertFalse(r['robust_loaded_controller_verified']);self.assertTrue(r['prescribed_loaded_hand_motion_is_an_assumption'])
    def test_reject_missing_place_gate(self):self.rejected(lambda r:r.update(placement_checkpoint_required=False))
    def test_reject_incorrect_native_placement_hold(self):self.rejected(lambda r:r.update(native_placement_hold_samples=0))
    def test_reject_false_loaded_response_claim(self):self.rejected(lambda r:r['experimental_model_review'].update(physical_cube_rotation_verified=True))
    def test_reject_missing_uncertainty_case(self):self.rejected(lambda r:r['dynamic_sensitivity'].pop())
    def test_reject_new_contact_after_release(self):self.rejected(lambda r:r['dynamic_sensitivity'][0].update(max_self_after_release_mm=.001))
    def test_reject_larger_initial_thumb_corner_bound(self):self.rejected(lambda r:r.update(initial_thumb_neighbor_limit_mm=.6))
    def test_reject_forged_self_envelope(self):self.rejected(lambda r:r['dynamic_sensitivity'][0]['initial_self_contacts_mm'].clear())
    def test_reject_empty_only_program_without_place_callback(self):
        self.plan.CAPTURE_PLACEMENT=False
        with self.assertRaisesRegex(RuntimeError,'PLACE/REMOVE'):validate_proof(self.plan,FIXTURE/'screen.json')
    def test_both_tips_must_be_measured_clear_before_next_goal(self):
        bus=LoadedQuarterBus.__new__(LoadedQuarterBus);bus.approve(self.plan,FIXTURE/'screen.json');bus.report={}
        bus._goal_writes=2+bus._ring_release_writes;bus.latest_release_state={'position':self.plan.source_position()};bus.latest_release_read_at=time.monotonic()
        with patch.object(BaseBus,'write') as fallback:
            with self.assertRaisesRegex(RuntimeError,'Thumb-tip release not measured'):bus.write(116,4,motion_goals()[0])
            fallback.assert_not_called();bus.latest_release_state['position'][0]=bus._tip_release_threshold
            with self.assertRaisesRegex(RuntimeError,'Ring release not measured'):bus.write(116,4,motion_goals()[0])
            fallback.assert_not_called();bus.latest_release_state['position'][14]=bus._ring_release_threshold;bus.latest_release_read_at=time.monotonic()-1
            with self.assertRaisesRegex(RuntimeError,'No fresh'):bus.write(116,4,motion_goals()[0])
            fallback.assert_not_called();bus.latest_release_read_at=time.monotonic();bus.write(116,4,motion_goals()[0]);fallback.assert_called_once()
            self.assertTrue(bus.report['ring_release_measured_verified']);self.assertTrue(bus.report['thumb_tip_release_measured_verified'])
    def test_place_and_remove_callbacks_do_not_send_goals(self):
        state={'position':motion_goals()[0].copy(),'current':np.zeros(16,dtype=int)}
        protocol=LoadedQuarterCapture.__new__(LoadedQuarterCapture);clock=itertools.count()
        protocol.now=lambda:next(clock)*.1;protocol.sleep=lambda x:None
        observe=Mock(return_value=state);protocol._observer=Mock(return_value=observe)
        protocol._photo=Mock(return_value=(state,'saved_photo.jpg'));protocol._space=Mock(return_value=state)
        bus=Mock();report={'physical_flip_verified':False};command=motion_goals()[0]
        with contextlib.redirect_stdout(io.StringIO()):
            protocol.place(bus,command,report,Mock());self.assertTrue(report['cube_placement_confirmed']);self.assertTrue(report['cube_present'])
            protocol.finish(bus,command,report)
        self.assertEqual([c.args[0] for c in protocol._space.call_args_list],['place_cube','remove_cube'])
        bus.write.assert_not_called();bus.off.assert_not_called();self.assertTrue(report['cube_removal_confirmed']);self.assertFalse(report['physical_flip_verified'])
    def test_runtime_never_sends_loaded_goal_before_place_confirmation(self):
        plan=self.plan;raw=plan.source_position();state={'position':raw.copy(),'goal':raw.copy()}
        state.update({k:np.full(16,i+100) for i,k in enumerate(('current_cap','acceleration','velocity','watchdog','d_gain','i_gain','p_gain'))})
        report={};callbacks=[];count=itertools.count()
        class FakeBus:
            plan_module=plan
            def __init__(self):self.state=copy.deepcopy(state);self.loaded_count=0;self.off_count=0
            def bind(self,a):pass
            def adopt(self,a):pass
            def snapshot(self):return copy.deepcopy(self.state)
            def write(self,addr,size,v,verify=True):
                if addr==116:
                    if report.get('phase')==TURN_PHASE:
                        if not report.get('cube_placement_confirmed'):raise AssertionError('loaded command before PLACE')
                        self.loaded_count+=1
                    self.state['position']=np.asarray(v).copy();self.state['goal']=np.asarray(v).copy()
            def off(self):self.off_count+=1
        bus=FakeBus()
        def sample(b,command,r,phase,snapshot_check):
            s=b.snapshot();snapshot_check(s);return s,np.zeros(16)
        def place(b,c,r,g):
            self.assertEqual(b.loaded_count,0);callbacks.append('PLACE');r['cube_placement_confirmed']=True
        def remove(b,c,r):
            self.assertEqual(b.loaded_count,760);callbacks.append('REMOVE');r['cube_removal_confirmed']=True
        with patch.object(runner,'check_off'),patch.object(runner.checked,'health'),patch.object(runner.checked,'sample',side_effect=sample),contextlib.redirect_stdout(io.StringIO()):
            runner.run(bus,state,report,cancel=lambda:None,sleep=lambda x:None,now=lambda:next(count)*.002,phase_callback=place,prepared_callback=remove)
        self.assertEqual(callbacks,['PLACE','REMOVE']);self.assertEqual(bus.off_count,1);self.assertTrue(report['settings_restored'])
        self.assertEqual(report['status'],plan.STATUS)
    def test_default_opens_no_devices(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_loaded_quarter_trial_20261025.py')],capture_output=True,text=True,encoding='utf-8',timeout=5)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)

if __name__=='__main__':unittest.main()
