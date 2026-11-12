"""Offline continuation and hardware-boundary regression checks. No devices."""
import ast,copy,itertools,json,subprocess,sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import reverse_quarter_resume_empty_plan_20261025 as module
from reverse_quarter_resume_empty_plan_20261025 import ReverseQuarterResumeEmptyPlan,ROOT,resume_goals,START_HOLD,TURN_PHASE,END_HOLD
from reverse_quarter_resume_empty_gate_20261025 import validate_proof
from leap_reverse_quarter_resume_empty_20261025 import ResumeBus
from leap_book_cube_grasp_trial import Bus as BookBus
from book_grasp_trial_plan import RING_RELEASE
from book_empty_preparation_plan import LIFT
import quarter_motion_runner_20261025 as runner
FIXTURE=ROOT/'right_hand_rotation_20261024/reverse_quarter_resume_empty/fixture_v4'

class Checks(unittest.TestCase):
    def setUp(self):
        self.plan=ReverseQuarterResumeEmptyPlan(FIXTURE/'request.json')
        self.proof=json.loads((FIXTURE/'screen.json').read_bytes())
    def rejected(self,mutation):
        mutation(self.proof)
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'proof.json';p.write_text(json.dumps(self.proof),encoding='utf-8')
            with self.assertRaises(RuntimeError):validate_proof(self.plan,p)
    def reject_goals(self,mutation):
        meta=json.loads((ROOT/module.GOALS).read_bytes());mutation(meta)
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'goals.json';p.write_text(json.dumps(meta),encoding='utf-8')
            with patch.object(module,'GOALS',str(p)):
                with self.assertRaises(ValueError):resume_goals()
    def test_complete_fixture(self):
        r=validate_proof(self.plan,FIXTURE/'screen.json')
        self.assertTrue(r['interrupted_suffix_verified']);self.assertTrue(r['complete_recovery_and_suffix_checked'])
        self.assertFalse(r['full_quarter_reference_path'])
    def test_exact_unexecuted_suffix_instead_of_planned_final_goal(self):
        meta=json.loads((ROOT/module.GOALS).read_bytes())
        actual=json.loads((ROOT/meta['source_actual_trial']).read_bytes())
        full=np.asarray(json.loads((ROOT/meta['source_full_goals']).read_bytes())['raw_goal_ticks'])
        self.assertEqual(meta['resumed_original_waypoint'],472)
        np.testing.assert_array_equal(resume_goals(),full[472:])
        self.assertEqual(len(resume_goals())-1,288)
        self.assertEqual(resume_goals()[0].tolist(),actual['samples'][-1]['goal_readback'])
        self.assertNotEqual(resume_goals()[0].tolist(),actual['final_command'])
    def test_reject_changed_last_issued_goal(self):self.reject_goals(lambda r:r['raw_goal_ticks'][0].__setitem__(0,r['raw_goal_ticks'][0][0]+1))
    def test_reject_changed_suffix(self):self.reject_goals(lambda r:r['raw_goal_ticks'][1].__setitem__(0,r['raw_goal_ticks'][1][0]+1))
    def test_reject_changed_actual_record_hash(self):self.reject_goals(lambda r:r.update(source_actual_trial_sha256='0'*64))
    def test_reject_wrong_resume_index(self):self.reject_goals(lambda r:r.update(resumed_original_waypoint=473))
    def test_bounds_holds_no_cube_and_recovery_order(self):
        raw=self.plan.source_position();rows=self.plan.plan(raw)
        self.assertFalse(self.plan.CAPTURE_PLACEMENT);self.assertIsNone(self.plan.CAPTURE_BEFORE_PHASE)
        self.assertEqual(rows[0][0],RING_RELEASE)
        first_join=next(i for i,(p,q) in enumerate(rows) if p==LIFT)
        self.assertTrue(all(p==RING_RELEASE for p,q in rows[:first_join]))
        self.assertLessEqual(rows[first_join-1][1][14],raw[14]-64)
        np.testing.assert_array_equal(np.array([q for p,q in rows if p==TURN_PHASE]),resume_goals()[1:])
        self.assertEqual(sum(p==START_HOLD for p,q in rows),40)
        self.assertEqual(sum(p==END_HOLD for p,q in rows),20)
        values=np.vstack([raw]+[q for p,q in rows]);limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
        self.assertLessEqual(abs(np.diff(values,axis=0)).max(),2)
        self.assertTrue(np.all(values>=limits['lower_ticks']));self.assertTrue(np.all(values<=limits['upper_ticks']))
        self.assertLess((len(rows)+20)*.06+2,self.plan.ON_TIMEOUT)
    def test_reject_placement_claim(self):self.rejected(lambda r:r.update(placement_checkpoint_required=True))
    def test_reject_loaded_rotation_claim(self):self.rejected(lambda r:r.update(rotation_included=True))
    def test_reject_missing_static_case(self):self.rejected(lambda r:r['static_screens'].pop())
    def test_reject_missing_native_case(self):self.rejected(lambda r:r['dynamic_sensitivity'].pop())
    def test_reject_missing_housing(self):self.rejected(lambda r:r['model_manifests'][0]['middle_ring_proxies'].pop())
    def test_reject_post_release_self_contact(self):self.rejected(lambda r:r['dynamic_sensitivity'][0].update(max_self_after_release_mm=.001))
    def test_reject_extended_escape_deadline(self):self.rejected(lambda r:r['dynamic_sensitivity'][0].update(support_escape_deadline=20.))
    def test_reject_enlarged_ring_neighbor_bound(self):self.rejected(lambda r:r.update(initial_ring_neighbor_limit_mm=1.6))
    def test_reject_above_existing_passive_contact_limit(self):
        def changed(r):
            s=r['dynamic_sensitivity'][0];pair='mf_bs_visual_support_proxy|support_glass'
            s['passive_enable_hold_support']['contact_maximums_mm'][pair]=.151
            s['initial_support_contacts_mm'][pair]=.151
        self.rejected(changed)
    def test_passive_contact_is_classified_from_nominal_origin(self):
        r=validate_proof(self.plan,FIXTURE/'screen.json');s=r['dynamic_sensitivity'][0];pair='mf_bs_visual_support_proxy|support_glass'
        self.assertNotIn(pair,s['nominal_initial_support_contacts_mm'])
        static=[c['initial_support_contacts_mm'].get(pair,0) for c in r['static_screens'] if c['book_front_mm']==s['book_front_mm']]
        depth=s['passive_enable_hold_support']['contact_maximums_mm'][pair]
        self.assertGreater(depth,max(static));self.assertLessEqual(depth,.15)
        self.assertEqual(s['initial_support_contacts_mm'][pair],depth)
    def test_reject_forged_nominal_contact_origin(self):self.rejected(lambda r:r['dynamic_sensitivity'][0]['nominal_initial_support_contacts_mm'].clear())
    def test_no_join_before_fresh_measured_ring_release(self):
        bus=ResumeBus.__new__(ResumeBus);bus.approve(self.plan,FIXTURE/'screen.json');bus.report={}
        bus._goal_writes=2+bus._ring_release_writes
        bus.latest_release_state={'position':self.plan.source_position()};bus.latest_release_read_at=time.monotonic()
        with patch.object(BookBus,'write') as fallback:
            with self.assertRaisesRegex(RuntimeError,'Ring release not measured'):bus.write(116,4,resume_goals()[0])
            fallback.assert_not_called()
            bus.latest_release_state['position'][14]=bus._ring_release_threshold;bus.latest_release_read_at=time.monotonic()-1
            with self.assertRaisesRegex(RuntimeError,'No fresh'):bus.write(116,4,resume_goals()[0])
            fallback.assert_not_called()
            bus.latest_release_read_at=time.monotonic();bus.write(116,4,resume_goals()[0]);fallback.assert_called_once()
            self.assertTrue(bus.report['ring_release_measured_verified'])
    def test_safety_device_calls_preserved_from_reviewed_runner(self):
        names={'bus.off','bus.write','bus.snapshot','bus.bind','bus.adopt','check_off','checked.health','checked.sample','held_guard.check','held_guard.advance'}
        def signature(path):
            tree=ast.parse(path.read_text(encoding='utf-8'))
            f=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='run')
            return [ast.dump(n) for n in ast.walk(f) if isinstance(n,ast.Call) and ast.unparse(n.func) in names]
        self.assertEqual(signature(ROOT/'leap_thumb_prefix_empty.py'),signature(ROOT/'quarter_motion_runner_20261025.py'))
    def test_health_fault_stops_writes_restores_and_records_actual_last_command(self):
        raw=self.plan.source_position();plan=self.plan
        original={'position':raw.copy(),'goal':raw.copy()}
        original.update({k:np.full(16,i+100) for i,k in enumerate(('current_cap','acceleration','velocity','watchdog','d_gain','i_gain','p_gain'))})
        class FakeBus:
            plan_module=plan
            def __init__(self):self.state=copy.deepcopy(original);self.writes=[];self.off_count=0
            def bind(self,a):pass
            def adopt(self,a):pass
            def snapshot(self):return copy.deepcopy(self.state)
            def write(self,addr,size,values,verify=True):
                self.writes.append((addr,size,np.asarray(values).copy()))
                if addr==116:self.state['goal']=np.asarray(values).copy();self.state['position']=np.asarray(values).copy()
            def off(self):self.off_count+=1
        bus=FakeBus();report={};calls=[];ticks=itertools.count()
        def sample(b,command,r,phase,snapshot_check):
            calls.append(phase)
            if len(calls)==4:raise RuntimeError('injected actual health fault')
            state=b.snapshot();snapshot_check(state);return state,np.zeros(16)
        with patch.object(runner,'check_off'),patch.object(runner.checked,'health'),patch.object(runner.checked,'sample',side_effect=sample):
            with self.assertRaisesRegex(RuntimeError,'actual health fault'):
                runner.run(bus,original,report,cancel=lambda:None,sleep=lambda x:None,now=lambda:next(ticks)*.01)
        issued=[v for a,s,v in bus.writes if a==116];rows=plan.plan(raw)
        self.assertEqual(len(issued),5);np.testing.assert_array_equal(issued[-1],rows[2][1])
        self.assertEqual(report['last_issued_command'],issued[-1].tolist());self.assertEqual(report['final_command'],issued[-1].tolist())
        self.assertEqual(report['planned_final_command'],rows[-1][1].tolist());self.assertNotEqual(report['final_command'],report['planned_final_command'])
        self.assertEqual(bus.off_count,1);self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])
        for (a,size,key),(got,size2,value) in zip(((102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),(98,1,'watchdog'),(80,2,'d_gain'),(82,2,'i_gain'),(84,2,'p_gain')),bus.writes[-7:]):
            self.assertEqual((a,size),(got,size2));np.testing.assert_array_equal(value,original[key])
    def test_default_is_device_free(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_reverse_quarter_resume_empty_20261025.py')],capture_output=True,text=True,encoding='utf-8',timeout=5)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)

if __name__=='__main__':unittest.main()
