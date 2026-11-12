"""Loaded checkpoints, camera scope, shutdown and strict voltage boundaries."""
import contextlib,io,itertools,json,subprocess,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np
import camera_gap_motion_runner_20261026 as runner
import test_fresh_voltage_sample_20261026 as voltage_tests
from test_quarter_support_photo_20261025 import FakeBus
from loaded_thumb_rotation_plan_20261026 import LoadedThumbRotationPlan,FIXTURE,TURN_PHASE,ROOT,ROOT_UP
from loaded_thumb_rotation_protocol_20261026 import LoadedThumbCapture
from loaded_thumb_rotation_camera_20261026 import LoadedThumbCamera,BeforePlacementGuard
from recovering_empty_camera_20261026 import RecoveringEmptyCamera
from quarter_finish_empty_camera_20261025 import QuarterFinishEmptyCamera
from camera_frame_signal import CameraImageUnavailable
import loaded_thumb_voltage_sample_20261026 as loaded_voltage
from loaded_thumb_return_clearance_20261026 import goals as return_clearance,original_goals,EXTRA_CMC_TICKS
import loaded_thumb_return_model_resolution_20261026 as resolution
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan
from loaded_thumb_rotation_plan_20261026 import REVERSE,RETURN_PHASE,extra_hashes
from loaded_thumb_rotation_gate_20261026 import validate_proof

class EmptyReturnChecks(unittest.TestCase):
    def test_bound_file_keys_are_portable_between_Windows_and_WSL(self):
        self.assertFalse(any('\\' in key for key in extra_hashes()))
    def test_new_clearance_preserves_endpoints_steps_and_other_fingers(self):
        for anchor in (0,1,10,200,600,760):
            original=original_goals(anchor);new=return_clearance(anchor)
            np.testing.assert_array_equal(new[[0,-1]],original[[0,-1]])
            self.assertLessEqual(np.max(abs(np.diff(new,axis=0)),initial=0),2)
            if len(new)==len(original):
                np.testing.assert_array_equal(new[:,np.arange(16)!=3],original[:,np.arange(16)!=3])
                self.assertTrue(np.all((original[:,3]-new[:,3]>=0)&(original[:,3]-new[:,3]<=EXTRA_CMC_TICKS)))
    def test_post_place_path_is_exact_completed_empty_goal_sequence(self):
        p=LoadedThumbRotationPlan(FIXTURE/'request.json');raw=p.source_position()
        base=ThumbRootEmptyPlan.plan(p,raw);actual=p.plan(raw)
        start=next(i for i,(phase,q) in enumerate(actual) if phase==TURN_PHASE)
        expected=[(REVERSE.get(phase,phase),q) for phase,q in actual[start:]]
        from thumb_clear_root_empty_plan_20261026 import TURN_PHASE as EMPTY_TURN
        source_start=next(i for i,(phase,q) in enumerate(base) if phase==EMPTY_TURN)
        self.assertEqual([phase for phase,q in expected],[phase for phase,q in base[source_start:]])
        np.testing.assert_array_equal([q for phase,q in expected],[q for phase,q in base[source_start:]])
    def test_maximum_step_bound_also_holds_at_return_phase_boundaries(self):
        p=LoadedThumbRotationPlan(FIXTURE/'request.json');raw=p.source_position();rows=p.plan(raw)
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([raw]+[q for phase,q in rows]),axis=0))),2)
    def test_ring_resolution_is_limited_to_exact_pair_phase_and_ten_nanometres(self):
        pair=resolution.RING_PAIR;limit=resolution.RING_ZERO_MM
        self.assertEqual(resolution.classify({pair:limit},phase=RETURN_PHASE),{})
        for phase in (None,TURN_PHASE):self.assertEqual(resolution.classify({pair:limit},phase=phase),{pair:limit})
        self.assertEqual(resolution.classify({pair:limit*1.01},phase=RETURN_PHASE),{pair:limit*1.01})
        self.assertFalse(resolution.accepted_discard('palm_collision_5|rf_tip',limit))
        self.assertFalse(resolution.accepted_discard(pair,float('nan')))

@unittest.skipUnless((FIXTURE/'screen.json').exists(),'Full native proof not yet generated')
class SavedProofChecks(unittest.TestCase):
    def check_mutation(self,mutate):
        r=json.loads((FIXTURE/'screen.json').read_bytes());mutate(r)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'changed.json';path.write_text(json.dumps(r),encoding='utf-8')
            with self.assertRaises(RuntimeError):validate_proof(LoadedThumbRotationPlan(FIXTURE/'request.json'),path)
    def test_final_native_proof_is_accepted_on_Windows(self):
        self.assertTrue(validate_proof(LoadedThumbRotationPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')['experimental_loaded_trial'])
    def test_changed_code_hash_cannot_reuse_saved_proof(self):
        self.check_mutation(lambda r:r['extra_code_hashes'].__setitem__('loaded_thumb_rotation_plan_20261026.py','0'*64))
    def test_camera_recovery_cannot_be_extended_into_placement(self):
        self.check_mutation(lambda r:r['camera_gap_hold_policy'].__setitem__('scope','including loaded motion'))
    def test_larger_or_unknown_ring_contact_cannot_be_accepted_as_resolution(self):
        self.check_mutation(lambda r:r['return_corner_discarded_maximums_by_front'][0].__setitem__(resolution.RING_PAIR,resolution.RING_ZERO_MM*1.01))

class CameraScopeChecks(unittest.TestCase):
    def test_recovery_is_enabled_only_before_placement(self):
        report={};guard=Mock();guard.wait_if_needed.return_value=.4;scoped=BeforePlacementGuard(guard,report)
        self.assertEqual(scoped.wait_if_needed(Mock()),.4)
        report['cube_present']=True;self.assertEqual(scoped.wait_if_needed(Mock()),0)
        guard.wait_if_needed.assert_called_once()
    def test_camera_failure_with_cube_still_stops(self):
        c=LoadedThumbCamera.__new__(LoadedThumbCamera);c.report={'cube_present':True}
        with patch.object(QuarterFinishEmptyCamera,'check',side_effect=CameraImageUnavailable('Synthetic loaded blank')):
            with self.assertRaisesRegex(CameraImageUnavailable,'loaded blank'):c.check()
    def test_empty_preparation_keeps_recovery_check(self):
        c=LoadedThumbCamera.__new__(LoadedThumbCamera);c.report={}
        with patch.object(RecoveringEmptyCamera,'check') as check:c.check();check.assert_called_once()
    def test_placement_window_is_strict_even_before_second_space(self):
        report={'cube_placement_window_open':True};guard=Mock();scoped=BeforePlacementGuard(guard,report)
        self.assertEqual(scoped.wait_if_needed(Mock()),0);guard.wait_if_needed.assert_not_called()
        c=LoadedThumbCamera.__new__(LoadedThumbCamera);c.report=report
        with patch.object(QuarterFinishEmptyCamera,'check',side_effect=CameraImageUnavailable('Placement blank')):
            with self.assertRaisesRegex(CameraImageUnavailable,'Placement blank'):c.check()

class LoadedVoltageChecks(unittest.TestCase):
    def replay(self,values,**kwargs):
        return voltage_tests.VoltageChecks().replay(values,sampler=loaded_voltage,loaded=True,**kwargs)
    def test_two_healthy_reads_fit_original_budget(self):
        r,c,reads,value,error=self.replay([36,37,37,38,38])
        self.assertIsNone(error);self.assertLess(r['voltage_events'][0]['elapsed_ms'],150)
        self.assertEqual(r['voltage_events'][0]['healthy_confirmation_count'],2)
    def test_low_voltage_cutoff_is_preserved(self):
        r,c,reads,value,error=self.replay([35])
        self.assertRegex(str(error),'immediate stop');self.assertEqual(len(reads),1)
    def test_late_recovery_is_not_accepted(self):
        r,c,reads,value,error=self.replay([36,37,37,38,38],duration=.034)
        self.assertIsInstance(error,RuntimeError)
    def test_loaded_sampler_cannot_silently_replace_empty_sampler(self):
        r,c,reads,value,error=voltage_tests.VoltageChecks().replay([38],sampler=loaded_voltage)
        self.assertRegex(str(error),'requires reviewed LOADED');self.assertEqual(reads,[])

class LoadedRunnerChecks(unittest.TestCase):
    def run_case(self,fault=None):
        p=LoadedThumbRotationPlan(FIXTURE/'request.json');report={};bus=FakeBus(p,report);bus.state['current']=np.zeros(16,dtype=int);initial=bus.snapshot();clock=itertools.count()
        now=lambda:next(clock)*.002
        protocol=LoadedThumbCapture.__new__(LoadedThumbCapture);protocol.now=now;protocol.sleep=lambda _:None
        protocol._observer=Mock(return_value=Mock());events=[]
        def photo(name,phase,observe):return bus.snapshot(),dict(image=name)
        def space(phase,observe):
            events.append((phase,sum(addr==116 for addr,q in bus.writes)))
            if (fault=='place' and phase=='place_cube') or (fault=='remove' and phase=='remove_cube'):
                raise KeyboardInterrupt('Synthetic user stop')
            return bus.snapshot()
        protocol._photo=photo;protocol._space=space
        def sample(b,command,r,phase,snapshot_check):
            if fault=='health' and phase==TURN_PHASE:raise RuntimeError('Synthetic loaded health fault')
            state=b.snapshot();snapshot_check(state);return state,np.zeros(16)
        with patch.object(runner,'check_off'),patch.object(runner.checked,'health'),patch.object(runner.checked,'sample',side_effect=sample),contextlib.redirect_stdout(io.StringIO()):
            if fault:
                with self.assertRaises(RuntimeError if fault=='health' else KeyboardInterrupt):
                    runner.run(bus,initial,report,cancel=lambda:None,sleep=lambda _:None,now=now,phase_callback=protocol.place,prepared_callback=protocol.finish)
            else:runner.run(bus,initial,report,cancel=lambda:None,sleep=lambda _:None,now=now,phase_callback=protocol.place,prepared_callback=protocol.finish)
        return p,report,bus,events,initial
    def test_explicit_place_then_all_goals_then_remove(self):
        p,r,b,e,initial=self.run_case();rows=p.plan(p.source_position())
        self.assertEqual([phase for phase,count in e],['place_cube','remove_cube'])
        self.assertEqual(e[0][1],2+next(i for i,(phase,q) in enumerate(rows) if phase==TURN_PHASE))
        np.testing.assert_array_equal([q for addr,q in b.writes if addr==116][2:],[q for phase,q in rows])
        self.assertTrue(r['cube_removal_confirmed']);self.assertFalse(r['cube_present'])
        self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored']);self.assertEqual(r['status'],p.STATUS)
        self.assertFalse(r['physical_flip_verified'])
    def test_no_loaded_goals_when_place_is_cancelled(self):
        p,r,b,e,initial=self.run_case('place');rows=p.plan(p.source_position());count=next(i for i,(phase,q) in enumerate(rows) if phase==TURN_PHASE)
        self.assertEqual(sum(addr==116 for addr,q in b.writes),count+2)
        self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
    def test_loaded_health_failure_still_off_without_first_loaded_goal(self):
        p,r,b,e,initial=self.run_case('health');self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
        self.assertEqual(sum(addr==116 for addr,q in b.writes),e[0][1])
    def test_removal_cancel_is_never_recorded_as_removed(self):
        p,r,b,e,initial=self.run_case('remove');self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
        self.assertNotIn('cube_removal_confirmed',r);self.assertTrue(r['cube_present'])
    def test_default_opens_no_devices(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_loaded_thumb_rotation_20261026.py')],capture_output=True,text=True,encoding='utf-8',timeout=8)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)
if __name__=='__main__':unittest.main()
