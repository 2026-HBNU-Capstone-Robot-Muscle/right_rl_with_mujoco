import ast,contextlib,copy,io,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import adaptive_yaw_loaded_plan_20261029 as p
import adaptive_yaw_loaded_gate_20261029 as gate
import leap_adaptive_yaw_loaded_20261029 as entry
from leap_four_reply_yaw_20261029 import FourReplyBus,ReferenceCapture,PairGraspCamera,confirmed_run
from adaptive_yaw_empty_gate_20261029 import runner_review
class Checks(unittest.TestCase):
    def test_actual_empty_run_did_not_activate_wait(self):
        r=gate.empty_evidence();self.assertEqual(r['yaw_samples'],320);self.assertEqual(r['root_wait_events'],0)
        self.assertTrue(r['improvement_cannot_be_attributed_to_inactive_wait']);self.assertFalse(r['actual_wait_controller_activation_verified'])
    def test_full_angular_command_queue_is_the_same(self):
        class Reference(p.Base):PLAN_VERSION=p.AdaptiveYawLoadedPlan.PLAN_VERSION
        a=Reference(p.REQUEST);b=p.AdaptiveYawLoadedPlan(p.REQUEST);aa=a.plan(a.source_position());bb=b.plan(b.source_position())
        self.assertEqual([s for s,q in aa],[s for s,q in bb]);np.testing.assert_array_equal([q for s,q in aa],[q for s,q in bb])
        self.assertEqual(sum(s=='stage_7_horizontal_yaw' for s,q in bb),638);self.assertEqual(b.ON_TIMEOUT,300)
        self.assertFalse(getattr(b,'EMPTY_RELEASE_ONLY',False));self.assertIs(b.CAPTURE_PROTOCOL_TYPE,ReferenceCapture);self.assertIs(b.CAMERA_TYPE,PairGraspCamera)
    def test_same_health_transport_write_and_cleanup_implementation(self):
        for name in ('checked_sample','_block_once','_receive','write'):
            self.assertIs(getattr(entry.AdaptiveLoadedBus,name),getattr(FourReplyBus,name))
        self.assertTrue(runner_review()['all_original_health_held_progress_timeout_cleanup_code_unchanged'])
    def test_complete_finite_loaded_and_recovery_evidence(self):
        r=gate.validate_review(p.AdaptiveYawLoadedPlan(p.REQUEST),p.PROOF)
        self.assertEqual(len(r['empty_recovery']['static_cases']),201);self.assertEqual(len(r['empty_recovery']['dynamic_cases']),9)
        self.assertEqual(r['loaded_fixed_goal_pause_stress']['cases'],27);self.assertTrue(r['loaded_fixed_goal_pause_stress']['coarse2ms_grid_rejected'])
        self.assertEqual(r['loaded_fixed_goal_pause_stress']['simulation_timestep_s'],.001);self.assertTrue(r['wait_included_in_original300s'])
        self.assertEqual(r['hardware_tracking_stop_rad'],.18);self.assertEqual(r['current_ma'],450);self.assertFalse(r['motor_gain_change'])
    def test_stress_only_adds_fixed_targets_and_does_not_extend_runtime_queue(self):
        z=np.load(p.STRESS_PATH);base=np.load(p.LOADED_PATH)
        np.testing.assert_array_equal(z['goals'][~z['fixed_hold_rows']],base['goals'][base['phases']=='yaw'])
        self.assertEqual(int(z['fixed_hold_rows'].sum()),256)
        self.assertEqual(float(z['each_hold_seconds']),1.2);self.assertLessEqual(float(z['added_hold_seconds']),20.)
    def test_refinement_keeps_physics_and_all_stop_criteria(self):
        old=ast.parse((p.ROOT/'screen_loaded_yaw_path_20261027.py').read_text(encoding='utf-8'))
        refined=ast.parse((p.ROOT/'screen_loaded_yaw_convergence_20261029.py').read_text(encoding='utf-8'))
        def clauses(tree):
            return [ast.dump(n.test,include_attributes=False) for n in ast.walk(tree) if isinstance(n,ast.If) and any(isinstance(v,ast.Name) and v.id=='maxself' for v in ast.walk(n.test))]
        self.assertEqual(len(clauses(old)),1);self.assertEqual(clauses(old),clauses(refined))
        def cube_physics(tree):
            return [ast.dump(n,include_attributes=False) for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id in ('set_cube_physics','correct_model','build')]
        self.assertEqual(cube_physics(old),cube_physics(refined))
    def test_changed_wait_or_tracking_criteria_are_rejected(self):
        r=json.loads(p.PROOF.read_bytes())
        for field in ('root_lag_pause_policy','hardware_tracking_stop_rad'):
            bad=copy.deepcopy(r)
            if field=='root_lag_pause_policy':bad[field]['maximum_single_wait_s']=2.
            else:bad[field]=.2
            with tempfile.TemporaryDirectory() as folder:
                path=Path(folder)/'bad.json';path.write_text(json.dumps(bad),encoding='utf-8')
                with self.assertRaises(RuntimeError):gate.validate_review(p.AdaptiveYawLoadedPlan(p.REQUEST),path)
    def test_default_entry_does_not_open_devices(self):
        import cv2,dynamixel_sdk
        with patch.object(sys,'argv',['leap_adaptive_yaw_loaded_20261029.py']),patch.object(cv2,'VideoCapture',side_effect=AssertionError('Camera opened')),patch.object(dynamixel_sdk,'PortHandler',side_effect=AssertionError('Motor opened')),contextlib.redirect_stdout(io.StringIO()):self.assertEqual(entry.main(),0)
if __name__=='__main__':unittest.main(verbosity=2)
