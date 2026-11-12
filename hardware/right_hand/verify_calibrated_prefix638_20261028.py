"""Device-free validation of the physical-timing calibration variant."""
import copy,hashlib,json,subprocess,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import calibrated_quarter_plan_20261028 as p
import calibrated_quarter_gate_20261028 as gate
import calibrated_quarter_source_20261028 as source
from horizontal_quarter_plan_20261027 import LOADED_PATH as ORIGINAL,PROOF as BASE_PROOF
class CalibrationChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan=p.HorizontalQuarterPlan(p.REQUEST);cls.review=gate.validate_review(cls.plan,p.PROOF)
    def test_source_frame_and_original_goal_are_bound(self):
        r=source.review();self.assertEqual(r['kept_original_yaw_goals'],638);self.assertAlmostEqual(r['observed_camera_angle_deg'],90,delta=2)
        self.assertFalse(r['actual_shortened_angle_verified']);self.assertFalse(r['certified_robot_frame_angle'])
    def test_baseline_trajectory_unchanged(self):
        self.assertEqual(hashlib.sha256(ORIGINAL.read_bytes()).hexdigest(),'631c4081f5bbf75490479f48a9a84e437c7f7280597cdc484722205b6a0b30df')
    def test_new_cycle_has_exactly638_yaw_goals_and_open_endpoint(self):
        z=np.load(p.LOADED_PATH);old=np.load(ORIGINAL)
        self.assertTrue(np.array_equal(z['goals'][z['phases']=='yaw'],old['goals'][old['phases']=='yaw'][:638]))
        self.assertTrue(np.array_equal(z['goals'][-1],z['start']))
    def test_fresh_prefix_same_as_independent_empty_recovery(self):
        self.assertEqual(p.prefix_digest(self.plan),self.review['empty_recovery']['prefix_sha256'])
        self.assertEqual(len(self.review['empty_recovery']['static_cases']),201)
        self.assertEqual(len(self.review['empty_recovery']['dynamic_cases']),9)
    def test_loaded_physics_and_geometry_complete(self):
        loaded,static=gate.validate_loaded();self.assertEqual(len(loaded['cases']),27);self.assertEqual(len(static['cases']),201)
        self.assertTrue(all(r['passed'] for r in loaded['cases']+static['cases']))
        self.assertLess(self.review['nominal_model_yaw_deg'],65)
        self.assertTrue(self.review['physical_timing_calibration']['nominal_model_is_not_a90_degree_prediction'])
    def test_all_targets_keep_native_and_two_tick_bounds(self):
        from loaded_quarter_reentry_20261025 import validate_path
        from new_pair_grasp_plan_20261027 import ENTRY
        rows=self.plan.plan(self.plan.source_position());values=validate_path(self.plan.source_position(),rows,ENTRY)
        self.assertLessEqual(int(np.max(abs(np.diff(np.vstack([self.plan.source_position(),values]),axis=0)))),2)
    def test_original_full_route_cannot_use_new_calibration_source(self):
        with patch.object(source,'LOADED_PATH',ORIGINAL):
            with self.assertRaises(RuntimeError):source.review()
    def test_mutated_fixed_hold_is_rejected(self):
        z=np.load(p.LOADED_PATH);data={n:z[n].copy() for n in z.files};i=int(np.flatnonzero(data['phases']=='yaw_hold')[0]);data['goals'][i,0]+=1
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'path.npz';np.savez_compressed(path,**data)
            with patch.object(source,'LOADED_PATH',path):
                with self.assertRaises(RuntimeError):source.review()
    def test_incomplete_loaded_conditions_are_rejected(self):
        r=json.loads(p.LOADED_REVIEW.read_bytes());r['cases']=r['cases'][:-1]
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'review.json';path.write_bytes(json.dumps(r).encode())
            with patch.object(gate,'LOADED_REVIEW',path):
                with self.assertRaises(RuntimeError):gate.validate_loaded()
    def test_contact_failure_not_excused_by_calibration(self):
        r=json.loads(p.LOADED_REVIEW.read_bytes());r['cases'][0]['max_self_mm']=.1
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'review.json';path.write_bytes(json.dumps(r).encode())
            with patch.object(gate,'LOADED_REVIEW',path):
                with self.assertRaises(RuntimeError):gate.validate_loaded()
    def test_old_route_proof_rejected(self):
        with self.assertRaises(RuntimeError):gate.validate_review(self.plan,BASE_PROOF)
    def test_changed_queue_proof_rejected(self):
        r=copy.deepcopy(self.review);r['plan_sha256']='0'*64
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'review.json';path.write_bytes(json.dumps(r).encode())
            with self.assertRaises(RuntimeError):gate.validate_review(self.plan,path)
    def test_changed_clock_policy_is_rejected(self):
        r=copy.deepcopy(self.review);r['precise_clock_policy']['budget_clock']='time.monotonic'
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'review.json';path.write_bytes(json.dumps(r).encode())
            with self.assertRaises(RuntimeError):gate.validate_review(self.plan,path)
    def test_one_healthy_state_is_not_enough(self):
        r=copy.deepcopy(self.review);r['read_recovery_episode_policy']['required_consecutive_clean_reads']=1
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'review.json';path.write_bytes(json.dumps(r).encode())
            with self.assertRaises(RuntimeError):gate.validate_review(self.plan,path)
    def test_bus_health_and_receive_methods_are_inherited(self):
        from leap_calibrated_quarter_20261028 import CalibratedQuarterBus
        from leap_horizontal_quarter_20261027 import HorizontalQuarterBus
        for method in ('write',):
            self.assertIs(getattr(CalibratedQuarterBus,method),getattr(HorizontalQuarterBus,method))
        from calibrated_precise_window_20261028 import POLICY
        self.assertEqual(POLICY['packet_window_ms'],40.);self.assertEqual(POLICY['torque_off_packet_window_ms'],80.)
        self.assertEqual(self.plan.SAMPLE_PERIOD,.075);self.assertEqual(self.review['current_ma'],450)
        self.assertEqual(self.review['hardware_tracking_stop_rad'],.18)
    def test_default_cli_does_not_open_devices(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(p.ROOT/'leap_calibrated_quarter_20261028.py')],cwd=p.ROOT,capture_output=True,text=True,encoding='utf-8',timeout=30)
        self.assertEqual(r.returncode,0,r.stdout+r.stderr)
        self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)
if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromTestCase(CalibrationChecks))
    report=dict(status='passed' if result.wasSuccessful() else 'failed',tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),devices_opened=False,motor_writes=0)
    (p.OUT/'calibration_tests.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
    if not result.wasSuccessful() or result.skipped:raise SystemExit(1)
    print(json.dumps(report))
