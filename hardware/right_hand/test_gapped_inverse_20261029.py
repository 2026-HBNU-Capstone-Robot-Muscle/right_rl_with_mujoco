"""Meaningful admission/checkpoint tests. No device or motor access."""
import json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import gapped_inverse_gate_20261029 as gate
import leap_gapped_inverse_trial_20261029 as entry
from gapped_inverse_plan_20261029 import (GappedInversePlan,REQUEST,PROOF,OUT,
    LOADED,OPEN_PATH,ENTRY,READY,GAP,GAP_HOLD,placement_goal)
from gapped_inverse_capture_20261029 import GappedInverseCapture
from loaded_quarter_reentry_20261025 import limits
from calibrated_quarter_plan_20261028 import LOADED_PATH as FORWARD_PATH

class InverseAdmissionTests(unittest.TestCase):
    def setUp(self):self.plan=GappedInversePlan(REQUEST)
    def test_completed_models_are_bound(self):
        r=gate.validate_review(self.plan,PROOF)
        self.assertEqual(r['inverse_model']['cases'],27)
        self.assertEqual(r['additional_hand_path']['new_geometry_cases'],201)
        self.assertEqual(r['additional_hand_path']['finite_empty_cases'],9)
        self.assertFalse(r['inverse_90_ready'])
        self.assertFalse(r['physical_inverse_cube_rotation_verified'])
        self.assertEqual(r['current_ma'],450)
        self.assertEqual(r['hardware_tracking_stop_rad'],.18)
        self.assertFalse(r['automatic_goal_or_setting_resend'])
    def test_every_goal_in_native_bounds_and_previous_goal_step(self):
        start=self.plan.source_position();rows=self.plan.plan(start)
        values=np.vstack([start,*[q for _,q in rows]])
        lo,hi=limits()
        others=np.array([i for i in range(16) if i!=3])
        self.assertTrue(np.all(values[:,others]>=lo[others]) and np.all(values<=hi))
        # Existing passive OFF ID4 admission is a monotonic return, not a new
        # lowered active range. Every other phase must be inside configured bounds.
        previous=int(start[3])
        for phase,q in rows:
            if phase==ENTRY:
                self.assertGreaterEqual(int(q[3]),previous)
                self.assertGreaterEqual(int(q[3]),int(start[3]))
            else:self.assertGreaterEqual(float(q[3]),float(lo[3]))
            previous=int(q[3])
        self.assertLessEqual(int(np.max(abs(np.diff(values,axis=0)))),2)
        self.assertLess((len(rows)+40)*.075+20,300)
        z=np.load(LOADED)
        forward=np.load(FORWARD_PATH)
        np.testing.assert_array_equal(z['goals'][z['phases']=='stage_7_horizontal_yaw'],
            forward['goals'][forward['phases']=='yaw'][::-1])
    def test_placement_occurs_after_empty_gap_and_before_closing(self):
        rows=self.plan.plan(self.plan.source_position())
        first=next(i for i,(phase,_) in enumerate(rows) if phase==self.plan.CAPTURE_BEFORE_PHASE)
        self.assertEqual(rows[first-1][0],GAP_HOLD)
        np.testing.assert_array_equal(rows[first-1][1],placement_goal())
        for i,phase in enumerate((READY,GAP)):
            selected=[q for p,q in rows if p==phase]
            mask=self.plan.frozen_for_phase(phase)
            self.assertTrue(np.all(np.asarray(selected)[:,mask]==np.asarray(selected)[0,mask]))
    def test_checkpoint_endpoints_and_no_fabricated_removal(self):
        capture=GappedInverseCapture.__new__(GappedInverseCapture)
        opened=np.load(OPEN_PATH)['open_goal']
        capture.require_open(placement_goal())
        with self.assertRaises(RuntimeError):capture.require_open(opened)
        capture.placement_done=True;capture.require_open(opened)
        with self.assertRaises(RuntimeError):capture.require_open(placement_goal())
        with self.assertRaises(RuntimeError):capture.finish(None,opened,{})
    def test_stale_proof_rejected(self):
        report=json.loads(PROOF.read_bytes());report['plan_sha256']='stale'
        handle=tempfile.NamedTemporaryFile(dir=OUT,suffix='.json',delete=False)
        path=Path(handle.name);handle.close()
        try:
            path.write_text(json.dumps(report),encoding='utf-8')
            with self.assertRaises(RuntimeError):gate.validate_review(self.plan,path)
        finally:path.unlink(missing_ok=True)
    def test_default_does_not_construct_devices(self):
        with patch.object(sys,'argv',['inverse-check']),patch.object(entry,'GappedInverseBus',side_effect=AssertionError('device constructed')) as bus,patch.object(GappedInversePlan,'CAMERA_TYPE',side_effect=AssertionError('camera constructed')) as camera:
            self.assertEqual(entry.main(),0);bus.assert_not_called();camera.assert_not_called()
    def test_rejected_physics_never_reaches_runtime(self):
        with patch.object(gate,'inverse_model_review',side_effect=RuntimeError('model rejected')),patch.object(entry.runtime,'main',side_effect=AssertionError('runtime reached')) as run:
            self.assertEqual(entry.main(),2);run.assert_not_called()

if __name__=='__main__':unittest.main()
