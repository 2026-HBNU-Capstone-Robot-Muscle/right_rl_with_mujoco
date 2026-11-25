"""Fault-injection admission and two-leg capture tests; no devices are opened."""
import copy,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import yaw_roundtrip_gate_20261030 as gate
from yaw_roundtrip_plan_20261030 import ROOT,OUT,LOADED,REQUEST,PROOF,MODEL_SOURCES,YawRoundtripPlan
from yaw_roundtrip_capture_20261030 import YawRoundtripCapture
class RoundtripTests(unittest.TestCase):
    def scratch(self):
        tmp=tempfile.TemporaryDirectory(dir=OUT,prefix='test_');Path(tmp.name).resolve().relative_to(ROOT)
        self.addCleanup(tmp.cleanup);return Path(tmp.name)
    def changed_tape(self,mutation):
        z=np.load(LOADED);data={k:z[k].copy() for k in z.files};mutation(data)
        p=self.scratch()/'changed.npz';np.savez_compressed(p,**data);return p
    def test_exact_composition(self):
        r=gate.composition_review();self.assertEqual((r['forward_yaw_goals'],r['inverse_yaw_goals'],r['new_joint_goal_vectors']),(638,638,0))
    def test_reordered_previously_known_goals_rejected(self):
        def bad(d):d['goals'][100:102]=d['goals'][100:102][::-1]
        p=self.changed_tape(bad)
        with patch.object(gate,'LOADED',p),self.assertRaises(RuntimeError):gate.composition_review()
    def test_inserted_open_between_legs_rejected(self):
        def bad(d):d['goals'][int(d['forward_end_sample'])]=d['start']
        p=self.changed_tape(bad)
        with patch.object(gate,'LOADED',p),self.assertRaises(RuntimeError):gate.composition_review()
    def test_fake_hold_capture_boundary_rejected(self):
        def bad(d):d['inverse_hold_goal']=d['forward_hold_goal'].copy()
        p=self.changed_tape(bad)
        with patch.object(gate,'LOADED',p),self.assertRaises(RuntimeError):gate.composition_review()
    def test_two_leg_photos_are_distinct(self):
        c=YawRoundtripCapture.__new__(YawRoundtripCapture);clock=[1.];c.now=lambda:clock[0]
        class Camera:
            def capture_named(self,name,after):return name if clock[0]>=after else None
        c.camera=Camera();r={};z=np.load(LOADED);state={'position':z['forward_hold_goal']}
        c.rotation_frame(None,z['forward_hold_goal'],r,state);clock[0]=1.3;c.rotation_frame(None,z['forward_hold_goal'],r,state)
        clock[0]=50.;c.rotation_frame(None,z['inverse_hold_goal'],r,{'position':z['inverse_hold_goal']});clock[0]=50.3;c.rotation_frame(None,z['inverse_hold_goal'],r,{'position':z['inverse_hold_goal']})
        self.assertEqual(r['forward_observation_photo'],'cube_forward_turn_hold.jpg');self.assertEqual(r['inverse_observation_photo'],'cube_inverse_turn_hold.jpg')
        self.assertFalse(r['physical_inverse90_verified']);self.assertEqual(len(c.roundtrip_hold_starts),2)
    def test_unreviewed_hold_endpoint_rejected(self):
        c=YawRoundtripCapture.__new__(YawRoundtripCapture)
        with self.assertRaises(RuntimeError):c.rotation_frame(None,np.zeros(16,int),{},dict(position=np.zeros(16,int)))
    def test_no_checkpoint_bypass(self):
        p=YawRoundtripPlan(REQUEST);p.EMPTY_RELEASE_ONLY=True
        with self.assertRaisesRegex(RuntimeError,'Scope'):gate.make_review(p,{},None)
    def test_preserved_timing_bounds(self):
        p=YawRoundtripPlan(REQUEST);self.assertEqual(p.SAMPLE_PERIOD,.075);self.assertEqual(p.ON_TIMEOUT,300)
        a=p.source_position();rows=p.plan(a);self.assertLessEqual(np.max(abs(np.diff(np.vstack([a,*[q for _,q in rows]]),axis=0))),2)
    def cube_fixture(self,mutate):
        reports=[json.loads(p.read_bytes()) for p in MODEL_SOURCES];r=copy.deepcopy(reports[0]);r['cases']=[c for report in reports for c in report['cases']];mutate(r)
        p=self.scratch()/'model.json';p.write_text(json.dumps(r),encoding='utf-8');return p
    def reject_model(self,mutate):
        p=self.cube_fixture(mutate)
        with patch.object(gate,'MODEL_SOURCES',[p]),self.assertRaises(RuntimeError):gate.cube_models()
    def test_real_models_not_success_certificate(self):
        r=gate.cube_models();self.assertEqual(r['cases'],27);self.assertFalse(r['physical_inverse90_verified'])
    def test_missing_model_case_rejected(self):self.reject_model(lambda r:r['cases'].pop())
    def test_initial_overlap_cannot_be_ignored(self):self.reject_model(lambda r:r['cases'][0]['initial_placement_probe'].update(max_cube_mm=1.01))
    def test_submillimetre_self_contact_rejected(self):self.reject_model(lambda r:r['cases'][0].update(max_self_mm=.001))
    def test_retention_loss_rejected(self):self.reject_model(lambda r:r['cases'][0].update(longest_no_hand_contact_seconds=.251))
    def test_missing_free_seating_endpoint_rejected(self):self.reject_model(lambda r:r['cases'][0].update(vertical_seating_line_upper_verified_free=False))
    def test_changed_model_code_rejected(self):self.reject_model(lambda r:r.update(model_source_sha256='0'*64))
    def test_fabricated90_claim_in_proof_rejected(self):
        r=json.loads(PROOF.read_bytes());r['inverse90_ready']=True;p=self.scratch()/'proof.json';p.write_text(json.dumps(r),encoding='utf-8')
        with self.assertRaises(RuntimeError):gate.validate_review(YawRoundtripPlan(REQUEST),p)
if __name__=='__main__':unittest.main()
