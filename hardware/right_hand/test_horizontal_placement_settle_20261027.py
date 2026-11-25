"""Replay the actual67-tick plateau and reject drift, stale reads and excess."""
import json,unittest
from pathlib import Path
import numpy as np
from horizontal_placement_settle_20261027 import confirm
ROOT=Path(__file__).resolve().parent

class StablePlacementTests(unittest.TestCase):
    def setUp(self):
        r=json.loads((ROOT/'horizontal_quarter_trial_20261027_221539_291945/result.json').read_text(encoding='utf-8'))
        self.rows=[dict(position=np.array(s['actual']),goal=np.array(s['goal']),sample_index=i)
                   for i,s in enumerate(r['samples']) if s['phase']=='placement_checkpoint']
        self.command=self.rows[0]['goal'];self.report={}
    def reject(self):
        with self.assertRaises((RuntimeError,ValueError,TypeError)):confirm(self.rows,self.command,self.report)
        self.assertEqual(self.report['placement_settle_confirmation']['status'],'stopped')
    def test_actual67_tick_stable_plateau_is_accepted(self):
        result=confirm(self.rows,self.command,self.report)
        self.assertEqual(result['status'],'confirmed_stable');self.assertEqual(result['maximum_error_ticks_by_motor'][11],67)
    def test_id12_72_ticks_is_the_boundary(self):
        for row in self.rows:row['position'][11]=row['goal'][11]-72
        self.assertEqual(confirm(self.rows,self.command,self.report)['status'],'confirmed_stable')
    def test_id12_73_ticks_is_rejected(self):
        for row in self.rows:row['position'][11]=row['goal'][11]-73
        self.reject()
    def test_other_motor65_ticks_is_still_rejected(self):
        for row in self.rows:row['position'][1]=row['goal'][1]-65
        self.reject()
    def test_three_tick_motion_is_rejected(self):self.rows[-1]['position'][0]+=3;self.reject()
    def test_previous_goal_packet_is_rejected(self):self.rows[3]['goal'][0]-=1;self.reject()
    def test_missing_sample_is_rejected(self):self.rows.pop();self.reject()
    def test_repeated_sample_index_is_rejected(self):self.rows[4]['sample_index']=self.rows[3]['sample_index'];self.reject()
    def test_nonfinite_position_is_rejected(self):
        self.rows[0]['position']=self.rows[0]['position'].astype(float);self.rows[0]['position'][0]=float('nan');self.reject()
    def test_outside_native_position_is_rejected(self):
        self.command=self.command.copy();self.command[3]=1700
        for row in self.rows:row['goal']=self.command.copy();row['position'][3]=1700
        self.reject()

class PlacementModelEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import horizontal_placement_evidence_20261027 as evidence
        cls.evidence=evidence;cls.geometry=json.loads(evidence.GEOMETRY.read_bytes());cls.weak=json.loads(evidence.WEAK.read_bytes())
    def test_measured_geometry_and_weaker_response_are_bound(self):
        result=self.evidence.review();self.assertEqual(result['geometry']['cases'],207)
        self.assertEqual(result['weaker_response']['required_cases'],9)
        self.assertFalse(result['weaker_response']['calibrated_model'])
    def test_missing_weak_case_is_rejected(self):
        import copy
        data=copy.deepcopy(self.weak);data['cases'].pop()
        with self.assertRaises(RuntimeError):self.evidence.weak_review(data)
    def test_false_pass_with_excess_tracking_is_rejected(self):
        import copy
        data=copy.deepcopy(self.weak);next(v for v in data['cases'] if v['gain_scale']==.5)['max_tracking_rad']=.181
        with self.assertRaises(RuntimeError):self.evidence.weak_review(data)
    def test_unapproved_native_geometry_is_rejected(self):
        import copy
        data=copy.deepcopy(self.geometry);data['cases'][0]['native']=False
        with self.assertRaises(RuntimeError):self.evidence.geometry_review(data)
    def test_relaxed_geometry_boundary_is_rejected(self):
        import copy
        data=copy.deepcopy(self.geometry);data['id12_maximum_error_ticks']=96
        with self.assertRaises(RuntimeError):self.evidence.geometry_review(data)

if __name__=='__main__':unittest.main()
