import json
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from sac_anchor_bundle import fit,project
ROOT=Path(__file__).resolve().parent

class BundleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture=json.loads((ROOT/'sac_anchor_bundle_synthetic_fixture.json').read_text())

    def test_projection_matches_independent_opencv(self):
        f=self.fixture;p=f['projection_fixture']
        result=project(np.array(p['points']),p['rvec'],np.array(p['tvec']),np.array(f['intrinsic']),np.array(f['distortion']))
        np.testing.assert_allclose(result,p['expected'],atol=1e-7)

    def test_multiview_known_rigid_transform(self):
        f=self.fixture;result=fit(f)
        self.assertTrue(result['diagnostic_fit_passed'],result)
        angle=np.degrees(Rotation.from_matrix(np.array(result['relative_rotation_41_to_40']).T@np.array(f['truth_relative_rotation'])).magnitude())
        shift=np.linalg.norm(np.array(result['marker40_origin_in_marker41_m'])-f['truth_relative_translation_m'])*1000
        self.assertLess(angle,1.);self.assertLess(shift,.7)
        self.assertFalse(result['hardware_authorized'])

    def test_same_view_cannot_certify_multiview(self):
        f=dict(self.fixture);f['frames']=[dict(self.fixture['frames'][0],view=i) for i in range(3)]
        result=fit(f)
        self.assertFalse(result['diagnostic_fit_passed'])
        self.assertIn('insufficient_camera_normal_diversity_below_12deg',result['rejection_reasons'])

if __name__=='__main__':unittest.main()
