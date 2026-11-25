import unittest,copy
import numpy as np
from scipy.spatial import ConvexHull
from upper_layer_cylinder_clearance_20261105 import clearance_for_vertices
from upper_layer_separation_certificate_20261105 import find_plane
from canonical_grip_task_review_20261105 import task_reasons

def evidence():return dict(safety_passed=True,regrasp_yaw_change_deg=2.,final_up_tilt_deg=3.,settled_tail=dict(seconds=4.05,yaw_drift_deg=.1,center_shift_mm=.1,minimum_upper_gap_sampled_75ms_mm=.01,opposed_lower_body_contact_fraction=1.,minimum_separating_plane_gap_1ms_mm=2.,separation_certificate_bad_steps_1ms=0,certificate_unqueried_steps_after_first_failure=0,certificate=dict(queries=4050)))
class TaskReviewTests(unittest.TestCase):
    def test_exact_certificate_suffices_despite_conservative_vertical_bound(self):self.assertEqual(task_reasons(evidence()),[])
    def test_any_uncertified_state_fails(self):
        d=evidence();d['settled_tail']['separation_certificate_bad_steps_1ms']=1;self.assertTrue(task_reasons(d))
    def test_missing_queries_fail(self):
        d=evidence();d['settled_tail']['certificate']['queries']=4049;self.assertTrue(task_reasons(d))
    def test_guard_failure_still_fails(self):
        d=evidence();d['safety_passed']=False;self.assertIn('original_dynamic_safety',task_reasons(d))
    def test_direction_and_hold_requirements_still_apply(self):
        d=evidence();d['regrasp_yaw_change_deg']=6.;self.assertIn('regrasp_yaw',task_reasons(d))
    def test_plane_gap_threshold_unchanged(self):
        d=evidence();d['settled_tail']['minimum_separating_plane_gap_1ms_mm']=.29;self.assertTrue(task_reasons(d))
    def test_vertical_bound_can_underestimate_true_radial_separation(self):
        v=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)],float)*[2,2,2]+[45,-55/6+2+.01,0];h=ConvexHull(v);e=np.array(sorted({tuple(sorted((int(a),int(b)))) for f in h.simplices for a,b in ((f[0],f[1]),(f[1],f[2]),(f[2],f[0]))}));legacy=clearance_for_vertices(v,e,True);self.assertLess(legacy,.3);plane=find_plane(v,e,h.equations[:,:3],.3);self.assertIsNotNone(plane);self.assertGreater(plane[2],4.)
if __name__=='__main__':unittest.main()
