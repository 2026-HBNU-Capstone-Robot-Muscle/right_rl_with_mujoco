import unittest
import numpy as np
from cube_orientation_routes_20261028 import validate,NAMES,NORMALS,REFERENCE,MOVES
from cube_task_frame_20261105 import local_working_basis,working_rotation,normal,face_up_error_deg,aligned_top_face,orientation_error_deg,upright_yaw_target

class CubeTaskFrameTests(unittest.TestCase):
    def test_all_six_faces_define_proper_working_basis(self):
        for face in range(6):
            Q=local_working_basis(face);np.testing.assert_array_equal(Q.T@Q,np.eye(3));self.assertEqual(np.linalg.det(Q),1.);np.testing.assert_array_equal(-Q[:,1],NORMALS[face])
    def test_existing_green_up_frame_is_identical(self):
        np.testing.assert_array_equal(local_working_basis(5),np.eye(3));np.testing.assert_array_equal(working_rotation(REFERENCE,5),REFERENCE)
    def test_all24_ideal_top_faces_are_selected(self):
        for (top,wrist),(B,moves) in validate().items():
            selected=aligned_top_face(B,[0,0,1]);self.assertEqual(selected['face_name'],top);W=working_rotation(B,selected['face_id']);np.testing.assert_array_equal(-W[:,1],[0,0,1])
    def test_tipped_cube_is_not_claimed_ready(self):
        a=np.pi/4;R=np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]]);self.assertIsNone(aligned_top_face(R@REFERENCE,[0,0,1])['face_id'])
    def test_facing_up_does_not_prove_requested_yaw(self):
        B=MOVES['yaw']@REFERENCE;self.assertEqual(aligned_top_face(B,[0,0,1])['face_name'],'green');self.assertAlmostEqual(orientation_error_deg(B,REFERENCE),90.)
    def test_height_axis_is_face_independent(self):
        for face in range(6):
            Q=local_working_basis(face);point=normal(face)*.018;self.assertAlmostEqual(float(point@(-Q[:,1]))*1000,18.)
    def test_reflection_is_rejected(self):
        with self.assertRaises(ValueError):working_rotation(np.diag([-1,1,1]),5)
    def test_face_id_requires_explicit_valid_integer(self):
        for value in (True,-1,6,1.5,'5'):
            with self.assertRaises((ValueError,TypeError)):normal(value)
    def test_zero_up_and_ambiguous_threshold_are_rejected(self):
        with self.assertRaises(ValueError):face_up_error_deg(REFERENCE,5,[0,0,0])
        with self.assertRaises(ValueError):aligned_top_face(REFERENCE,[0,0,1],45.)
    def test_upright_yaw_removes_initial_tip_without_changing_requested_twist(self):
        a=np.radians(7);Rx=np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]]);start=Rx@REFERENCE;end=upright_yaw_target(start,[0,0,1],90)
        self.assertAlmostEqual(face_up_error_deg(end,5,[0,0,1]),0.,places=5)
        relative=end@start.T;w=np.sqrt(1+np.trace(relative))/2;v=np.array([relative[2,1]-relative[1,2],relative[0,2]-relative[2,0],relative[1,0]-relative[0,1]])/(4*w)
        self.assertAlmostEqual(float(np.degrees(2*np.arctan2(v[2],w))),90.,places=7)
    def test_already_upright_target_is_exact_quarter(self):
        expected=np.array([[0.,-1,0],[1,0,0],[0,0,1]])@REFERENCE;np.testing.assert_allclose(upright_yaw_target(REFERENCE,[0,0,1],90),expected,atol=1e-14)
    def test_antipodal_or_invalid_up_target_is_rejected(self):
        with self.assertRaises(ValueError):upright_yaw_target(REFERENCE,[0,0,-1],90)
        with self.assertRaises(ValueError):upright_yaw_target(REFERENCE,[0,0,0],90)
if __name__=='__main__':unittest.main()
