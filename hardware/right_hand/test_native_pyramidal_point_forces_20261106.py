import unittest,numpy as np
from native_pyramidal_point_forces_20261106 import point_rays,pyramid_margin

class NativeConeTests(unittest.TestCase):
    def test_native_unit_edges_use_the_configured_anisotropic_coefficients(self):
        E=point_rays(.3,.1);self.assertTrue(np.allclose(E,[[1,1,1,1],[.3,-.3,0,0],[0,0,.1,-.1]]))
        for ray in E.T:self.assertAlmostEqual(pyramid_margin(ray,.3,.1),0.)
    def test_diagonal_force_inside_elliptic_cone_can_be_outside_native_pyramid(self):
        force=np.array([1.,.2,.2]);self.assertLess(np.linalg.norm(force[1:]),.3*force[0]);self.assertLess(pyramid_margin(force,.3,.3),0.)
        self.assertGreaterEqual(pyramid_margin(np.array([1.,.15,.15]),.3,.3),-1e-12)
    def test_nonnegative_native_edge_combinations_remain_in_the_native_cone(self):
        E=point_rays(.3,.2);rng=np.random.default_rng(12)
        for _ in range(40):self.assertGreaterEqual(pyramid_margin(E@rng.uniform(0,2,4),.3,.2),-1e-12)
        with self.assertRaises(ValueError):point_rays(0.,.3)
if __name__=='__main__':unittest.main()
