import unittest,itertools
import numpy as np
from upper_layer_cylinder_clearance_20261105 import radial_distance,clearance_for_vertices,RADIUS
SIGNS=np.array(list(itertools.product((-1.,1.),repeat=3)))
EDGES=np.array([(i,j) for i in range(8) for j in range(i+1,8) if sum(SIGNS[i]!=SIGNS[j])==1])
def box(c,h):return SIGNS*np.array(h)+np.array(c)
class Tests(unittest.TestCase):
    def test_below_upper_layer_is_clear(self):self.assertGreater(clearance_for_vertices(box((28,5,0),(2,3,2)),EDGES),0)
    def test_intermediate_turn_hits_even_with_initial_square_clear(self):
        v=box((35.5,-16,0),(.5,2,1));self.assertGreater(v[:,0].min(),27.5);self.assertLess(clearance_for_vertices(v,EDGES),0)
    def test_corner_outside_circle_is_clear(self):self.assertGreater(clearance_for_vertices(box((36,-16,36),(.5,2,.5)),EDGES),0)
    def test_tangent_is_not_positive_clearance(self):self.assertAlmostEqual(clearance_for_vertices(box((RADIUS+1,-16,0),(1,2,1)),EDGES),0,places=8)
    def test_height_clipping_avoids_cross_height_false_collision(self):
        v=np.array([np.array([52,-30,0])+(s[0]+1)/2*np.array([-32,60,0])+s[1]*np.array([.1,0,0])+s[2]*np.array([0,0,.1]) for s in SIGNS]);self.assertLess(clearance_for_vertices(v,EDGES,False),0);self.assertGreater(clearance_for_vertices(v,EDGES,True),0)
    def test_projected_segment_crossing_origin(self):self.assertEqual(radial_distance(np.array([[-1,0],[1,0]])),0)
if __name__=='__main__':unittest.main()
