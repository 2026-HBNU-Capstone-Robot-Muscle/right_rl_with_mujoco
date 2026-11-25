import unittest
import numpy as np
from offline_goal_tracking_bounds_20261106 import tracking_increment_bounds

class TrackingProposalTests(unittest.TestCase):
    def test_stalled_joint_retreats_instead_of_advancing(self):
        lo,hi=tracking_increment_bounds(np.full(16,-2.),np.full(16,2.),np.full(16,100.),np.zeros(16),78.)
        np.testing.assert_array_equal(lo,np.full(16,-2.));np.testing.assert_array_equal(hi,lo)
    def test_near_margin_keeps_fractional_progress_but_limits_next_goal(self):
        lo,hi=tracking_increment_bounds(np.full(16,-2.),np.full(16,2.),np.full(16,77.9),np.zeros(16),78.2)
        np.testing.assert_allclose(hi,.3);self.assertTrue(np.all(77.9+hi<=78.2+1e-12));self.assertTrue(np.all(lo>=-2))
    def test_native_bound_wins_when_actual_joint_is_beyond_reachable_band(self):
        lo,hi=tracking_increment_bounds(np.full(16,-2.),np.zeros(16),np.full(16,100.),np.full(16,200.),50.)
        np.testing.assert_array_equal(lo,np.zeros(16));np.testing.assert_array_equal(hi,np.zeros(16))
    def test_invalid_or_empty_intervals_are_rejected(self):
        with self.assertRaises(ValueError):tracking_increment_bounds(np.ones(16),np.zeros(16),np.zeros(16),np.zeros(16),50.)
        with self.assertRaises(ValueError):tracking_increment_bounds(np.zeros(16),np.ones(16),np.zeros(16),np.zeros(16),float('nan'))
if __name__=='__main__':unittest.main()
