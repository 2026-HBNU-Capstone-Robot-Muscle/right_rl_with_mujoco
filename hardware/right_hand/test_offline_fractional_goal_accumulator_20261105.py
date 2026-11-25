import unittest
import numpy as np
from offline_fractional_goal_accumulator_20261105 import FractionalGoals

class FractionalGoalTests(unittest.TestCase):
    def make(self):return FractionalGoals(np.full(16,150),np.full(16,100),np.full(16,200))
    def test_small_intended_motion_is_not_erased(self):
        p=self.make();naive=150
        for _ in range(10):naive=round(naive+.2);p.advance(np.full(16,.2))
        self.assertEqual(naive,150);self.assertTrue(np.all(p.goal==152))
    def test_rounding_errors_do_not_accumulate_into_drift(self):
        p=self.make()
        for _ in range(100):p.advance(np.full(16,.3));p.advance(np.full(16,-.3))
        np.testing.assert_array_equal(p.goal,np.full(16,150))
    def test_bound_used_by_optimizer_limits_quantized_step(self):
        p=self.make();p.advance(np.full(16,.49));_,upper=p.increment_bounds();q=p.advance(upper);np.testing.assert_array_equal(q,np.full(16,152))
    def test_native_edge_is_retained_with_fractional_updates(self):
        p=self.make()
        for _ in range(25):p.advance(p.increment_bounds()[1])
        np.testing.assert_array_equal(p.goal,np.full(16,200));np.testing.assert_array_equal(p.increment_bounds()[1],np.zeros(16))
    def test_illegal_step_does_not_advance_planner(self):
        p=self.make()
        with self.assertRaises(ValueError):p.advance(np.full(16,3))
        np.testing.assert_array_equal(p.continuous,np.full(16,150))
    def test_nonfinite_update_does_not_advance_planner(self):
        p=self.make()
        with self.assertRaises(ValueError):p.advance(np.full(16,np.nan))
        np.testing.assert_array_equal(p.goal,np.full(16,150))
    def test_native_limit_violation_is_rejected(self):
        p=self.make()
        with self.assertRaises(ValueError):p.advance(np.full(16,51))
        self.assertTrue(np.all(p.goal==150))
if __name__=='__main__':unittest.main()
