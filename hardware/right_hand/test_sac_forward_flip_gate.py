"""Check the sign of the direction metric with exact rigid rotations."""

import unittest

import numpy as np

from sac_forward_flip_gate import motion_from_quats, verdict


class ForwardFlipGateTest(unittest.TestCase):
    def test_positive_forward_quarter_turn_passes(self):
        q = [np.sqrt(.5), 0, 0, -np.sqrt(.5)]
        motion = motion_from_quats([1, 0, 0, 0], q)
        self.assertAlmostEqual(motion['green_tip_toward_initial_orange_front_deg'], 90)
        self.assertTrue(verdict(motion)['full_forward_flip'])

    def test_opposite_quarter_turn_fails(self):
        q = [np.sqrt(.5), 0, 0, np.sqrt(.5)]
        motion = motion_from_quats([1, 0, 0, 0], q)
        self.assertAlmostEqual(motion['green_tip_toward_initial_orange_front_deg'], -90)
        self.assertFalse(verdict(motion)['forward_progress'])

    def test_top_axis_yaw_is_not_forward_flip(self):
        q = [np.sqrt(.5), 0, -np.sqrt(.5), 0]
        motion = motion_from_quats([1, 0, 0, 0], q)
        self.assertAlmostEqual(motion['green_normal_change_deg'], 0)
        self.assertTrue(verdict(motion)['wrong_side_turn'])
        self.assertFalse(verdict(motion)['full_forward_flip'])


if __name__ == '__main__':
    unittest.main()
