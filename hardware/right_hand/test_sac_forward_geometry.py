import unittest
import numpy as np
from sac_forward_geometry import measure, multiply, eligible, reward_value


def axis_angle(axis, degrees):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    a = np.deg2rad(degrees)/2
    return np.r_[np.cos(a), axis*np.sin(a)]


class ForwardGeometryTest(unittest.TestCase):
    def test_quarter_turn_neighborhood(self):
        for deg in (70.1, 85, 89, 89.999, 90, 90.001, 91, 95, 109.9):
            with self.subTest(deg=deg):
                values = measure([1,0,0,0], axis_angle([0,0,-1],deg))
                np.testing.assert_allclose(np.rad2deg(values), [deg,0,abs(90-deg)], atol=1e-6)
                self.assertTrue(eligible(*values, .0))

    def test_opposite_sideways_and_half_turn_fail(self):
        for axis, deg in (([0,0,-1],-90), ([0,-1,0],90), ([0,0,-1],170),
                          ([1,0,0],180)):
            self.assertFalse(eligible(*measure([1,0,0,0],axis_angle(axis,deg)), .0))

    def test_frame_and_quaternion_sign_invariance(self):
        start=axis_angle([1,2,3],73)
        end=multiply(start,axis_angle([0,0,-1],91))
        for a,b in ((start,end),(-start,end),(start,-end),(-start,-end)):
            np.testing.assert_allclose(np.rad2deg(measure(a,b)),[91,0,1],atol=1e-6)

    def test_side_swing_is_not_hidden_by_correct_top_face(self):
        end=multiply(axis_angle([1,0,0],40),axis_angle([0,0,-1],90))
        tip, swing, error=measure([1,0,0,0],end)
        self.assertAlmostEqual(np.rad2deg(swing),40)
        self.assertFalse(eligible(tip,swing,error,0))

    def test_reverse_direction_and_translation_limit(self):
        values=measure([1,0,0,0],axis_angle([0,0,-1],-90),-1)
        self.assertTrue(eligible(*values,0,-1))
        self.assertFalse(eligible(*values,.031,-1))

    def test_reward_continuous_and_peaked_at_goal(self):
        rewards=[]
        for deg in (0,85,89.99,90,90.01,95,170,-90):
            _,swing,error=measure([1,0,0,0],axis_angle([0,0,-1],deg))
            rewards.append(reward_value(error,swing,0,np.zeros(16),False,False))
        self.assertAlmostEqual(rewards[2],rewards[4],places=8)
        self.assertGreater(rewards[3],rewards[1])
        self.assertGreater(rewards[1],rewards[0])
        self.assertGreater(rewards[0],rewards[-1])

    def test_holding_outside_success_cannot_farm_reward(self):
        error=np.deg2rad(21)
        self.assertLess(reward_value(error,0,0,np.zeros(16),False,False,
                                     previous_target_error=error),0)
        self.assertGreater(reward_value(0,0,0,np.zeros(16),True,False,
                                        previous_target_error=0),0)


if __name__=='__main__':
    unittest.main()
