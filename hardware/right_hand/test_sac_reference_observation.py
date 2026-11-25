import unittest
import numpy as np
import cv2
from sac_reference_numpy import DEFAULT
from sac_reference_observation import ReferenceObservation, motor_ticks_to_sim


class ObservationTest(unittest.TestCase):
    def sample(self, builder, **changes):
        values = dict(measured_joints=np.ones(16), commanded_joints=np.zeros(16),
                      palm_position=[1, 2, 3], cube_position=[4, 6, 8],
                      cube_quaternion=[1, 0, 0, 0], goal_quaternion=[1, 0, 0, 0],
                      last_executed_action=np.zeros(16), reference_step=0)
        values.update(changes)
        return builder.build(**values)

    def test_id_mapping_units(self):
        ticks = np.arange(16)*32 + 2048
        actual = motor_ticks_to_sim(ticks)
        expected = np.r_[np.arange(4, 16), np.arange(4)]*32*2*np.pi/4096
        np.testing.assert_allclose(actual, expected, atol=4e-7)

    def test_histories_sign_and_last_action(self):
        b = ReferenceObservation()
        for i in range(3):
            obs = self.sample(b, measured_joints=np.full(16, i+1),
                               last_executed_action=np.full(16, -.25))
        self.assertTrue(b.warmed)
        np.testing.assert_array_equal(obs[16:64].reshape(3, 16)[:, 0], [3, 2, 1])
        np.testing.assert_array_equal(obs[64:67], [-3, -4, -5])
        np.testing.assert_array_equal(obs[91:107], np.full(16, -.25))
        b.reset(); self.assertFalse(b.warmed)

    def test_noncommuting_quaternion_order_and_sign(self):
        s = np.sqrt(.5)
        # cube Rx(90), goal Ry(90): expected Rx(90) @ Ry(-90).
        rx = cv2.Rodrigues(np.array([np.pi/2, 0., 0.]))[0]
        ry_inv = cv2.Rodrigues(np.array([0., -np.pi/2, 0.]))[0]
        a = self.sample(ReferenceObservation(), cube_quaternion=[s, s, 0, 0], goal_quaternion=[s, 0, s, 0])
        b = self.sample(ReferenceObservation(), cube_quaternion=[-s, -s, 0, 0], goal_quaternion=[s, 0, s, 0])
        np.testing.assert_allclose(a[73:79], (rx@ry_inv).ravel()[3:], atol=3e-7)
        np.testing.assert_allclose(a, b, atol=3e-7)

    def test_bad_input_does_not_contaminate_history(self):
        b = ReferenceObservation()
        for changes in (dict(cube_quaternion=[0, 0, 0, 0]),
                        dict(cube_position=[0, np.nan, 0]), dict(reference_step=-1),
                        dict(last_executed_action=np.full(16, 2))):
            with self.assertRaises(ValueError): self.sample(b, **changes)
            self.assertEqual(b.samples, 0)

    def test_phase_boundary(self):
        for step, expected in ((0, [0, 0]), (71, [71/72, 0]), (72, [1, 1]), (90, [1, 1])):
            obs = self.sample(ReferenceObservation(), reference_step=step)
            np.testing.assert_allclose(obs[-2:], expected, atol=1e-7)

    def test_recorded_jax_observation_roundtrip(self):
        # Reconstruct component inputs from JAX's independently produced 90
        # observations. Validates layout/history/math, not camera calibration.
        with np.load(DEFAULT/'jax_fixtures.npz') as z:
            recorded = z['rollout_observations'].copy(); steps = z['steps'].copy()
        b = ReferenceObservation(); maximum = 0.
        for index, (obs, step) in enumerate(zip(recorded, steps)):
            row1, row2 = obs[73:79].reshape(2, 3)
            rotation = np.stack([np.cross(row1, row2), row1, row2])
            rv = cv2.Rodrigues(rotation.astype(float))[0].ravel()
            angle = np.linalg.norm(rv)
            quat = np.r_[np.cos(angle/2), rv*np.sin(angle/2)/max(angle, 1e-12)]
            reconstructed = b.build(obs[:16], obs[:16]-obs[16:32], [0, 0, 0],
                                    -obs[64:67], quat, [1, 0, 0, 0], obs[91:107], int(step))
            if index >= 2:
                maximum = max(maximum, float(np.max(abs(reconstructed-obs))))
        self.assertLess(maximum, 3e-6)


if __name__ == '__main__': unittest.main()
