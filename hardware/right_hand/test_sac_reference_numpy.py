"""Export contract failures and independent recorded JAX parity."""
import unittest
import numpy as np
from sac_reference_numpy import ReferencePolicy, check


class ExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = ReferencePolicy()

    def test_independent_jax_rollout_and_random_inputs(self):
        self.assertEqual(check()['status'], 'passed')

    def test_rejects_old_policy_observation_dimension(self):
        with self.assertRaises(ValueError):
            self.policy.action(np.zeros(108))

    def test_rejects_missing_vision_nan(self):
        obs = np.zeros(110); obs[64] = np.nan
        with self.assertRaises(ValueError):
            self.policy.command(obs, self.policy.base, 0)

    def test_rejects_unknown_previous_goal(self):
        with self.assertRaises(ValueError):
            self.policy.command(np.zeros(110), np.full(16, np.nan), 0)

    def test_rejects_negative_phase(self):
        with self.assertRaises(ValueError):
            self.policy.reference(-1)


if __name__ == '__main__':
    unittest.main()
