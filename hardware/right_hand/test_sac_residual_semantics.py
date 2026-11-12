"""Check residual composition and indexed reset used by independent evaluation."""
import unittest
from types import SimpleNamespace
import numpy as np
from sac_residual_real_start_env import ResidualFaceUp

class CaptureEnv:
    def reset_at(self,rng,index): return ('reset',index)
    def step(self,state,action): return np.asarray(action)

class ResidualSemantics(unittest.TestCase):
    def test_zero_residual_preserves_teacher_and_clips_combined_action(self):
        wrapper=ResidualFaceUp.__new__(ResidualFaceUp)
        wrapper.env=CaptureEnv(); wrapper.scale=.15
        wrapper.teacher=lambda obs: np.asarray([.95,-.95,.3])
        state=SimpleNamespace(obs={'state':np.zeros(3)})
        np.testing.assert_allclose(wrapper.step(state,np.zeros(3)),[.95,-.95,.3],atol=1e-7)
        np.testing.assert_allclose(wrapper.step(state,np.asarray([1.,-1.,-1.])),[1.,-1.,.15],atol=1e-7)
        self.assertEqual(wrapper.reset_at(None,4),('reset',4))

if __name__=='__main__': unittest.main()
