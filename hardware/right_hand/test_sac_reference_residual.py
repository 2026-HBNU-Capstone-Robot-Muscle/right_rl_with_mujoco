"""Reference timing, correction units, and episode reset behavior."""
from types import SimpleNamespace
import unittest
import jax.numpy as jp
import numpy as np
from test_sac_forward_env import Data,State
from sac_reference_residual_env import ReferenceResidual

class TargetSource:
    dt=.05
    _config=SimpleNamespace(action_scale=.2)
    _safe_lowers=jp.full(16,-2.);_safe_uppers=jp.full(16,2.)
    def step(self,state,action):
        return state.replace(data=state.data.replace(ctrl=state.data.ctrl+.2*action),obs={'state':jp.zeros(3)})

class ReferenceTest(unittest.TestCase):
    def setUp(self):
        self.env=ReferenceResidual.__new__(ReferenceResidual);self.env.env=TargetSource()
        self.env.base=jp.zeros(16);self.env.targets=jp.asarray([[.24]*16,[.48]*16,[.72]*16])
        self.env.move_seconds=3.6;self.env.scale=.05
        self.state=State(Data(jp.zeros(23),jp.zeros(22),jp.zeros(16),jp.zeros((1,4))),{'reference_step':jp.asarray(0)}, {},{'state':jp.zeros(5)})
    def test_twenty_hz_waypoints_and_hold(self):
        for step,target in [(0,.01),(23,.24),(24,.25),(47,.48),(71,.72),(100,.72)]:
            np.testing.assert_allclose(self.env.desired_target(jp.asarray(step)),target,atol=1e-6)
    def test_zero_correction_tracks_absolute_reference(self):
        state=self.state
        for step in range(100):
            state=self.env.step(state,jp.zeros(16))
            np.testing.assert_allclose(state.data.ctrl,self.env.desired_target(jp.asarray(step)),atol=1e-6)
            self.assertEqual(state.obs['state'].shape,(5,))
    def test_correction_is_not_integrated(self):
        state=self.state
        for step in range(20):state=self.env.step(state,jp.ones(16))
        np.testing.assert_allclose(state.data.ctrl,.25,atol=1e-6)
    def test_wrapper_episode_reset_resets_reference_clock(self):
        info={'reference_step':jp.asarray(50),'steps':jp.asarray(0)}
        state=self.env.step(self.state.replace(info=info),jp.zeros(16))
        self.assertEqual(int(state.info['reference_step']),1)
        np.testing.assert_allclose(state.data.ctrl,.01,atol=1e-6)

if __name__=='__main__':unittest.main()
