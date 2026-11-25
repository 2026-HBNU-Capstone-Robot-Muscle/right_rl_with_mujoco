"""Behavior tests of the real directional wrapper, using a controlled pose source."""
from dataclasses import dataclass, replace
from types import SimpleNamespace
import unittest
import numpy as np
import jax.numpy as jp
from sac_forward_task_env import ForwardFirstFlip
from sac_forward_geometry import target_quaternion


@dataclass
class Record:
    def replace(self,**changes): return replace(self,**changes)


@dataclass
class Data(Record):
    qpos: object
    qvel: object
    ctrl: object
    mocap_quat: object


@dataclass
class State(Record):
    data: object
    info: object
    metrics: object
    obs: object
    reward: object=0.
    done: object=0.


class PoseSource:
    _config=SimpleNamespace(chain_goals=False,face_onehot_obs=False,history_len=3)
    degrees=0.
    def reset(self,rng):
        qpos=jp.r_[jp.zeros(16),jp.asarray([0.,0.,.1,1.,0.,0.,0.])]
        data=Data(qpos,jp.zeros(22),jp.zeros(16),jp.asarray([[1.,0.,0.,0.]]))
        info={k:jp.zeros(n) for k,n in [('qpos_error_history',48),
                                       ('cube_pos_error_history',9),('cube_ori_error_history',18)]}
        return State(data,info,{}, {})
    def _get_obs(self,data,info): return {'goal':data.mocap_quat[0]}
    def step(self,state,action):
        a=jp.deg2rad(self.degrees)/2
        q=jp.asarray([jp.cos(a),0.,0.,-jp.sin(a)])
        return state.replace(data=state.data.replace(qpos=state.data.qpos.at[-4:].set(q)))


class ForwardWrapperTest(unittest.TestCase):
    def test_goal_in_observation_matches_reward(self):
        state=ForwardFirstFlip(PoseSource()).reset(None)
        np.testing.assert_allclose(state.obs['goal'],target_quaternion([1.,0.,0.,0.]),atol=1e-6)
    def test_hold_crossing_ninety_completes(self):
        base=PoseSource(); env=ForwardFirstFlip(base); state=env.reset(None)
        for i,degree in enumerate([89,90,91]):
            base.degrees=degree; state=env.step(state,jp.zeros(16))
            self.assertEqual(bool(state.done),i==2)
        self.assertTrue(bool(state.metrics['forward/success']))
    def test_wrong_direction_never_accumulates_success(self):
        base=PoseSource(); base.degrees=-90
        env=ForwardFirstFlip(base); state=env.reset(None)
        for _ in range(5): state=env.step(state,jp.zeros(16))
        self.assertFalse(bool(state.metrics['forward/success']))
        self.assertEqual(int(state.info['forward_goal_hold']),0)


if __name__=='__main__': unittest.main()
