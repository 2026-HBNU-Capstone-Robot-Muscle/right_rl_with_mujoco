"""Support contact must invalidate a simultaneous orientation success."""
from dataclasses import dataclass
from types import SimpleNamespace
import unittest
import jax.numpy as jp
from test_sac_forward_env import Data,PoseSource
from sac_supported_start_env import SupportedForward

@dataclass
class ContactData(Data):
    contact: object=None

class ContactSource(PoseSource):
    support_ids=jp.asarray([100,101])
    contact_ids=jp.asarray([[2,3],[4,5]])
    distances=jp.asarray([-.001,.01])
    def reset(self,rng):
        state=super().reset(rng)
        return state.replace(data=ContactData(**vars(state.data),contact=SimpleNamespace(geom=self.contact_ids,dist=self.distances)))
    def step(self,state,action):
        state=super().step(state,action)
        return state.replace(data=state.data.replace(contact=SimpleNamespace(geom=self.contact_ids,dist=self.distances)))

class SupportTest(unittest.TestCase):
    def test_extended_hold_does_not_terminate_at_three_steps(self):
        source=ContactSource();source.degrees=90;env=SupportedForward(source);env.required_hold_steps=23;state=env.reset(None)
        for step in range(23):
            state=env.step(state,jp.zeros(16))
            self.assertEqual(bool(state.done),step==22)
            self.assertEqual(bool(state.metrics['forward/success']),step==22)
    def test_drop_still_terminates_during_extended_hold(self):
        source=ContactSource();source.degrees=90;env=SupportedForward(source);env.required_hold_steps=23;state=env.reset(None)
        for _ in range(2):state=env.step(state,jp.zeros(16))
        state=state.replace(data=state.data.replace(qpos=state.data.qpos.at[-5].set(-.06)))
        state=env.step(state,jp.zeros(16))
        self.assertTrue(bool(state.done))
    def test_support_fault_cannot_count_as_success(self):
        source=ContactSource();source.degrees=90;env=SupportedForward(source);state=env.reset(None)
        for _ in range(2):state=env.step(state,jp.zeros(16))
        source.contact_ids=jp.asarray([[100,3],[4,5]])
        state=env.step(state,jp.zeros(16))
        self.assertTrue(bool(state.done));self.assertTrue(bool(state.metrics['support/fault']))
        self.assertEqual(float(state.metrics['forward/success']),0.)
        self.assertEqual(int(state.info['forward_goal_hold']),0)
        self.assertLess(float(state.reward),0.)
    def test_unrelated_contact_does_not_fail_support_gate(self):
        source=ContactSource();source.degrees=90;env=SupportedForward(source);state=env.reset(None)
        for _ in range(3):state=env.step(state,jp.zeros(16))
        self.assertFalse(bool(state.metrics['support/fault']));self.assertTrue(bool(state.metrics['forward/success']))
    def test_full_contact_buffer_stops_instead_of_ignoring_omissions(self):
        source=ContactSource();source.distances=jp.asarray([-.001,-.001])
        env=SupportedForward(source);state=env.step(env.reset(None),jp.zeros(16))
        self.assertTrue(bool(state.done));self.assertTrue(bool(state.metrics['support/fault']))
    def test_separated_support_pair_does_not_fail(self):
        source=ContactSource();source.contact_ids=jp.asarray([[4,100],[3,101]]);source.distances=jp.asarray([.001,.01])
        env=SupportedForward(source);state=env.step(env.reset(None),jp.zeros(16))
        self.assertFalse(bool(state.done));self.assertEqual(float(state.metrics['support/max_penetration_mm']),0.)

if __name__=='__main__':unittest.main()
