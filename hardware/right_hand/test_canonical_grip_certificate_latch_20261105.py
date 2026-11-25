"""A failed clearance proof stays failed while dynamics still runs fully."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import unittest
from unittest.mock import patch
import numpy as np,mujoco
import canonical_grip_dynamics_20261105 as module

def fixture():
    links=''.join(f'<body pos="{2+i*.1} 0 0"><joint name="j{i}" type="hinge"/><geom type="box" size=".005 .005 .005" mass=".01" contype="0" conaffinity="0"/></body>' for i in range(16))
    motors=''.join(f'<position joint="j{i}" kp="1"/>' for i in range(16))
    xml=f'<mujoco><option gravity="0 0 0" timestep=".001"/><worldbody><body name="palm"><geom type="box" size=".1 .1 .005"/>{links}</body><body name="cube" pos="0 0 .0325" quat=".7071067811865476 -.7071067811865475 0 0"><freejoint/><geom name="cube" type="box" size=".0275 .0275 .0275" mass=".1"/></body></worldbody><actuator>{motors}</actuator></mujoco>'
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d);z={k:getattr(d,k).copy() for k in ('qpos','qvel','ctrl','act','qacc_warmstart')};z.update(time=0.,up=np.array([0.,0.,1.]),initial_cube_rotation=d.xmat[m.body('cube').id].reshape(3,3).copy(),initial_cube_center=d.qpos[-7:-4].copy());return m,z

class FakeCertificate:
    fail=False
    def __init__(self,*args):self.queries=0
    def query(self,*args):
        self.queries+=1
        return (-.1,[0]) if self.fail and self.queries==1 else (1.,[])
    def metadata(self):return dict(queries=self.queries,synthetic_fault_for_test_only=True)

class ClearanceLatchTests(unittest.TestCase):
    def test_first_failure_never_becomes_a_pass(self):
        m,z=fixture()
        class OnceBad(FakeCertificate):fail=True
        with patch.object(module,'SweepCertificate',OnceBad):result=module.run(m,z,np.tile(2048,(134,16)),1)
        self.assertTrue(result['safety_passed']);self.assertEqual(result['completed_goals'],134);self.assertEqual(result['settled_tail']['certificate']['queries'],1);self.assertEqual(result['settled_tail']['separation_certificate_bad_steps_1ms'],1);self.assertEqual(result['settled_tail']['certificate_unqueried_steps_after_first_failure'],4049);self.assertFalse(result['whole_grip_task_passed'])
    def test_passing_proofs_still_check_every_tail_state(self):
        m,z=fixture()
        with patch.object(module,'SweepCertificate',FakeCertificate):result=module.run(m,z,np.tile(2048,(134,16)),1)
        self.assertTrue(result['safety_passed']);self.assertEqual(result['settled_tail']['certificate']['queries'],4050);self.assertEqual(result['settled_tail']['separation_certificate_bad_steps_1ms'],0);self.assertEqual(result['settled_tail']['certificate_unqueried_steps_after_first_failure'],0)
if __name__=='__main__':unittest.main()
