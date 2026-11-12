import os
os.environ['MUJOCO_GL']='egl'
os.environ['JAX_PLATFORMS']='cpu'
import unittest,numpy as np,mujoco
from render_canonical_grip_evidence_20261105 import extra,display_physical_cube,cube_display
from cube_task_frame_20261105 import normal

class RenderDisplayTests(unittest.TestCase):
    def setUp(self):
        self.m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="cube"><freejoint/><geom name="cube" type="box" size=".0275 .0275 .0275" mass=".1" group="3"/><geom name="old_visual" type="box" size=".035 .035 .035" mass="0" contype="0" conaffinity="0" group="2"/></body><body name="goal" mocap="true" pos="0 0 .2"><geom type="box" size=".035 .035 .035" contype="0" conaffinity="0" group="2"/></body></worldbody></mujoco>')
    def test_physical_size_preserved_and_shown(self):
        props={n:getattr(self.m,n).copy() for n in ('geom_size','geom_contype','geom_conaffinity','body_mass','body_inertia')};report=display_physical_cube(self.m);self.assertTrue(np.allclose(report['physical_cube_side_mm'],55));self.assertEqual(self.m.geom_group[self.m.geom('cube').id],2);self.assertEqual(self.m.geom_group[self.m.geom('old_visual').id],5)
        for n,v in props.items():self.assertTrue(np.array_equal(getattr(self.m,n),v))
    def test_mocap_goal_hidden(self):
        display_physical_cube(self.m);goal=self.m.body('goal').id;self.assertTrue(all(self.m.geom_group[g]==5 for g in range(self.m.ngeom) if self.m.geom_bodyid[g]==goal))
    def test_sweep_height_matches_physical_layer(self):
        scene=mujoco.MjvScene(self.m,maxgeom=10);r=.0275*np.sqrt(2);half=.055/6;extra(scene,mujoco.mjtGeom.mjGEOM_CYLINDER,[r,half,0],[0,0,0],np.eye(3),[1,.5,0,.2]);self.assertTrue(np.allclose(scene.geoms[0].size,[r,r,half]))
    def test_capsule_connector_half_length(self):
        scene=mujoco.MjvScene(self.m,maxgeom=10);extra(scene,mujoco.mjtGeom.mjGEOM_CAPSULE,[.00025,.0275,0],[0,0,0],np.eye(3),[0,0,0,1]);self.assertTrue(np.allclose(scene.geoms[0].size,[.00025,.00025,.0275]))
    def test_selected_face_sweep_follows_all_six_normals(self):
        B=np.array([[0.,-1,0],[0,0,-1],[1,0,0]]);C=np.array([.1,.2,.3])
        for face in range(6):
            scene=mujoco.MjvScene(self.m,maxgeom=30);cube_display(scene,C,B,True,face);sweep=scene.geoms[12];up=B@normal(face)
            self.assertTrue(np.allclose(sweep.pos,C+up*.055/3))
            self.assertAlmostEqual(abs(float(sweep.mat.reshape(3,3)[:,2]@up)),1.)
            self.assertTrue(np.allclose(sweep.size,[.0275*np.sqrt(2),.0275*np.sqrt(2),.055/6]))
    def test_no_sweep_without_requested_working_layer(self):
        scene=mujoco.MjvScene(self.m,maxgeom=30);cube_display(scene,np.zeros(3),np.eye(3));self.assertEqual(scene.ngeom,12)
if __name__=='__main__':unittest.main()
