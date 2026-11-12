import unittest
import numpy as np
from scipy.spatial import ConvexHull
from upper_layer_separation_certificate_20261105 import find_plane,cylinder_minimum,RADIUS
from upper_layer_separation_certificate_20261105 import SweepCertificate

def box(center,half):
    v=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)],float)*half+center;h=ConvexHull(v);edges=np.array(sorted({tuple(sorted((int(a),int(b)))) for f in h.simplices for a,b in ((f[0],f[1]),(f[1],f[2]),(f[2],f[0]))}));return v,edges,h.equations[:,:3]

class SeparationCertificateTests(unittest.TestCase):
    def check(self,center,half,expected):
        v,e,n=box(np.array(center),np.array(half));p=find_plane(v,e,n,.3);self.assertEqual(p is not None,expected)
        if p is not None:
            normal,b,gap=p;self.assertAlmostEqual(np.linalg.norm(normal),1.,places=7);self.assertLessEqual(max(v@normal),b+1e-7);self.assertGreater(float(cylinder_minimum(normal)-b),.3)
    def test_radial_separation(self):self.check([45,-18,0],[2,2,2],True)
    def test_below_layer(self):self.check([0,0,0],[10,8,10],True)
    def test_above_layer(self):self.check([0,-35,0],[10,2,10],True)
    def test_intermediate_rotation_obstruction(self):self.check([33,-18,0],[1,2,1],False)
    def test_diagonal_separation(self):self.check([32,-18,32],[2,2,2],True)
    def test_inside_sweep(self):self.check([0,-18,0],[1,1,1],False)
    def test_cylinder_support_contains_sampled_points(self):
        theta=np.linspace(0,2*np.pi,1024);points=np.vstack([np.c_[RADIUS*np.cos(theta),np.full(len(theta),y),RADIUS*np.sin(theta)] for y in (-27.5,-55/6)])
        for n in ([1,2,3],[-2,1,.4],[0,1,0],[0,-1,0]):
            normal=np.array(n,float);normal/=np.linalg.norm(normal);self.assertLessEqual(float(cylinder_minimum(normal)),float(min(points@normal))+1e-7)
    def test_rotated_hand_box(self):
        v,e,n=box(np.array([0.,0.,0.]),np.array([2.,8.,2.]));t=.7;G=np.array([[np.cos(t),-np.sin(t),0],[np.sin(t),np.cos(t),0],[0,0,1]]);v=v@G.T+[46,-18,0];p=find_plane(v,e,n@G.T,.3);self.assertIsNotNone(p)

class NativePoseCertificateTests(unittest.TestCase):
    def setUp(self):
        import mujoco
        self.mj=mujoco
        self.m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="palm"><freejoint/><geom type="box" pos=".045 -.018 0" size=".002 .002 .002"/><geom type="box" pos="0 0 0" size=".01 .008 .01"/><geom type="box" pos=".032 -.018 .032" size=".002 .002 .002"/></body></worldbody></mujoco>')
        self.d=mujoco.MjData(self.m);mujoco.mj_forward(self.m,self.d);self.cert=SweepCertificate(self.m,{self.m.body('palm').id},.3)
    def test_native_static_poses_clear(self):
        gap,bad=self.cert.query(self.d,np.zeros(3),np.eye(3));self.assertFalse(bad);self.assertGreater(gap,.3)
    def test_cached_plane_rejects_later_collision(self):
        self.cert.query(self.d,np.zeros(3),np.eye(3));self.d.qpos[0]=-.012;self.mj.mj_forward(self.m,self.d);gap,bad=self.cert.query(self.d,np.zeros(3),np.eye(3));self.assertTrue(bad);self.assertLessEqual(gap,.3)
    def test_world_and_cube_frame_transform_witnesses(self):
        from scipy.spatial.transform import Rotation
        rng=np.random.default_rng(1314);theta=np.linspace(0,2*np.pi,1024);cylinder=np.vstack([np.c_[RADIUS*np.cos(theta),np.full(len(theta),y),RADIUS*np.sin(theta)] for y in (-27.5,-55/6)])
        for _ in range(20):
            quat=Rotation.from_rotvec(rng.normal(0,.02,3)).as_quat();self.d.qpos[:3]=rng.normal(0,.0002,3);self.d.qpos[3:]=np.r_[quat[3],quat[:3]];self.mj.mj_forward(self.m,self.d);B=Rotation.from_rotvec(rng.normal(0,.02,3)).as_matrix();C=rng.normal(0,.0002,3);gap,bad=self.cert.query(self.d,C,B)
            if bad:continue
            for i,(g,v,e) in enumerate(self.cert.geometry):
                G=self.d.geom_xmat[g].reshape(3,3);world_vertices=v@G.T+self.d.geom_xpos[g];nworld=G@self.cert.normal[i];bworld=self.cert.offset[i]/1000+nworld@self.d.geom_xpos[g]
                self.assertLessEqual(float(max(world_vertices@nworld)),bworld+1e-10)
                world_cylinder=cylinder@B.T/1000+C
                self.assertGreater(float(min(world_cylinder@nworld)-bworld)*1000,.3-1e-7)
if __name__=='__main__':unittest.main()
