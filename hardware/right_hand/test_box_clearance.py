import unittest
import numpy as np
from box_clearance import clearance

class BoxClearanceTests(unittest.TestCase):
    def test_saved_hand_pairs_against_independent_optimization(self):
        import json
        from pathlib import Path
        from scipy.optimize import minimize
        path=Path(__file__).resolve().parent/'backoff_video_review_20261016/middle_contact_audit.json'
        cases=json.loads(path.read_text())['distance_disagreements']
        self.assertGreater(len(cases),0)
        for row in cases:
            # Millimetres improve optimizer scaling on small physical parts.
            ca,ra,ha=map(np.array,row['box_a']);cb,rb,hb=map(np.array,row['box_b'])
            ca,ha,cb,hb=(v*1000 for v in (ca,ha,cb,hb))
            got,overlap=clearance(ca,ra,ha,cb,rb,hb)
            A=np.concatenate((ra,-rb),axis=1);offset=ca-cb
            def fun(x):
                delta=offset+A@x
                return np.dot(delta,delta),2*A.T@delta
            opt=minimize(fun,np.zeros(6),jac=True,bounds=list(zip(-np.r_[ha,hb],np.r_[ha,hb])),
                         method='L-BFGS-B',options={'ftol':1e-14,'gtol':1e-10,'maxiter':1000})
            self.assertTrue(opt.success,opt.message)
            self.assertFalse(overlap)
            self.assertAlmostEqual(got,np.sqrt(max(0,opt.fun)),delta=1e-4)

    def test_known_face_edge_corner_and_overlap(self):
        for center, expected, overlap in [([3,0,0],1,False), ([3,3,0],2**.5,False),
                ([3,3,3],3**.5,False), ([1,0,0],0,True), ([2,0,0],0,True)]:
            result, hit = clearance(np.zeros(3),np.eye(3),np.ones(3),np.array(center),np.eye(3),np.ones(3))
            self.assertAlmostEqual(result,expected,places=10)
            self.assertEqual(hit,overlap)

    def test_random_rotated_boxes_against_independent_optimization(self):
        from scipy.optimize import minimize
        from scipy.spatial.transform import Rotation
        rng=np.random.default_rng(19)
        for _ in range(80):
            ca=rng.normal(size=3);cb=rng.normal(size=3)
            ra=Rotation.random(random_state=rng).as_matrix();rb=Rotation.random(random_state=rng).as_matrix()
            ha=rng.uniform(.1,.6,3);hb=rng.uniform(.1,.6,3)
            got,_=clearance(ca,ra,ha,cb,rb,hb)
            A=np.concatenate((ra,-rb),axis=1);offset=ca-cb
            def fun(x):
                delta=offset+A@x
                return np.dot(delta,delta),2*A.T@delta
            opt=minimize(fun,np.zeros(6),jac=True,bounds=list(zip(-np.r_[ha,hb],np.r_[ha,hb])),
                         method='SLSQP',options={'ftol':1e-13,'maxiter':300})
            self.assertTrue(opt.success,opt.message)
            self.assertAlmostEqual(got,np.sqrt(max(0,opt.fun)),delta=2e-6)
            reverse,_=clearance(cb,rb,hb,ca,ra,ha)
            self.assertAlmostEqual(got,reverse,places=10)

if __name__=='__main__':unittest.main()
