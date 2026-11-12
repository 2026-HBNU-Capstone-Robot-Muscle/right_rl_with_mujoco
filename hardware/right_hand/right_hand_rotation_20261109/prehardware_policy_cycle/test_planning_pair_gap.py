"""Analytical geometry checks for the bounded planning cost, without MuJoCo."""
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from types import SimpleNamespace
import unittest
import numpy as np
from planning_pair_gap import PlanningPairGap


VERTICES=np.array([[x,y,z] for x in (-1.,1.) for y in (-1.,1.) for z in (-1.,1.)])


def scene(delta,rotation=None,translation=None):
    rotation=np.eye(3) if rotation is None else rotation;translation=np.zeros(3) if translation is None else translation
    cost=PlanningPairGap.__new__(PlanningPairGap);cost.cache={0:(VERTICES,np.eye(3)),1:(VERTICES,np.eye(3))}
    data=SimpleNamespace(geom_xpos=np.array([translation,translation+rotation@np.array(delta)]),geom_xmat=np.array([rotation.ravel(),rotation.ravel()]))
    return cost,data


class Geometry(unittest.TestCase):
    def test_separated_boxes(self):
        cost,data=scene([3,0,0]);self.assertAlmostEqual(cost.gap(data,0,1),1.,12)
    def test_touching_boxes(self):
        cost,data=scene([2,0,0]);self.assertAlmostEqual(cost.gap(data,0,1),0.,12)
    def test_intersecting_boxes_have_no_positive_certificate(self):
        cost,data=scene([1.5,0,0]);self.assertLessEqual(cost.gap(data,0,1),0.)
    def test_coincident_centers_are_finite(self):
        cost,data=scene([0,0,0]);self.assertTrue(np.isfinite(cost.gap(data,0,1)));self.assertLess(cost.gap(data,0,1),0.)
    def test_reversing_pair_keeps_gap(self):
        cost,data=scene([3,.2,-.4]);self.assertAlmostEqual(cost.gap(data,0,1),cost.gap(data,1,0),12)
    def test_rigid_transform_of_known_separation(self):
        angle=.73;rotation=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1.]])
        cost,data=scene([3,0,0],rotation,np.array([100.,-70.,3.]));self.assertAlmostEqual(cost.gap(data,0,1),1.,11)
    def test_positive_bounds_do_not_exceed_feasible_point_distances(self):
        random=np.random.default_rng(713)
        for _ in range(50):
            delta=random.uniform(-5,5,3);cost,data=scene(delta)
            lower=cost.gap(data,0,1)
            first=random.uniform(-1,1,(100,3));second=random.uniform(-1,1,(100,3))+delta
            self.assertTrue(np.all(lower<=np.linalg.norm(first-second,axis=1)+1e-12))


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromTestCase(Geometry))
    out=Path(__file__).resolve().parent
    report=dict(status='passed' if result.wasSuccessful() else 'failed',tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        source_sha256=hashlib.sha256((out/'planning_pair_gap.py').read_bytes()).hexdigest(),test_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        native_dynamics_or_complete_clearance_not_certified=True,devices_opened=0)
    (out/'planning_pair_gap_verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    if not result.wasSuccessful():raise SystemExit(1)
