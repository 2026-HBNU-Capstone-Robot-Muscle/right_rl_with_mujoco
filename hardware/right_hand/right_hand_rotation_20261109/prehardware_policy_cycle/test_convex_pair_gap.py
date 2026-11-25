"""Analytical separation, overlap and rigid-frame tests, without MuJoCo."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import unittest
import hashlib
import json
import numpy as np
from convex_pair_gap import signed_axis_gap

BOX=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)],float)
I=np.eye(3)


class ConvexGapTests(unittest.TestCase):
    def gap(self,shift,rotation=I):
        return signed_axis_gap(BOX,BOX@rotation.T+shift,I,I@rotation.T,I,I@rotation.T)

    def test_separated_boxes(self):self.assertAlmostEqual(self.gap([2.3,0,0]),.3)
    def test_touching_boxes(self):self.assertAlmostEqual(self.gap([2,0,0]),0.)
    def test_overlapping_boxes(self):self.assertAlmostEqual(self.gap([1.8,0,0]),-.2)
    def test_rotated_box(self):
        angle=np.pi/4;R=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        self.assertAlmostEqual(self.gap([3,0,0],R),2-np.sqrt(2))
    def test_common_rigid_transform(self):
        angle=.93;R=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
        translation=np.array([12.,-3.,7.]);a=BOX@R.T+translation;b=(BOX+[2.3,0,0])@R.T+translation
        self.assertAlmostEqual(signed_axis_gap(a,b,I@R.T,I@R.T,I@R.T,I@R.T),.3)
    def test_submicron_gap_remains_signed(self):
        self.assertGreater(self.gap([2+2e-7,0,0]),0)
        self.assertLess(self.gap([2-2e-7,0,0]),0)
    def test_nonfinite_input_rejected(self):
        with self.assertRaises(ValueError):self.gap([np.nan,0,0])


if __name__=='__main__':
    program=unittest.main(verbosity=2,exit=False)
    source=Path(__file__).with_name('convex_pair_gap.py')
    report=dict(status='passed' if program.result.wasSuccessful() else 'failed',tests_run=program.result.testsRun,
        failures=len(program.result.failures),errors=len(program.result.errors),
        geometry_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        test_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        analytic_geometry_not_a_native_or_physical_trial=True,devices_opened=0,hardware_execution_allowed=False)
    Path(__file__).with_name('convex_pair_gap_verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    raise SystemExit(0 if program.result.wasSuccessful() else 1)
