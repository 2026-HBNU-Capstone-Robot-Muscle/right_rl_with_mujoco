"""Analytical contact/weight cases. These tests are not robot model trials."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import unittest
import hashlib
import json
import numpy as np
from whole_hand_support import force_on_cube,summarize_contacts,SupportWindow


def contact(name,force,normal=None,body=1):
    return dict(geom=name,other_body_id=body,force_world_N=force,
                normal_N=float(np.linalg.norm(force) if normal is None else normal))


class WholeHandSupportTests(unittest.TestCase):
    def summarize(self,contacts,gravity=(0,0,-9.81)):
        return summarize_contacts(contacts,{1,2,3},gravity,.981)

    def test_palm_alone_can_carry_weight(self):
        result=self.summarize([contact('palm_collision',[0,0,.981])])
        self.assertTrue(result['supported_sample'])
        self.assertAlmostEqual(result['upward_support_over_weight'],1.)

    def test_ring_and_base_are_counted_without_named_fingertips(self):
        result=self.summarize([contact('rf_tip',[0,0,.4],body=2),
                               contact('if_bs_collision_1',[0,0,.581],body=3)])
        self.assertTrue(result['supported_sample'])
        self.assertEqual(result['loaded_hand_geoms'],['if_bs_collision_1','rf_tip'])

    def test_floor_and_nonhand_palm_name_do_not_count(self):
        for name in ('floor','palm_collision'):
            with self.subTest(name=name):
                result=self.summarize([contact(name,[0,0,.981],body=99)])
                self.assertFalse(result['supported_sample'])
                self.assertEqual(result['hand_force_world_N'],[0.,0.,0.])

    def test_external_assistance_is_not_hand_support_success(self):
        result=self.summarize([contact('rf_tip',[0,0,.5],body=2),contact('floor',[0,0,.481],body=99)])
        self.assertFalse(result['supported_sample'])
        self.assertEqual(len(result['external_loaded_contacts']),1)

    def test_contact_without_load_and_downward_or_side_only_load_fail(self):
        cases=([],[contact('palm',[0,0,0])],[contact('palm',[1.,0,0])],
               [contact('palm',[0,0,-.981])])
        for contacts in cases:
            with self.subTest(contacts=contacts):self.assertFalse(self.summarize(contacts)['supported_sample'])

    def test_gravity_axis_is_not_assumed_world_z(self):
        result=self.summarize([contact('palm',[.981,0,0])],gravity=(-9.81,0,0))
        self.assertTrue(result['supported_sample'])
        np.testing.assert_allclose(result['force_with_gravity_world_N'],0,atol=1e-12)

    def test_multiple_contact_points_accumulate_per_geometry(self):
        result=self.summarize([contact('palm',[0,0,.1962]) for _ in range(5)])
        self.assertTrue(result['supported_sample'])
        self.assertAlmostEqual(result['by_geom']['palm']['normal_N'],.981)

    def test_load_partition_between_geometries_does_not_change_support(self):
        result=self.summarize([contact('hand_geom_'+str(i),[0,0,.1635]) for i in range(6)])
        self.assertTrue(result['supported_sample'])
        self.assertEqual(result['loaded_hand_geoms'],[])
        self.assertAlmostEqual(result['upward_support_over_weight'],1.)

    def test_upward_load_boundary_does_not_pass(self):
        self.assertFalse(self.summarize([contact('palm',[0,0,.2])])['supported_sample'])

    def test_force_frame_and_geom_order_agree(self):
        frame=np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])
        np.testing.assert_allclose(force_on_cube([2.,0.,0.],frame,False),[0,2,0])
        np.testing.assert_allclose(force_on_cube([2.,0.,0.],frame,True),[0,-2,0])

    def test_nonfinite_contact_or_invalid_gravity_is_rejected(self):
        with self.assertRaises(ValueError):self.summarize([contact('palm',[0,0,np.nan],normal=1)])
        with self.assertRaises(ValueError):self.summarize([],gravity=(0,0,0))

    def test_one_snapshot_is_not_a_hold(self):
        window=SupportWindow();window.add(self.summarize([contact('palm',[0,0,.981])]))
        self.assertFalse(window.report()['support_window_passed'])

    def test_complete_window_still_has_no_pose_or_clearance_certificate(self):
        window=SupportWindow();sample=self.summarize([contact('palm',[0,0,.981])])
        for _ in range(4050):window.add(sample)
        report=window.report()
        self.assertTrue(report['support_window_passed'])
        self.assertTrue(report['stable_grasp_requires_separate_pose_drift_clearance_and_native_gates'])

    def test_eighty_percent_boundary_does_not_pass(self):
        window=SupportWindow();loaded=self.summarize([contact('palm',[0,0,.981])]);empty=self.summarize([])
        for index in range(4050):window.add(loaded if index<3240 else empty)
        self.assertEqual(window.report()['supported_fraction'],.8)
        self.assertFalse(window.report()['support_window_passed'])


if __name__=='__main__':
    program=unittest.main(verbosity=2,exit=False)
    source=Path(__file__).with_name('whole_hand_support.py')
    report=dict(status='passed' if program.result.wasSuccessful() else 'failed',
        tests_run=program.result.testsRun,failures=len(program.result.failures),errors=len(program.result.errors),
        core_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        test_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='Analytical force, contact ownership, frame/sign and duration boundary checks; no native dynamics executed',
        devices_opened=0,hardware_execution_allowed=False)
    Path(__file__).with_name('whole_hand_support_unit_verification.json').write_text(
        json.dumps(report,indent=2),encoding='utf-8')
    raise SystemExit(0 if program.result.wasSuccessful() else 1)
