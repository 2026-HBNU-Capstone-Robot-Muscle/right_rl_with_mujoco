"""Regression of the two user-run, zero-write v5 screen stops."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from grasp_place_plan import GraspPlacePlan
from thumb_live_start_gate import validate_proof, screen_failure_details
from review_ring_release_screen_20261022 import FOLDERS, ROOT

BASE=ROOT/'prepared_cube_review_20261022'


class Tests(unittest.TestCase):
    def test_budget_uses_existing_goal_window_without_changing_timing_or_step_size(self):
        self.assertEqual(GraspPlacePlan.INITIAL_SELF_ESCAPE_GOALS,40)
        self.assertEqual(GraspPlacePlan.MAX_GOAL_STEP,2)
        self.assertEqual(GraspPlacePlan.MAX_CONTACT_RELEASE_TRAVEL_TICKS,80)
        self.assertEqual(GraspPlacePlan.MAX_CONTACT_RELEASE_TRAVEL_TICKS,
                         GraspPlacePlan.INITIAL_SELF_ESCAPE_GOALS*GraspPlacePlan.MAX_GOAL_STEP)
        self.assertEqual(GraspPlacePlan.INITIAL_SELF_ESCAPE_DYNAMIC_S,2.5)

    def test_actual_failed_screens_passed_other_limits_and_sent_no_writes(self):
        for name in FOLDERS:
            with self.subTest(folder=name):
                r=json.loads((ROOT/name/'result.json').read_bytes())
                proof=json.loads((ROOT/name/'current_start_screen.json').read_bytes())
                self.assertEqual(r['write_events'],[])
                self.assertTrue(r['all_torque_off_verified'])
                self.assertFalse(proof['accepted'])
                self.assertEqual(max(s['contact_clear_pip_travel_ticks'] for s in proof['static_screens']),74)
                self.assertEqual(max(s['self_escape_complete_at'] for s in proof['static_screens']),37)
                for s in proof['static_screens']+proof['dynamic_sensitivity']:
                    self.assertEqual(s['self_escape_max_unapproved_overlap_mm'],0)
                    self.assertTrue(s['self_escape_complete'])
                for s in proof['static_screens']:
                    self.assertEqual(s['max_support_increase_mm'],0)
                    self.assertEqual(s['unapproved_native_range_violation_rad'],0)
                self.assertLessEqual(max(s['self_escape_complete_at'] for s in proof['dynamic_sensitivity']),2.5)

    def test_all_seven_new_plans_have_identical_goal_sequences_to_their_v5_sources(self):
        for folder in BASE.glob('grasp_trial_v6_*fixture'):
            with self.subTest(folder=folder.name):
                request=json.loads((folder/'request.json').read_bytes())
                old=ROOT/request['offline_replay_source']
                proofpath=old.parent/'screen.json'
                if not proofpath.exists():proofpath=old.parent/'current_start_screen.json'
                proof=json.loads(proofpath.read_bytes())
                plan=GraspPlacePlan(folder/'request.json')
                self.assertEqual(plan.plan_digest(plan.source_position()),proof['plan_sha256'])
                np.testing.assert_array_equal(plan.plan(plan.source_position())[-1][1],proof['last_goal_ticks'])

    def test_current_pose_has_real_native_proof_under_the_same_motion_path(self):
        folder=BASE/'grasp_trial_v6_125613_fixture'
        proof=validate_proof(GraspPlacePlan(folder/'request.json'),folder/'screen.json')
        self.assertEqual(len(proof['static_screens']),67)
        self.assertEqual(len(proof['dynamic_sensitivity']),3)
        self.assertEqual(max(s['contact_clear_pip_travel_ticks'] for s in proof['static_screens']),74)

    def test_distance_cannot_exceed_the_actual_goal_count_even_inside_the_overall_budget(self):
        folder=BASE/'grasp_trial_v6_fixture';plan=GraspPlacePlan(folder/'request.json')
        proof=json.loads((folder/'screen.json').read_bytes())
        row=proof['static_screens'][0]
        row['contact_clear_pip_travel_ticks']=74
        row['contact_clear_pip_goal_ticks']=proof['start_ticks'][14]-74
        row['self_escape_complete_at']=36  # At <=2 ticks/goal, only <=72 is possible.
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'proof.json';path.write_text(json.dumps(proof),encoding='utf-8')
            with self.assertRaisesRegex(RuntimeError,'Static clearance'):validate_proof(plan,path)

    def test_diagnostics_use_active_budget_and_still_describe_legacy_failure(self):
        source=json.loads((ROOT/FOLDERS[-1]/'current_start_screen.json').read_bytes())
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'proof.json';path.write_text(json.dumps(source),encoding='utf-8')
            self.assertIn('ring contact could not clear',screen_failure_details(path))
            source['plan_version']=GraspPlacePlan.PLAN_VERSION
            path.write_text(json.dumps(source),encoding='utf-8')
            self.assertNotIn('ring contact could not clear',screen_failure_details(path))
            source['static_screens'][0]['contact_clear_pip_travel_ticks']=81
            path.write_text(json.dumps(source),encoding='utf-8')
            self.assertIn('ring contact could not clear',screen_failure_details(path))


if __name__=='__main__':unittest.main()
