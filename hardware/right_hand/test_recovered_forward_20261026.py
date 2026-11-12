"""Check the narrow initial-contact scope and unchanged operator checkpoints."""
import contextlib,copy,io,json,subprocess,sys,tempfile,unittest
from pathlib import Path
import numpy as np
from interrupted_thumb_contact_policy_20261026 import metadata,RECORDED,PAIR
from recovered_forward_plan_20261026 import RecoveredForwardPlan,ROOT,FIXTURE
from forward_cube_placement_20261026 import ForwardPlacementPlan
from test_forward_cube_placement_20261026 import ForwardPlacementChecks
from recovered_forward_gate_20261026 import validate_proof

class RecoveryChecks(unittest.TestCase):
    def test_scope_is_only_known_surfaces_at_admitted_off_start(self):
        r=metadata(RECORDED)
        self.assertTrue(r['active']);self.assertEqual(r['pair'],PAIR)
        self.assertEqual(r['initial_pair_limit_mm'],4.)
        for key in ('geometry_and_contact_forces_changed','actual_health_stops_changed'):self.assertFalse(r[key])
        self.assertTrue(r['static_monotonic_contact_separation_required'])
        self.assertTrue(r['dynamic_no_contact_beyond_initial_envelope'])
        self.assertEqual(r['additional_initial_pairs'],['palm_collision_6|th_tip'])
        self.assertTrue(r['new_contacts_forbidden']);self.assertEqual(r['after_release_self_overlap_limit_mm'],1e-6)
        settled=RECORDED.copy();settled[0]-=50;settled[1]+=26
        self.assertTrue(metadata(settled)['active']);self.assertTrue(metadata(settled)['start_admission_required'])
        self.assertFalse(metadata(settled)['previous_pose_match_required'])
        with self.assertRaises(ValueError):metadata(np.full(16,np.nan))

    def test_recovery_changes_no_motor_goals_or_motion_limits(self):
        p=RecoveredForwardPlan(FIXTURE/'request.json');a=p.source_position()
        b=ForwardPlacementPlan.__new__(ForwardPlacementPlan)
        b.SOURCE=p.SOURCE;b.SOURCE_SHA=p.SOURCE_SHA;b.center=p.center.copy()
        # Both algorithms are inherited unchanged; use the same admitted raw
        # values to compare every target and phase rather than plan versions.
        self.assertIs(p.plan.__func__,b.plan.__func__)
        left=p.plan(a);right=b.plan(a)
        self.assertEqual([q for q,v in left],[q for q,v in right])
        np.testing.assert_array_equal([v for q,v in left],[v for q,v in right])
        self.assertEqual(p.ON_TIMEOUT,b.ON_TIMEOUT)

    def run_trial(self,fail=None):
        case=ForwardPlacementChecks();case.plan=RecoveredForwardPlan(FIXTURE/'request.json')
        with contextlib.redirect_stdout(io.StringIO()):return case.run_path(fail)

    def test_full_same_loaded_sequence_still_requires_place_and_remove(self):
        r,bus,protocol,_=self.run_trial()
        self.assertEqual(len(bus.loaded_goals),824)
        self.assertEqual([v.args[0] for v in protocol._space.call_args_list],['place_cube','remove_cube'])
        self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
        self.assertTrue(r['cube_removal_confirmed']);self.assertFalse(r['physical_flip_verified'])

    def test_cancel_before_place_sends_no_loaded_goals_and_restores(self):
        r,bus,_,_=self.run_trial('place_cube')
        self.assertEqual(len(bus.loaded_goals),0)
        self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])

    def test_default_is_device_free(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_recovered_forward_placement_20261026.py')],capture_output=True,text=True,encoding='utf-8',timeout=8)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)

class NativeProofChecks(unittest.TestCase):
    def test_proof_rejects_scope_or_monotonic_separation_tampering(self):
        plan=RecoveredForwardPlan(FIXTURE/'request.json');proof=validate_proof(plan,FIXTURE/'screen.json')
        self.assertTrue(proof['recorded_interrupted_contact_policy']['active'])
        for change in ('scope','new_contact'):
            bad=copy.deepcopy(proof)
            if change=='scope':bad['recorded_interrupted_contact_policy']['initial_pair_limit_mm']=4.1
            else:bad['static_screens'][0]['self_escape_max_unapproved_overlap_mm']=.01
            with tempfile.TemporaryDirectory() as d:
                f=Path(d)/'bad.json';f.write_text(json.dumps(bad),encoding='utf-8')
                with self.assertRaises(RuntimeError):validate_proof(plan,f)

if __name__=='__main__':unittest.main()
