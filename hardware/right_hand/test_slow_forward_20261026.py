"""Validate slower pacing, unchanged limits, and the real placement/abort flow."""
import contextlib,copy,io,json,subprocess,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import sequence_flip_checked as checked
import test_loaded_quarter_finish_20261026 as prior
from slow_forward_plan_20261026 import SlowForwardPlan,ROOT,FIXTURE,TURN_PHASE,FINISH_PHASE
from slow_forward_protocol_20261026 import SlowForwardCapture
from recovered_forward_plan_20261026 import RecoveredForwardPlan
from slow_forward_gate_20261026 import validate_proof
from slow_thumb_seam_resolution_20261026 import classify,PAIR,metadata

base_protocol=prior.protocol
def protocol(bus,fail):
    p=base_protocol(bus,fail);p.__class__=SlowForwardCapture;return p

class SlowFlowChecks(unittest.TestCase):
    def setUp(self):self.plan=SlowForwardPlan(FIXTURE/'request.json')
    def run_trial(self,fail=None):
        case=prior.LoadedFinishChecks();case.plan=self.plan
        with patch.object(prior,'protocol',protocol):return case.run_path(fail)
    def test_same_positions_at75ms_and_bounded180s(self):
        a=self.plan.source_position();old=RecoveredForwardPlan.__new__(RecoveredForwardPlan)
        old.SOURCE=self.plan.SOURCE;old.SOURCE_SHA=self.plan.SOURCE_SHA;old.center=self.plan.center.copy()
        slow=self.plan.plan(a);fast=old.plan(a)
        self.assertEqual([p for p,q in slow],[p for p,q in fast]);np.testing.assert_array_equal([q for p,q in slow],[q for p,q in fast])
        self.assertEqual(self.plan.SAMPLE_PERIOD,.075);self.assertEqual(self.plan.ON_TIMEOUT,180)
        self.assertLess((len(slow)+40)*.085+2,self.plan.ON_TIMEOUT)
    def test_full_run_retains_place_remove_and_no_physical_success_claim(self):
        r,bus,p,_=self.run_trial();self.assertEqual(len(bus.loaded_goals),824)
        self.assertEqual([v.args[0] for v in p._space.call_args_list],['place_cube','remove_cube'])
        self.assertEqual(r['requested_sample_period_s'],.075)
        self.assertEqual(r['expected_reference_seconds'],57.)
        self.assertEqual(r['expected_new_finish_seconds'],4.8)
        self.assertFalse(r['physical_flip_verified']);self.assertFalse(r['placement_comparison']['voltage_improvement_verified'])
        self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
    def test_cancel_at_place_still_sends_no_loaded_goals(self):
        r,bus,_,_=self.run_trial('place_cube');self.assertEqual(len(bus.loaded_goals),0)
        self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
    def test_health_failure_still_blocks_finish_and_restores(self):
        r,bus,_,_=self.run_trial('health');self.assertEqual(len(bus.loaded_goals),760)
        self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
    def test_35v_still_stops_in_slow_mode(self):
        goal=np.full(16,2200,dtype=int)
        state=dict(torque=np.ones(16,dtype=int),error=np.zeros(16,dtype=int),watchdog=np.full(16,25),
            current_cap=checked.CAP.copy(),velocity=np.zeros(16,dtype=int),acceleration=np.zeros(16,dtype=int),
            voltage=np.full(16,48),temperature=np.full(16,24),current=np.zeros(16,dtype=int),position=goal.copy(),goal=goal.copy(),
            **{k:v.copy() for k,v in checked.GAINS.items()})
        checked.health(state,goal)
        state['voltage'][0]=35
        with self.assertRaisesRegex(RuntimeError,'Undervoltage'):checked.health(state,goal)
        self.assertEqual(checked.MAX_TRACKING_RAD,.18)
    def test_default_opens_no_devices_and_reports75ms(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_slow_forward_placement_20261026.py')],capture_output=True,text=True,encoding='utf-8',timeout=8)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('75ms cycles',r.stdout)
        self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)

class SlowProofChecks(unittest.TestCase):
    def test_rejects_wrong_cycle_or_claim_of_electrical_validation(self):
        p=SlowForwardPlan(FIXTURE/'request.json');r=validate_proof(p,FIXTURE/'screen.json')
        for key,value in (('control_cycle_seconds',.05),('initial_hold_seconds',.75),('voltage_supply_model_identified',True)):
            bad=copy.deepcopy(r);bad[key]=value
            with tempfile.TemporaryDirectory() as d:
                f=Path(d)/'bad.json';f.write_text(json.dumps(bad),encoding='utf-8')
                with self.assertRaises(RuntimeError):validate_proof(p,f)

class SeamResolutionChecks(unittest.TestCase):
    def test_only_the_known_sub10micrometre_seam_is_classified_as_zero(self):
        pairs={PAIR:.004853874956628179,'unrelated|contact':.005}
        before=pairs.copy();result=classify(pairs)
        self.assertNotIn(PAIR,result);self.assertEqual(result['unrelated|contact'],.005)
        self.assertEqual(pairs,before)
    def test_larger_or_other_contacts_are_retained(self):
        self.assertEqual(classify({PAIR:.0101}),{PAIR:.0101})
        self.assertFalse(metadata()['collision_parameters_or_contact_forces_disabled'])
        self.assertFalse(metadata()['actual_motor_health_stops_changed'])
if __name__=='__main__':unittest.main()
