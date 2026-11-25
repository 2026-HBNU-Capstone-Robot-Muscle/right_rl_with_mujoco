"""Empty-only ring suffix, delay budget and motor fault tests; fake devices."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import supported_yaw_empty_plan as p
from grasp_place_plan import GraspPlacePlan
import leap_thumb_prefix_empty as runner
import leap_thumb_prepare_current_empty as current
from test_grasp_place_trial import Bus,transport
from test_hand_preparation_timing import Clock

FIXTURE=p.ROOT/'right_hand_rotation_20261022/fixture_supported_current'


class Tests(unittest.TestCase):
    def execute(self,fault=None,confirm=True,fixture=FIXTURE):
        clock=Clock();bus=Bus(clock);report={};entered=[]
        bus.state={k:np.asarray(v) for k,v in json.loads((fixture/'request.json').read_bytes())['off_state'].items()}
        original=bus.snapshot
        def snapshot():
            if report.get('phase')==p.SUFFIX_PHASE:
                entered.append(bus.goal_attempts)
                if fault=='read':raise RuntimeError('injected suffix read fault')
            return original()
        bus.snapshot=snapshot
        def write(self,addr,size,values,verify=True):
            if addr==116 and report.get('phase')==p.SUFFIX_PHASE and fault=='write':
                self.goal_attempts+=1;raise RuntimeError('injected suffix write fault')
            return transport(self,addr,size,values,verify)
        def cancel():
            if report.get('phase')==p.SUFFIX_PHASE and fault=='camera':raise RuntimeError('camera blank')
            clock.cancel()
        def now():
            return clock.now()+(76 if fault=='deadline' and report.get('phase')==p.SUFFIX_PHASE else 0)
        def screen(request,output,**kw):
            plan=p.SupportedYawEmptyPlan(request);proof=json.loads((fixture/'screen.json').read_bytes())
            proof.update(source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(plan.source_position()))
            Path(output).write_text(json.dumps(proof),encoding='utf-8')
        def release(b,c,r,g):
            r['release_goal_count']=b.goal_attempts
            if confirm:r['contact_release_operator_confirmed']=True
        def finish(b,c,r):r['cube_was_allowed']=False
        with tempfile.TemporaryDirectory() as folder,contextlib.redirect_stdout(io.StringIO()), \
             patch.object(runner.checked.Bus,'write',write), \
             patch.object(runner.checked.time,'monotonic',clock.now), \
             patch.object(runner.checked.time,'sleep',clock.sleep):
            try:
                current.prepare_and_run(bus,folder,report,plan_type=p.SupportedYawEmptyPlan,screener=screen,
                    cancel=cancel,sleep=clock.sleep,now=now,prepared_callback=finish,
                    release_callback=release)  # No PLACE callback exists.
            except BaseException as exc:return bus,report,exc,entered,clock
        return bus,report,None,entered,clock

    def test_complete_empty_suffix_uses_no_placement_and_single_enable(self):
        b,r,error,entered,clock=self.execute();self.assertIsNone(error);self.assertTrue(entered)
        self.assertEqual(r['status'],p.SupportedYawEmptyPlan.STATUS)
        self.assertFalse(r['cube_was_allowed']);self.assertNotIn('cube_placement_confirmed',r)
        goals=np.asarray([v for a,v in b.events if a==116])
        np.testing.assert_array_equal(goals[-256:],p.candidate())
        self.assertLessEqual(np.max(abs(np.diff(goals,axis=0))),2)
        self.assertTrue(np.any(goals[-1,12:]!=goals[-257,12:]))
        self.assertEqual(sum(a==64 and np.any(v) for a,v in b.events),1)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
        self.assertLess(r['samples'][-1]['t'],75)
        self.assertEqual(GraspPlacePlan.ON_TIMEOUT,55)

    def test_missing_release_observation_cannot_start_new_motion(self):
        b,r,error,entered,_=self.execute(confirm=False)
        self.assertIsNotNone(error);self.assertFalse(entered)
        self.assertEqual(b.goal_attempts,r['release_goal_count'])
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_suffix_faults_stop_without_retry_and_restore_off(self):
        for fault in ('read','write','camera'):
            with self.subTest(fault=fault):
                b,r,error,entered,_=self.execute(fault)
                self.assertIsNotNone(error);self.assertGreater(len(b.pending),0)
                if fault=='read':self.assertEqual(b.goal_attempts,entered[0])
                if fault=='write':self.assertEqual(b.goal_attempts,entered[0]+1)
                self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_palm_curl_and_unbounded_candidate_are_rejected(self):
        with patch.object(p,'CANDIDATE_SHA','changed'):
            with self.assertRaisesRegex(ValueError,'candidate changed'):p.candidate()
        q=p.candidate();base=p.endpoints()[1]
        self.assertTrue(np.all(q[:,[12,14,15]]<=base[[12,14,15]]))
        self.assertLessEqual(np.max(abs(q-base)),256)

    def test_cycle_joint_queries_do_not_reread_historical_json(self):
        plan=p.SupportedYawEmptyPlan(FIXTURE/'request.json')
        expected=plan.frozen_for_phase(p.SUFFIX_PHASE)
        with patch.object(p,'candidate',side_effect=AssertionError('Disk read inside cycle')), \
             patch.object(p,'endpoints',side_effect=AssertionError('Historical JSON inside cycle')):
            np.testing.assert_array_equal(plan.frozen_for_phase(p.SUFFIX_PHASE),expected)
            self.assertEqual(len(expected)+len(plan.progress_for_phase(p.SUFFIX_PHASE)),16)

    def test_75_second_budget_stops_before_suffix_goal_and_restores(self):
        b,r,error,entered,_=self.execute('deadline')
        self.assertIn('75s',str(error));self.assertFalse(entered)
        self.assertGreater(len(b.pending),0)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_failed_sag_start_extends_ring_tip_with_pip_then_keeps_it_fixed(self):
        source=p.ROOT/'right_hand_rotation_20261022/fixture_supported_failed_sag/request.json'
        plan=p.SupportedYawEmptyPlan(source);raw=plan.source_position();rows=plan.plan(raw)
        release=np.array([q for phase,q in rows if phase=='stage_0_empty_recover'])
        self.assertEqual(int(raw[12]-release[-1,12]),64)
        self.assertTrue(np.all(np.diff(np.r_[raw[12],release[:,12]])<=0))
        self.assertTrue(np.all(np.diff(np.r_[raw[14],release[:,14]])<=0))
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([raw,release]),axis=0))),2)
        fixed=plan.frozen_for_phase('stage_0_empty_recover')
        self.assertNotIn(12,fixed);self.assertIn(12,plan.progress_for_phase('stage_0_empty_recover'))
        for phase,q in rows:
            if phase=='stage_1_thumb_clearance':self.assertEqual(q[12],release[-1,12])

    def test_saved_failed_start_completes_with_new_certificate_and_no_limit_relaxation(self):
        fixture=p.ROOT/'right_hand_rotation_20261022/fixture_supported_failed_sag'
        b,r,error,entered,_=self.execute(fixture=fixture)
        self.assertIsNone(error);self.assertTrue(entered)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
        old=json.loads((p.ROOT/'supported_yaw_empty_20261022_155659_749370/current_start_screen.json').read_bytes())
        self.assertFalse(old['accepted'])
        self.assertGreater(old['dynamic_sensitivity'][0]['self_escape_max_unapproved_overlap_mm'],1e-6)
        from thumb_live_start_gate import validate_proof
        new=validate_proof(p.SupportedYawEmptyPlan(fixture/'request.json'),fixture/'screen.json')
        self.assertTrue(new['native_ranges_unchanged'])
        self.assertEqual(p.SupportedYawEmptyPlan.INITIAL_SELF_ESCAPE_DYNAMIC_S,2.5)
        self.assertEqual(p.SupportedYawEmptyPlan.MAX_CONTACT_RELEASE_TRAVEL_TICKS,80)
        for row in new['static_screens']+new['dynamic_sensitivity']:
            self.assertLessEqual(row['self_escape_max_unapproved_overlap_mm'],1e-6)
            self.assertTrue(row['self_escape_complete'])

    def test_short_release_keeps_original_tip_goals_and_fixed_guard(self):
        source=p.ROOT/'right_hand_rotation_20261022/fixture_supported_loaded_stop/request.json'
        plan=p.SupportedYawEmptyPlan(source);raw=plan.source_position()
        rows=plan.plan(raw)
        release=[q for phase,q in rows if phase=='stage_0_empty_recover']
        self.assertTrue(all(q[12]==raw[12] for q in release))
        self.assertIn(12,plan.frozen_for_phase('stage_0_empty_recover'))
        self.assertNotIn(12,plan.progress_for_phase('stage_0_empty_recover'))


if __name__=='__main__':unittest.main()
