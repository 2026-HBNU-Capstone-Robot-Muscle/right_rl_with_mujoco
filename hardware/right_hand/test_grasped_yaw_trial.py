"""Bounded small-turn plan and fault-injected transports; no real devices."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

import grasped_yaw_plan as p
import leap_thumb_prefix_empty as runner
import leap_thumb_prepare_current_empty as current
from grasped_yaw_capture import GraspedYawCapture
from slow_grasp_demo_plan import OBSERVATION_PHASE
from test_grasp_place_trial import Bus,transport
from test_hand_preparation_timing import Clock
from thumb_live_start_gate import validate_proof
from slow_grasp_demo_plan import demo_opening

ROOT=p.ROOT
FIXTURE=ROOT/'right_hand_rotation_20261023/grasped_yaw_fixture'


class PlanTests(unittest.TestCase):
    def test_contact_refinement_is_small_and_held_out_shortfall_is_not_hidden(self):
        plan=p.GraspedYawPlan(FIXTURE/'fresh_off_request.json')
        candidate=json.loads(p.CANDIDATE.read_bytes())
        self.assertEqual(candidate['version'],'contact_refined_small_yaw_candidate_v1')
        self.assertEqual(candidate['contact_refinement_samples'],20)
        self.assertEqual(candidate['rotation_samples'],70)
        refinement=plan._yaw[:20]
        self.assertEqual(int(np.max(abs(refinement-plan._closed))),18)
        np.testing.assert_array_equal(refinement[-1]-plan._closed,candidate['closure_delta_ticks'])
        self.assertFalse(candidate['full_held_out_angle_criterion_passed'])
        self.assertFalse(candidate['selected']['held_out_pass'])
        self.assertFalse(candidate['robust_policy'])
        self.assertFalse(candidate['live_sac'])
        self.assertFalse(candidate['physical_cube_rotation_verified'])
        self.assertTrue(candidate['experimental_trial_only'])
        cases=candidate['selected']['active_cube_replays']+candidate['selected']['held_out_cases']
        self.assertEqual(len(cases),9)
        self.assertTrue(all(row['yaw_about_gravity_up_deg']>0 for row in cases))
        self.assertLess(min(row['yaw_about_gravity_up_deg'] for row in cases),1.)

    def test_changed_source_evidence_is_rejected_before_any_binding(self):
        candidate=json.loads(p.CANDIDATE.read_bytes())
        relative=p.CANDIDATE_SOURCES[0];candidate['source_hashes'][relative]='0'*64
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'candidate.json';blob=json.dumps(candidate).encode('utf-8');path.write_bytes(blob)
            with patch.object(p,'CANDIDATE',path),patch.object(p,'CANDIDATE_SHA',hashlib.sha256(blob).hexdigest()), \
                 self.assertRaisesRegex(ValueError,'source evidence changed'):
                p.GraspedYawPlan(FIXTURE/'fresh_off_request.json')

    def test_candidate_cannot_claim_a_robust_policy_after_failed_angle_case(self):
        candidate=json.loads(p.CANDIDATE.read_bytes());candidate['robust_policy']=True
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'candidate.json';blob=json.dumps(candidate).encode('utf-8');path.write_bytes(blob)
            with patch.object(p,'CANDIDATE',path),patch.object(p,'CANDIDATE_SHA',hashlib.sha256(blob).hexdigest()), \
                 self.assertRaisesRegex(ValueError,'Invalid bounded'):
                p.GraspedYawPlan(FIXTURE/'fresh_off_request.json')

    def test_saved_LED_start_finishes_ring_return_before_thumb_moves(self):
        folder=ROOT/'right_hand_rotation_20261023/grasped_yaw_LED_complete_fixture'
        plan=p.GraspedYawPlan(folder/'fresh_off_request.json');raw=plan.source_position()
        old=demo_opening(raw,plan._opened);new=p.yaw_opening(raw,plan._opened)
        before=np.array([q for phase,q in old if phase=='stage_0_empty_recover'])
        release=np.array([q for phase,q in new if phase=='stage_0_empty_recover'])
        np.testing.assert_array_equal(before,release)
        ring=np.array([q for phase,q in new if phase==p.RING_CLEARANCE_PHASE])
        self.assertTrue(np.any(ring[:,12:]!=release[-1,12:]))
        np.testing.assert_array_equal(ring[:,:12],np.tile(release[-1,:12],(len(ring),1)))
        np.testing.assert_array_equal(ring[-1,12:],plan._opened[12:])
        following=np.array([q for phase,q in new if phase in ('stage_1_thumb_clearance','stage_2_empty_open')])
        np.testing.assert_array_equal(following[:,12:],np.tile(plan._opened[12:],(len(following),1)))
        np.testing.assert_array_equal(new[-1][1],plan._opened)
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([raw,[q for _,q in new]]),axis=0))),2)
        np.testing.assert_array_equal(plan.frozen_for_phase(p.RING_CLEARANCE_PHASE),np.arange(12))
        self.assertEqual(plan.CONTACT_RELEASE_BEFORE_PHASE,p.RING_CLEARANCE_PHASE)

    def test_unreleased_ring_keeps_original_route_after_fixed_checkpoint(self):
        plan=p.GraspedYawPlan(FIXTURE/'fresh_off_request.json');raw=plan.source_position()
        old=demo_opening(raw,plan._opened);new=p.yaw_opening(raw,plan._opened)
        self.assertEqual(next(q[12] for phase,q in reversed(old) if phase=='stage_0_empty_recover'),raw[12])
        retained=[(phase,q) for phase,q in new if phase!=p.RING_CLEARANCE_PHASE]
        self.assertEqual([phase for phase,_ in retained],[phase for phase,_ in old])
        np.testing.assert_array_equal([q for _,q in retained],[q for _,q in old])
        checkpoint=[q for phase,q in new if phase==p.RING_CLEARANCE_PHASE]
        self.assertEqual(len(checkpoint),20)
        np.testing.assert_array_equal(checkpoint,np.tile(checkpoint[0],(20,1)))

    def test_native_proof_and_bounded_turn_preserve_baseline_and_after_hold(self):
        plan=p.GraspedYawPlan(FIXTURE/'fresh_off_request.json');raw=plan.source_position()
        proof=validate_proof(plan,FIXTURE/'native_screen.json')
        self.assertEqual(len(proof['static_screens']),67);self.assertEqual(len(proof['dynamic_sensitivity']),3)
        rows=plan.plan(raw);q=np.array([x for _,x in rows]);phases=[phase for phase,_ in rows]
        baseline=q[np.array(phases)==OBSERVATION_PHASE];turn=q[np.array(phases)==p.TURN_PHASE]
        after=q[np.array(phases)==p.AFTER_PHASE]
        self.assertEqual(baseline.shape,(60,16));self.assertEqual(turn.shape,(90,16));self.assertEqual(after.shape,(40,16))
        np.testing.assert_array_equal(baseline,np.tile(plan._closed,(60,1)))
        np.testing.assert_array_equal(after,np.tile(turn[-1],(40,1)))
        np.testing.assert_array_equal(turn[:,p.FROZEN_MOTORS],np.tile(plan._closed[p.FROZEN_MOTORS],(90,1)))
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([raw,q]),axis=0))),2)
        self.assertLessEqual(np.max(abs(turn-plan._closed)),64)
        self.assertLess(phases.index(OBSERVATION_PHASE),phases.index(p.TURN_PHASE))
        self.assertLess(phases.index(p.TURN_PHASE),phases.index(p.AFTER_PHASE))
        self.assertEqual(plan.ON_TIMEOUT,75)
        np.testing.assert_array_equal(plan.frozen_for_phase(p.AFTER_PHASE),np.arange(16))
        self.assertEqual(len(plan.progress_for_phase(p.AFTER_PHASE)),0)

    def test_binding_and_enable_adoption_do_not_touch_files(self):
        plan=p.GraspedYawPlan(FIXTURE/'fresh_off_request.json');raw=plan.source_position()
        bus=object.__new__(runner.Bus);bus.plan_module=plan
        with patch.object(Path,'read_bytes',side_effect=AssertionError('ON file access')), \
             patch.object(Path,'read_text',side_effect=AssertionError('ON file access')):
            bus.bind(raw);bus.pending.pop(0);bus.adopt(raw);plan.validate_start(raw);plan.plan(raw)
        self.assertEqual(len(bus.pending),len(plan.plan(raw))+1)

    def test_changed_candidate_is_rejected_and_start_drift_still_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'candidate.json';path.write_text('{}',encoding='utf-8')
            with patch.object(p,'CANDIDATE',path),self.assertRaisesRegex(ValueError,'candidate changed'):
                p.GraspedYawPlan(FIXTURE/'fresh_off_request.json')
        plan=p.GraspedYawPlan(FIXTURE/'fresh_off_request.json')
        with self.assertRaisesRegex(ValueError,'screened'):plan.validate_start(plan.source_position()+5)

    def test_removal_is_after_saved_observation_and_never_claims_rotation_success(self):
        protocol=GraspedYawCapture(None,None,cancel=lambda:None);protocol.guard=object()
        report={'cube_placement_confirmed':True};state={'position':np.arange(16)}
        def remove(phase,observe):
            self.assertEqual(phase,'remove_cube')
            self.assertEqual(report['rotation_evidence_cutoff_camera_monotonic'],10.)
            self.assertTrue(report['rotation_command_path_completed'])
            return state
        with contextlib.redirect_stdout(io.StringIO()), \
             patch.object(protocol,'_observer',return_value=lambda phase:state), \
             patch.object(protocol,'_photo',return_value=(state,dict(image='after.png',camera_read_monotonic=10.))), \
             patch.object(protocol,'_space',side_effect=remove):
            protocol.finish(None,np.arange(16),report)
        self.assertFalse(report['physical_rotation_verified']);self.assertFalse(report['physical_flip_verified'])
        self.assertTrue(report['cube_removal_confirmed']);self.assertFalse(report['cube_present'])


class TransportTests(unittest.TestCase):
    def execute(self,fault=None):
        clock=Clock();bus=Bus(clock);report={};entered=[]
        base=json.loads((ROOT/'prepared_cube_review_20261022/grasp_trial_v6_fixture/request.json').read_bytes())['off_state']
        base.update(json.loads((FIXTURE/'fresh_off_request.json').read_bytes())['off_state'])
        bus.state={k:np.array(v) for k,v in base.items()};original=bus.snapshot;reader=Path.read_bytes
        def read_file(path):
            if np.any(bus.state['torque']):raise AssertionError('ON historical file access')
            return reader(path)
        def snapshot():
            if report.get('phase')==p.TURN_PHASE:
                entered.append(bus.goal_attempts)
                if fault=='read':raise RuntimeError('fake yaw read failure')
                if fault=='recovered_read' and not report.get('read_failures'):
                    report.setdefault('read_failures',[]).append(dict(recovered=True,kind='fake_recovered_read'))
            return original()
        bus.snapshot=snapshot
        def cancel():
            if report.get('phase')==p.TURN_PHASE:
                if fault=='q':raise KeyboardInterrupt('Q')
                if fault=='camera':raise RuntimeError('fake camera failure')
                if fault=='deadline':clock.t=report['enabled_at']+76
            clock.cancel()
        def screen(request,output,**kwargs):
            plan=p.GraspedYawPlan(request);proof=json.loads((FIXTURE/'native_screen.json').read_bytes())
            proof.update(source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(plan.source_position()))
            Path(output).write_text(json.dumps(proof),encoding='utf-8')
        def release(b,q,r,g):
            r['release_goal_attempt']=b.goal_attempts
            if fault!='no_release':r['contact_release_operator_confirmed']=True
        def place(b,q,r,g):
            r['placement_goal_attempt']=b.goal_attempts
            if fault!='no_place':r['cube_placement_confirmed']=True
        def finish(b,q,r):r.update(cube_removal_confirmed=True,physical_rotation_verified=False)
        with tempfile.TemporaryDirectory() as temp,contextlib.redirect_stdout(io.StringIO()), \
             patch.object(Path,'read_bytes',read_file),patch.object(runner.checked.Bus,'write',transport), \
             patch.object(runner.checked.time,'monotonic',clock.now),patch.object(runner.checked.time,'sleep',clock.sleep):
            try:
                current.prepare_and_run(bus,temp,report,plan_type=p.GraspedYawPlan,screener=screen,
                    cancel=cancel,sleep=clock.sleep,now=clock.now,
                    release_callback=release,phase_callback=place,prepared_callback=finish)
            except BaseException as exc:return bus,report,exc,entered
        return bus,report,None,entered

    def test_one_enable_full_before_turn_after_hold_then_off_restoration(self):
        bus,r,error,entered=self.execute();self.assertIsNone(error);self.assertEqual(len(entered),90)
        self.assertEqual(r['status'],p.GraspedYawPlan.STATUS)
        self.assertTrue(r['settings_restored'] and r['all_torque_off_verified'])
        self.assertEqual(sum(a==64 and np.any(v) for a,v in bus.events),1)
        self.assertEqual(sum(s['phase']==OBSERVATION_PHASE for s in r['samples']),60)
        self.assertEqual(sum(s['phase']==p.AFTER_PHASE for s in r['samples']),40)
        self.assertFalse(r['physical_rotation_verified'])

    def test_read_camera_and_q_in_turn_stop_without_motion_retry(self):
        for fault in ('read','camera','q'):
            bus,r,error,entered=self.execute(fault)
            self.assertIsNotNone(error);self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
            self.assertNotEqual(r.get('status'),p.GraspedYawPlan.STATUS)
            if fault=='read':self.assertEqual(bus.goal_attempts,entered[0])

    def test_read_recovery_adds_two_observations_without_extra_goals(self):
        normal,_,error,_=self.execute();self.assertIsNone(error)
        bus,r,error,entered=self.execute('recovered_read');self.assertIsNone(error)
        self.assertEqual(bus.goal_attempts,normal.goal_attempts)
        np.testing.assert_array_equal([q for a,q in bus.events if a==116],
                                      [q for a,q in normal.events if a==116])
        self.assertEqual(len(entered),92)
        self.assertEqual(r['read_recovery_confirmation_events'][0]['clean_reads'],2)
        self.assertEqual(r['read_recovery_confirmation_events'][0]['goal_writes'],0)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_release_and_place_confirmation_are_required_before_turn(self):
        for fault,key in (('no_release','release_goal_attempt'),('no_place','placement_goal_attempt')):
            bus,r,error,entered=self.execute(fault)
            self.assertIsNotNone(error);self.assertFalse(entered)
            self.assertEqual(bus.goal_attempts,r[key]);self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_75second_deadline_is_not_disabled(self):
        bus,r,error,entered=self.execute('deadline')
        self.assertIn('75s',str(error));self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])


if __name__=='__main__':unittest.main()
