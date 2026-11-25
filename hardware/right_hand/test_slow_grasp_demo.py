"""Real saved-path proof + fake transport/camera only. No devices."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
from unittest.mock import patch
import unittest
import numpy as np

import leap_thumb_prefix_empty as runner
import leap_thumb_prepare_current_empty as current
import slow_grasp_demo_plan as p
from marker_trial_camera import MarkerTrialCamera
from test_grasp_place_trial import Bus,transport
from test_hand_preparation_timing import Clock
from thumb_live_start_gate import validate_proof

ROOT=p.ROOT
FIXTURE=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_fixture'


class PlanTests(unittest.TestCase):
    def test_real_start_uses_reached_endpoints_and_fixed_hold_only(self):
        plan=p.SlowGraspDemoPlan(FIXTURE/'fresh_off_request.json');raw=plan.source_position()
        proof=validate_proof(plan,FIXTURE/'native_screen.json')
        self.assertTrue(proof['accepted']);self.assertEqual(len(proof['static_screens']),67)
        self.assertEqual(len(proof['dynamic_sensitivity']),3)
        rows=plan.plan(raw);hold=[q for phase,q in rows if phase==p.OBSERVATION_PHASE]
        self.assertEqual(len(hold),60)
        np.testing.assert_array_equal(hold,np.tile(p.endpoints()[1],(60,1)))
        q=np.vstack([raw,*[r[1] for r in rows]])
        self.assertLessEqual(np.max(abs(np.diff(q,axis=0))),2)
        self.assertFalse(any('rotation' in phase for phase,q in rows))
        self.assertEqual(plan.ON_TIMEOUT,60);self.assertEqual(plan.MAX_START_RANGE_EXCESS_RAD,.06)
        self.assertEqual(plan.INITIAL_SELF_ESCAPE_DYNAMIC_S,3.)
        self.assertEqual(p.GraspPlacePlan.INITIAL_SELF_ESCAPE_DYNAMIC_S,2.5)
        self.assertEqual(plan.INITIAL_SELF_ESCAPE_GOALS,44)
        self.assertEqual(plan.MAX_CONTACT_RELEASE_TRAVEL_TICKS,88)
        self.assertEqual(p.GraspPlacePlan.INITIAL_SELF_ESCAPE_GOALS,40)
        self.assertEqual(p.GraspPlacePlan.MAX_CONTACT_RELEASE_TRAVEL_TICKS,80)

    def test_on_binding_and_enable_adoption_do_not_read_any_files(self):
        from leap_thumb_prefix_empty import Bus as Scheduled
        plan=p.SlowGraspDemoPlan(FIXTURE/'fresh_off_request.json');raw=plan.source_position()
        bus=object.__new__(Scheduled);bus.plan_module=plan
        with patch.object(Path,'read_bytes',side_effect=AssertionError('File read while ON')), \
             patch.object(Path,'read_text',side_effect=AssertionError('File read while ON')):
            bus.bind(raw);bus.pending.pop(0);bus.adopt(raw);plan.validate_start(raw);plan.plan(raw)
        self.assertEqual(len(bus.pending),len(plan.plan(raw))+1)

    def test_mixed_passive_pose_admitted_only_for_native_screen_not_motion(self):
        plan=p.SlowGraspDemoPlan(FIXTURE/'fresh_off_request.json');raw=plan.source_position()
        opened,closed=p.endpoints()
        self.assertGreater(abs(raw[6]-closed[6]),512)
        self.assertLess(abs(raw[6]-opened[6]),512)
        p.admission(raw)
        bad=raw.copy();bad[0]=0
        with self.assertRaisesRegex(ValueError,'512-tick'):p.admission(bad)
        bad=raw.copy();bad[3]=int(plan._lower[3]-.061/p.TICK)
        with self.assertRaisesRegex(ValueError,'native re-entry'):p.admission(bad)
        with self.assertRaisesRegex(ValueError,'screened'):plan.validate_start(raw+5)

    def test_observation_freezes_all_joints_without_progress_commands(self):
        plan=p.SlowGraspDemoPlan(FIXTURE/'fresh_off_request.json')
        np.testing.assert_array_equal(plan.frozen_for_phase(p.OBSERVATION_PHASE),np.arange(16))
        self.assertEqual(len(plan.progress_for_phase(p.OBSERVATION_PHASE)),0)


class CameraTests(unittest.TestCase):
    def test_fixed_exposure_camera_is_explicit_and_marker_loss_is_not_motion_gate(self):
        from empty_trial_camera import EmptyTrialCamera
        with tempfile.TemporaryDirectory() as temp:
            report={}
            def fake_init(obj,folder,r,**kwargs):obj.report=r;obj.worker_options=kwargs['worker'].keywords
            with patch.object(EmptyTrialCamera,'__init__',fake_init):
                camera=MarkerTrialCamera(Path(temp)/'camera',report)
            self.assertFalse(report['marker_visibility_is_motion_gate'])
            self.assertEqual(camera.worker_options['display_mode'],'record')
            camera.restoration_path.write_text(json.dumps(dict(restoration=dict(restored_original_readbacks=True))),encoding='utf-8')
            with patch.object(EmptyTrialCamera,'finish',return_value=dict(status='complete')):
                result=camera.finish()
            self.assertEqual(result['status'],'complete');self.assertTrue(report['camera_exposure_restoration_verified'])

    def test_missing_restore_evidence_is_not_claimed_as_restored(self):
        from empty_trial_camera import EmptyTrialCamera
        with tempfile.TemporaryDirectory() as temp:
            camera=MarkerTrialCamera.__new__(MarkerTrialCamera)
            camera.restoration_path=Path(temp)/'not_written.json';camera.report={}
            with patch.object(EmptyTrialCamera,'finish',return_value=dict(status='complete')):
                result=camera.finish()
            self.assertEqual(result['status'],'incomplete');self.assertFalse(result['exposure_restoration_verified'])


class TransportTests(unittest.TestCase):
    def execute(self,fault=None):
        clock=Clock();bus=Bus(clock);report={};entered=[]
        base=json.loads((ROOT/'prepared_cube_review_20261022/grasp_trial_v6_fixture/request.json').read_bytes())['off_state']
        current_state=json.loads((FIXTURE/'fresh_off_request.json').read_bytes())['off_state']
        base.update(current_state);bus.state={k:np.array(v) for k,v in base.items()}
        original=bus.snapshot;original_bytes=Path.read_bytes
        def read_file(path):
            if np.any(bus.state['torque']):raise AssertionError('File read while torque ON')
            return original_bytes(path)
        def snapshot():
            if report.get('phase')==p.OBSERVATION_PHASE:
                entered.append(bus.goal_attempts)
                if fault=='read':raise RuntimeError('fake fixed-hold read failure')
            return original()
        bus.snapshot=snapshot
        def cancel():
            if report.get('phase')==p.OBSERVATION_PHASE:
                if fault=='q':raise KeyboardInterrupt('Q')
                if fault=='camera':raise RuntimeError('fake camera stopped')
                if fault=='deadline':clock.t=report['enabled_at']+61
            clock.cancel()
        def screen(request,output,**kwargs):
            plan=p.SlowGraspDemoPlan(request);saved=json.loads((FIXTURE/'native_screen.json').read_bytes())
            saved.update(source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(plan.source_position()))
            Path(output).write_text(json.dumps(saved),encoding='utf-8')
        def release(b,q,r,g):
            r['release_attempt']=b.goal_attempts
            if fault!='no_release':r['contact_release_operator_confirmed']=True
        def place(b,q,r,g):
            r['place_attempt']=b.goal_attempts
            if fault!='no_place':r['cube_placement_confirmed']=True
        def finish(b,q,r):r.update(cube_removal_confirmed=True,physical_flip_verified=False)
        with tempfile.TemporaryDirectory() as folder,contextlib.redirect_stdout(io.StringIO()), \
             patch.object(Path,'read_bytes',read_file),patch.object(runner.checked.Bus,'write',transport), \
             patch.object(runner.checked.time,'monotonic',clock.now),patch.object(runner.checked.time,'sleep',clock.sleep):
            try:
                current.prepare_and_run(bus,folder,report,plan_type=p.SlowGraspDemoPlan,screener=screen,
                    cancel=cancel,sleep=clock.sleep,now=clock.now,
                    release_callback=release,phase_callback=place,prepared_callback=finish)
            except BaseException as exc:return bus,report,exc,entered
        return bus,report,None,entered

    def test_demo_has_one_enable_and_exact_hold_then_off_and_restore(self):
        bus,report,error,entered=self.execute()
        self.assertIsNone(error);self.assertTrue(entered)
        self.assertEqual(report['status'],p.SlowGraspDemoPlan.STATUS)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertEqual(sum(a==64 and np.any(v) for a,v in bus.events),1)
        goals=np.array([v for a,v in bus.events if a==116])
        np.testing.assert_array_equal(goals[-60:],np.tile(p.endpoints()[1],(60,1)))
        self.assertFalse(report['physical_flip_verified'])

    def test_hold_read_q_camera_failures_stop_without_goal_retry(self):
        for fault in ('read','q','camera'):
            bus,report,error,entered=self.execute(fault)
            self.assertIsNotNone(error);self.assertTrue(entered or fault in ('q','camera'))
            self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
            self.assertNotEqual(report.get('status'),p.SlowGraspDemoPlan.STATUS)
            if fault=='read':self.assertEqual(bus.goal_attempts,entered[0])

    def test_missing_release_or_placement_confirmation_blocks_following_goals(self):
        for fault,key in (('no_release','release_attempt'),('no_place','place_attempt')):
            bus,report,error,entered=self.execute(fault)
            self.assertIsNotNone(error);self.assertFalse(entered)
            self.assertEqual(bus.goal_attempts,report[key])
            self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])

    def test_deadline_still_stops_and_restores(self):
        bus,report,error,entered=self.execute('deadline')
        self.assertIsNotNone(error);self.assertIn('60s',str(error))
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])


if __name__=='__main__':unittest.main()
