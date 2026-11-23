"""Queue, hold and failure tests. No SDK port or camera is opened."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import final_grip_pose_plan_20261116 as plan
import final_grip_pose_gate_20261116 as gate
import final_grip_pose_capture_20261116 as capture
import leap_final_grip_pose_20261116 as entry

class Clock:
    def __init__(self): self.t=0.
    def now(self): return self.t
    def sleep(self, seconds): self.t+=seconds

class Tests(unittest.TestCase):
    def hold(self, *, stop_after=5, timeout=500, fail_at=None, drift_at=None, pending=False, cancel_at=None):
        clock=Clock(); times=[]; q=plan.candidate()['goals'][-1]; r={'enabled_at':0.}
        class Keys:
            def arm(self): pass
            def consume_space(self): return len(times)>=stop_after
        def cancel():
            if cancel_at is not None and len(times)>=cancel_at:
                raise KeyboardInterrupt('User stop')
        def sample(bus, command, report, phase, **kwargs):
            times.append(clock.now())
            if fail_at == len(times): raise RuntimeError('health stop')
            actual=q.copy()
            if drift_at == len(times): actual[0]+=21
            state={'position':actual, 'goal':q.copy()}
            if 'snapshot_check' in kwargs: kwargs['snapshot_check'](state)
            return state,np.zeros(16)
        bus=SimpleNamespace(pending=[q] if pending else [],checked_sample=sample,
            plan_module=SimpleNamespace(ON_TIMEOUT=timeout,SAMPLE_PERIOD=.075),
            write=lambda *a,**k: (_ for _ in ()).throw(AssertionError('hold wrote motor goal')))
        camera=SimpleNamespace(capture_named=lambda *a,**k: {'mock':True})
        protocol=capture.FinalGripPoseCapture(camera,Keys(),cancel=cancel,sleep=clock.sleep,now=clock.now)
        return protocol,bus,q,r,clock,times

    def test_queue_empty_only_native_bounds_and_no_rotation(self):
        p=plan.FinalGripPosePlan(plan.REQUEST); rows=p.plan(p.source_position())
        self.assertTrue(p.EMPTY_RELEASE_ONLY);self.assertIsNone(p.CAPTURE_BEFORE_PHASE)
        self.assertIsNone(p.CONTACT_RELEASE_BEFORE_PHASE)
        self.assertFalse(any('loaded' in n or 'yaw' in n for n,_ in rows))
        self.assertTrue(np.array_equal(rows[-1][1],plan.candidate()['goals'][-1]))
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([p.source_position(),[q for _,q in rows]]),axis=0))),2)

    def test_current_proof_covers_entire_new_path(self):
        r=gate.validate_review(plan.FinalGripPosePlan(plan.REQUEST),plan.PROOF)
        self.assertEqual(len(r['full_empty_pose']['static_cases']),201)
        self.assertEqual(len(r['full_empty_pose']['dynamic_cases']),9)
        self.assertEqual((r['current_ma'],r['sample_period_s'],r['active_motion_limit_s']),(450,.075,500))
        self.assertFalse(r['physical_cube_grasp_verified']);self.assertFalse(r['automatic_open'])
        self.assertTrue(r['all_tested_runtime_conditions_passed'])
        self.assertTrue(r['weak_drive_whole_start_guard_passed'])

    def test_old_proof_cannot_approve_new_motion(self):
        with self.assertRaises((RuntimeError,KeyError)):
            gate.validate_review(plan.FinalGripPosePlan(plan.REQUEST),plan.ROOT/'right_hand_rotation_20261112/tip_first_trial_v2/review.json')

    def test_full_screen_tamper_rejected(self):
        r=json.loads(plan.PROOF.read_bytes());r['full_empty_pose']['dynamic_cases'][0]['max_self_mm']=.1
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'tampered.json';path.write_text(json.dumps(r))
            with self.assertRaises(RuntimeError):gate.validate_review(plan.FinalGripPosePlan(plan.REQUEST),path)

    def test_hold_until_space_reads_health_without_writes(self):
        c,b,q,r,clock,times=self.hold()
        with contextlib.redirect_stdout(io.StringIO()):c.finish(b,q,r)
        self.assertEqual(r['hold_end_reason'],'operator_SPACE')
        self.assertEqual(r['final_pose_hold_goal_writes'],0)
        self.assertGreater(r['final_pose_hold_checks'],1)
        self.assertTrue(all(y-x>=.075-1e-9 for x,y in zip(times[1:],times[2:])))
        self.assertFalse(r['physical_cube_grasp_verified'])

    def test_hold_respects_total_on_limit(self):
        c,b,q,r,clock,times=self.hold(stop_after=9999,timeout=.31)
        with contextlib.redirect_stdout(io.StringIO()):c.finish(b,q,r)
        self.assertEqual(r['hold_end_reason'],'existing_500s_total_on_limit')
        self.assertLessEqual(clock.now(),.31+1e-9)

    def test_health_failure_exits_without_more_commands(self):
        c,b,q,r,clock,times=self.hold(fail_at=3)
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaisesRegex(RuntimeError,'health stop'):c.finish(b,q,r)
        self.assertEqual(len(times),3)

    def test_held_joint_drift_exits(self):
        c,b,q,r,clock,times=self.hold(drift_at=3)
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(RuntimeError):c.finish(b,q,r)
        self.assertIn('fixed_hold_fault',r)

    def test_user_cancel_exits(self):
        c,b,q,r,clock,times=self.hold(cancel_at=3)
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(KeyboardInterrupt):c.finish(b,q,r)

    def test_pending_queue_prevents_hold(self):
        c,b,q,r,clock,times=self.hold(pending=True)
        with self.assertRaises(RuntimeError):c.finish(b,q,r)
        self.assertEqual(times,[])

    def test_default_opens_no_devices(self):
        with patch('dynamixel_sdk.PortHandler',side_effect=AssertionError('motor access')), \
             patch('camera_stream.CameraStream',side_effect=AssertionError('camera access')), \
             patch('subprocess.Popen',side_effect=AssertionError('external process')), \
             patch('sys.argv',['pose']),contextlib.redirect_stdout(io.StringIO()) as log:
            self.assertEqual(entry.main(),0)
        self.assertIn('CHECK ONLY',log.getvalue())

    def test_frozen_170_sources_unchanged(self):
        frozen=json.loads((plan.ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json').read_bytes())['code_hashes']
        self.assertEqual(len(frozen),170)
        for name,digest in frozen.items():self.assertEqual(plan.sha(plan.ROOT/name),digest,name)

if __name__=='__main__':
    r=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
    result=dict(tests=r.testsRun,failures=len(r.failures),errors=len(r.errors),passed=r.wasSuccessful(),devices_opened=False)
    (plan.OUT/'software_verification.json').write_text(json.dumps(result,indent=2)+'\n')
    raise SystemExit(not r.wasSuccessful())
