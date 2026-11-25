"""Continuation path and failure tests, no real camera/serial connection."""
import contextlib
import hashlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_thumb_prefix_empty as runner
import leap_thumb_continuation_empty as next_trial
import thumb_continuation_plan as p
from test_leap_thumb_prefix_empty import FakeBus, fixture


class ContinuationFake(FakeBus, next_trial.Bus):
    plan_module=p
    planner=staticmethod(p.validate_start)

    def __init__(self, fault=None):
        super().__init__(fault)
        a=p.source_position()
        self.state['position']=a.copy()
        self.state['goal']=a.copy()


class Tests(unittest.TestCase):
    def execute(self, b, cancel=lambda:None):
        report={}
        with patch.object(runner.checked.Bus,'write',FakeBus.transport),contextlib.redirect_stdout(io.StringIO()):
            try:
                runner.run(b,{k:v.copy() for k,v in b.state.items()},report,
                           cancel=cancel,sleep=lambda _:None)
            except BaseException as exc:
                return report,exc
        return report,None

    def test_new_source_and_nominal_exact_path(self):
        b=ContinuationFake();report,exc=self.execute(b)
        self.assertIsNone(exc)
        self.assertEqual(report['status'],p.STATUS)
        self.assertEqual(report['source_sha256'],p.SOURCE_SHA)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        goals=np.array([v for a,v in b.events if a==116])
        base=p.source_position()
        self.assertEqual(len(goals),362)
        self.assertLessEqual(abs(np.diff(goals,axis=0)).max(),2)
        np.testing.assert_array_equal(goals[:,p.FROZEN],np.tile(base[p.FROZEN],(362,1)))
        np.testing.assert_array_equal(goals[:122,:4],np.tile(base[:4],(122,1)))
        self.assertEqual(int(max(abs(goals[-1,:4]-base[:4]))),384)
        self.assertTrue(np.all(np.diff(goals[:,2])<=0))
        self.assertEqual(goals[-1,2],3602)
        self.assertFalse(any(a in (10,11,20,38) for a,_ in b.events))
        for key in ('current_cap','acceleration','velocity','watchdog','p_gain','i_gain','d_gain'):
            np.testing.assert_array_equal(b.state[key],fixture()[key])

    def test_source_hash_and_mapping_and_raw_wrap_rejected(self):
        with patch.object(p,'SOURCE_SHA','wrong'):
            with self.assertRaises(ValueError):p.source_position()
        with patch.object(p,'SIM_TO_MOTOR_IDS',list(range(1,17))):
            with self.assertRaises(ValueError):p.source_position()
        b=ContinuationFake();b.state['position'][2]-=4096
        report,exc=self.execute(b)
        self.assertIsNotNone(exc);self.assertEqual(b.events,[])

    def test_source_must_be_readonly_off_and_healthy(self):
        original=json.loads(p.SOURCE.read_bytes())
        for fault in ('writes','torque','watchdog','voltage','mapping','camera'):
            record=json.loads(json.dumps(original))
            if fault=='writes':record['write_events']=[{'address':116}]
            elif fault in ('torque','watchdog'):record['preflight_state'][fault][0]=1
            elif fault=='voltage':record['preflight_state']['voltage'][0]=35
            elif fault=='mapping':record['joint_mapping']['motor_ids_in_sim_order']=list(range(1,17))
            else:record['camera_recording']['status']='failed'
            content=json.dumps(record).encode()
            with tempfile.TemporaryDirectory() as folder:
                file=Path(folder)/'source.json';file.write_bytes(content)
                with patch.object(p,'SOURCE',file),patch.object(p,'SOURCE_SHA',hashlib.sha256(content).hexdigest()):
                    with self.assertRaises(ValueError):p.source_position()

    def test_neighborhood_holds_roots_if_above_clearance(self):
        a=p.source_position()
        for i in range(16):
            for sign in (-1,1):
                q=a.copy();q[i]+=32*sign
                rows=p.plan(q);goals=np.array([v for _,v in rows])
                self.assertLessEqual(abs(np.diff(np.vstack([q,goals]),axis=0)).max(),2)
                self.assertLessEqual(max(abs(goals[-1,:4]-q[:4])),384)
                self.assertTrue(np.all(np.diff(np.vstack([q,goals])[:,p.ROOTS],axis=0)>=0))
                q[i]+=sign
                with self.assertRaises(ValueError):p.plan(q)

    def test_off_drift_and_config_and_write_failure(self):
        for fault in ('prewrite_drift','bad_profile','write_failure','enable_wrap','goal_changed','current','voltage'):
            with self.subTest(fault=fault):
                b=ContinuationFake(fault);report,exc=self.execute(b)
                self.assertIsNotNone(exc)
                if fault=='prewrite_drift':self.assertEqual(b.events,[])
                else:
                    self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
                self.assertLessEqual(sum(a==64 and np.any(v) for a,v in b.events),1)
                if fault=='write_failure':self.assertEqual(b.goal_attempts,3)

    def test_late_cancel_and_stationary_thumb_off(self):
        b=ContinuationFake()
        def cancel():
            if b.goal_attempts>=250:raise KeyboardInterrupt('user stop')
        report,exc=self.execute(b,cancel)
        self.assertIsInstance(exc,KeyboardInterrupt)
        self.assertEqual(b.goal_attempts,250)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        b=ContinuationFake()
        original=b.snapshot
        def stalled():
            s=original()
            if b.goal_attempts>=123:s['position'][2]=p.source_position()[2]
            return s
        b.snapshot=stalled
        report,exc=self.execute(b)
        self.assertIn('no progress',str(exc))
        self.assertLess(b.goal_attempts,362)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])

    def test_bound_offline_path_hash_and_failure_reject_before_devices(self):
        runner.validate_offline_review(p)
        record=json.loads(p.REVIEW_PATH.read_bytes())
        for kind in ('hash','penetration','step'):
            bad=json.loads(json.dumps(record))
            if kind=='hash':bad['plan_sha256']='wrong'
            elif kind=='penetration':bad['static_screens'][0]['max_self_overlap_mm']=0.1
            else:bad['static_screens'][0]['max_goal_step_ticks']=3
            with tempfile.TemporaryDirectory() as folder:
                path=Path(folder)/'screen.json';path.write_text(json.dumps(bad),encoding='utf-8')
                with patch.object(p,'REVIEW_PATH',path):
                    with self.assertRaises(ValueError):runner.validate_offline_review(p)

    def test_capture_off_endpoint_saves_off_pose_without_writes(self):
        b=ContinuationFake();report={'final_on_state':{'position':(b.state['position']+20).tolist()}}
        runner.record_off_endpoint(b,report,sleep=lambda _:None)
        self.assertTrue(report['final_off_reference_valid'])
        self.assertEqual(report['on_to_off_position_change_ticks'],[-20]*16)
        self.assertEqual(len(report['final_off_snapshots']),2)
        self.assertEqual(b.events,[])

    def test_unstable_off_endpoint_is_not_a_valid_reference(self):
        b=ContinuationFake();original=b.snapshot;calls=0
        def drifting():
            nonlocal calls
            calls+=1;s=original()
            if calls==2:s['position'][3]+=3
            return s
        b.snapshot=drifting;report={}
        runner.record_off_endpoint(b,report,sleep=lambda _:None)
        self.assertFalse(report['final_off_reference_valid'])
        self.assertEqual(report['final_off_endpoint_drift_ticks'][3],3)
        self.assertEqual(b.events,[])

    def test_failed_or_unhealthy_off_capture_sends_no_writes(self):
        for fault in ('read','torque','voltage'):
            b=ContinuationFake()
            if fault=='read':
                def failed():raise RuntimeError('read failed')
                b.snapshot=failed
            elif fault=='torque':b.state['torque'][0]=1
            else:b.state['voltage'][0]=35
            report={}
            runner.record_off_endpoint(b,report,sleep=lambda _:None)
            self.assertFalse(report['final_off_reference_valid'])
            self.assertIn('off_reference_capture_error',report)
            self.assertEqual(b.events,[])


if __name__=='__main__':unittest.main()
