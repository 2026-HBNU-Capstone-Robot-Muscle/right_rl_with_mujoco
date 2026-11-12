"""New approach-stage checks with fake motors; no hardware or camera access."""
import contextlib
import copy
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import grasp_approach_plan as p
import leap_thumb_prefix_empty as runner
import leap_thumb_prepare_current_empty as current
import thumb_live_start_gate as gate
from test_thumb_current_start import Bus as BaseBus
from test_leap_thumb_prefix_empty import FakeBus
from test_hand_preparation_timing import Clock

FIXTURE=p.ROOT/'prepared_cube_review_20261022/empty_approach_v2_fixture'


def state():
    return {k:np.array(v) for k,v in json.loads((FIXTURE/'request.json').read_bytes())['off_state'].items()}


def proof_for(request):
    doc=json.loads((FIXTURE/'screen.json').read_bytes());plan=p.GraspApproachPlan(request)
    doc.update(source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(plan.source_position()))
    return doc


class Bus(BaseBus):
    def __init__(self,clock=None):
        super().__init__();self.state=state();self.clock=clock
    def snapshot(self):
        if self.clock:self.clock.t+=.031
        return super().snapshot()


def transport(self,addr,size,values,verify=True):
    if self.clock:self.clock.t+=.001
    return FakeBus.transport(self,addr,size,values,verify)


class Tests(unittest.TestCase):
    def execute(self,b,*,cancel=None,write=transport):
        clock=b.clock;report={}
        with tempfile.TemporaryDirectory() as folder:
            def screen(request,output,**kw):Path(output).write_text(json.dumps(proof_for(request)),encoding='utf-8')
            with patch.object(runner.checked.Bus,'write',write),contextlib.redirect_stdout(io.StringIO()):
                try:
                    current.prepare_and_run(b,folder,report,plan_type=p.GraspApproachPlan,screener=screen,
                        cancel=cancel or (clock.cancel if clock else lambda:None),
                        sleep=clock.sleep if clock else lambda _:None,
                        now=clock.now if clock else __import__('time').monotonic)
                except BaseException as exc:return report,exc
        return report,None

    def test_complete_under_realistic_delay_budget_and_restore(self):
        clock=Clock();b=Bus(clock);original=copy.deepcopy(b.state)
        report,exc=self.execute(b)
        self.assertIsNone(exc);self.assertEqual(report['status'],p.GraspApproachPlan.STATUS)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertLess(clock.t-report['enabled_at'],55)
        self.assertFalse(p.GraspApproachPlan.CAPTURE_PLACEMENT)
        goals=np.array([v for a,v in b.events if a==116]);self.assertLessEqual(abs(np.diff(goals,axis=0)).max(),2)
        np.testing.assert_array_equal(goals[-1][:12],p.target()[:12])
        for key in ('current_cap','acceleration','velocity','watchdog','p_gain','i_gain','d_gain'):
            np.testing.assert_array_equal(b.state[key],original[key])
        self.assertEqual(sum(a==64 and np.any(v) for a,v in b.events),1)
        self.assertFalse(any(a in (10,11,20,38) for a,_ in b.events))

    def test_new_phase_cancel_off_without_resume(self):
        b=Bus()
        def cancel():
            if b.goal_attempts>=300:raise KeyboardInterrupt('Q')
        report,exc=self.execute(b,cancel=cancel)
        self.assertIsInstance(exc,KeyboardInterrupt);self.assertEqual(b.goal_attempts,300)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])

    def test_held_ring_drift_and_thumb_stall_stop_the_new_phase(self):
        for fault,expected in (('ring','fixed finger'),('thumb','ID2 no progress')):
            b=Bus();original=b.snapshot
            def read():
                s=original()
                # Inject after the new approach has begun, while the ring's
                # goal is held. During stage 2 it was still being commanded;
                # a prior tracking offset there is not fresh held-joint drift.
                if b.goal_attempts>300:
                    if fault=='ring':s['position'][12]+=21
                    else:s['position'][1]=state()['position'][1]
                return s
            b.snapshot=read;report,exc=self.execute(b)
            self.assertIn(expected,str(exc));self.assertLess(b.goal_attempts,700)
            self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])

    def test_goal_write_failure_in_approach_has_no_retry(self):
        b=Bus()
        def fail(self,addr,size,values,verify=True):
            if addr==116 and self.goal_attempts>=300:
                self.goal_attempts+=1;raise RuntimeError('approach write failure')
            return transport(self,addr,size,values,verify)
        report,exc=self.execute(b,write=fail)
        self.assertIn('approach write failure',str(exc));self.assertEqual(b.goal_attempts,301)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])

    def test_altered_target_or_wrong_raw_family_blocks_all_writes(self):
        b=Bus()
        with patch.object(p,'TARGET_SHA','invalid'):
            report,exc=self.execute(b)
            self.assertIn('target changed',str(exc));self.assertEqual(b.events,[])
        for i,delta in ((0,-4096),(7,400),(13,500)):
            b=Bus();b.state['position'][i]+=delta;report,exc=self.execute(b)
            self.assertIsNotNone(exc);self.assertEqual(b.events,[])

    def test_target_json_hash_required_in_proof(self):
        with tempfile.TemporaryDirectory() as folder:
            request=gate.write_request(folder,state(),p.GraspApproachPlan.PLAN_VERSION,p.healthy_off)
            proof=proof_for(request);proof['extra_planner_sha256'].pop('prepared_cube_review_20261022/empty_grasp_target.json')
            out=Path(folder)/'proof.json';out.write_text(json.dumps(proof),encoding='utf-8')
            with self.assertRaises(RuntimeError):gate.validate_proof(p.GraspApproachPlan(request),out)


if __name__=='__main__':unittest.main()
