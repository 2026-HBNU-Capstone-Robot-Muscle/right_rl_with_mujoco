"""Measured-delay replay and finger failure tests; fake motors only."""
import contextlib
import copy
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_thumb_prefix_empty as runner
import leap_thumb_prepare_current_empty as current
import thumb_live_start_gate as gate
from hand_preparation_plan import PreparationPlan,FULL_TARGET
from thumb_live_start_plan import ROOT
from test_thumb_current_start import Bus as OriginalBus

FIXTURE=ROOT/'right_hand_preparation_20261022/finger_fixture'

def state():
    return {k:np.array(v) for k,v in json.loads((FIXTURE/'request.json').read_bytes())['off_state'].items()}

def proof_for(request):
    proof=json.loads((FIXTURE/'screen.json').read_bytes());plan=PreparationPlan(request)
    proof.update(source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(plan.source_position()))
    return proof

class Clock:
    def __init__(self):self.t=0.;self.sleeps=[]
    def now(self):return self.t
    def sleep(self,seconds):
        self.sleeps.append(seconds)
        # Ten milliseconds of scheduling jitter on every positive sleep.
        self.t+=seconds+(.010 if seconds>0 else 0)
    def cancel(self):self.t+=.012

class Bus(OriginalBus):
    def __init__(self,clock=None,fault=None):
        super().__init__(fault);self.state=state();self.clock=clock
    def snapshot(self):
        if self.clock:self.clock.t+=.031
        s=super().snapshot()
        if self.fault=='frozen_thumb' and self.goal_attempts>194:s['position'][0]+=21
        if self.fault=='finger_stall' and self.goal_attempts>194:s['position'][8]=state()['position'][8]
        return s

def transport(self,addr,size,values,verify=True):
    if self.clock:self.clock.t+=.001
    # Reuse the existing fault-injected transport without a real SDK.
    from test_leap_thumb_prefix_empty import FakeBus
    return FakeBus.transport(self,addr,size,values,verify)

class Tests(unittest.TestCase):
    def execute(self,b,clock=None,cancel=None):
        report={}
        with tempfile.TemporaryDirectory() as folder:
            def screen(request,output,**kw):Path(output).write_text(json.dumps(proof_for(request)),encoding='utf-8')
            with patch.object(runner.checked.Bus,'write',transport),contextlib.redirect_stdout(io.StringIO()):
                try:
                    current.prepare_and_run(b,folder,report,cancel=cancel or (clock.cancel if clock else lambda:None),
                        sleep=clock.sleep if clock else lambda _:None,now=clock.now if clock else __import__('time').monotonic,
                        screener=screen,plan_type=PreparationPlan)
                except BaseException as exc:return report,exc
        return report,None

    def test_recorded_read_camera_delays_finish_hold_below_same_55s(self):
        clock=Clock();b=Bus(clock);report,exc=self.execute(b,clock)
        self.assertIsNone(exc);self.assertEqual(report['status'],PreparationPlan.STATUS)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertEqual(report['cycle_pacing'],'processing_included')
        self.assertLess(clock.t-report['enabled_at'],55)
        self.assertGreater(clock.t-report['enabled_at'],35)
        self.assertTrue(any(0<s<.02 for s in clock.sleeps))
        goals=np.array([v for a,v in b.events if a==116])
        np.testing.assert_array_equal(goals[-1],FULL_TARGET)
        self.assertEqual(len(goals),706);self.assertLessEqual(abs(np.diff(goals,axis=0)).max(),2)
        np.testing.assert_array_equal(goals[194:,:4],np.tile(FULL_TARGET[:4],(512,1)))

    def test_actual_thumb_target_evidence_is_not_a_completed_1s_hold(self):
        result=json.loads((ROOT/'thumb_prepare_current_empty_20261022_002034_279500/result.json').read_bytes())
        self.assertEqual(result['failed_phase'],'stage_2_fixed_hold')
        hold=[s for s in result['samples'] if s['phase']=='stage_2_fixed_hold']
        self.assertEqual(len(hold),4)
        self.assertTrue(all(s['goal']==result['final_command'] for s in hold))
        self.assertLessEqual(max(abs(np.array(hold[-1]['actual'][:4])-np.array(FULL_TARGET[:4]))),11)
        self.assertLess(hold[-1]['t']-hold[0]['t'],1)
        self.assertTrue(result['all_torque_off_verified'] and result['settings_restored'])

    def test_held_thumb_drift_and_stationary_finger_stop(self):
        for fault,message in (('frozen_thumb','fixed finger'),('finger_stall','ID9 no progress')):
            b=Bus(fault=fault);report,exc=self.execute(b)
            self.assertIn(message,str(exc));self.assertLess(b.goal_attempts,706)
            self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])

    def test_cancel_during_new_finger_phase_off_and_restore(self):
        b=Bus()
        def cancel():
            if b.goal_attempts>=350:raise KeyboardInterrupt('Q')
        report,exc=self.execute(b,cancel=cancel)
        self.assertIsInstance(exc,KeyboardInterrupt);self.assertEqual(b.goal_attempts,350)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])

    def test_thumb_outside_completed_neighborhood_no_finger_goals(self):
        b=Bus();b.state['position'][0]-=100
        report,exc=self.execute(b)
        self.assertIn('Thumb differs',str(exc));self.assertEqual(b.events,[])

    def test_missing_extra_planner_hash_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            request=gate.write_request(folder,state(),PreparationPlan.PLAN_VERSION)
            proof=proof_for(request);proof['extra_planner_sha256']={}
            output=Path(folder)/'proof.json';output.write_text(json.dumps(proof),encoding='utf-8')
            with self.assertRaises(RuntimeError):gate.validate_proof(PreparationPlan(request),output)

    def test_overrunning_cycle_never_sends_catchup_goals(self):
        # A read still within the health deadline can exceed the 50ms cycle.
        # Scheduling sleeps no negative time; read-before-write remains intact.
        clock=Clock();b=Bus(clock);reads=0;original=b.snapshot
        def read():
            nonlocal reads
            reads+=1
            if b.goal_attempts>2:clock.t+=.025
            return original()
        b.snapshot=read
        report,exc=self.execute(b,clock)
        if exc is not None:self.assertIn('55s',str(exc))
        self.assertGreaterEqual(reads,b.goal_attempts-2)
        self.assertTrue(all(s>=0 for s in clock.sleeps))
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])

if __name__=='__main__':unittest.main()
