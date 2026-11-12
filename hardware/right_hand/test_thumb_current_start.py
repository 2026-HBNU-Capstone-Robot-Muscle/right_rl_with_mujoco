"""Current-start acquisition, proof gates, process cancellation and motion faults.

All motor behavior is simulated with a fake bus. The proof fixture comes from
an actual offline MuJoCo run, not a fake success report installed for hardware.
"""
import contextlib
import copy
import io
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_thumb_prefix_empty as runner
import leap_thumb_prepare_current_empty as current
import thumb_live_start_gate as gate
import thumb_live_start_plan as p
from test_leap_thumb_prefix_empty import FakeBus

FIXTURE=p.ROOT/'right_hand_preparation_20261021/fresh_start_fixture'


def state():
    return {k:np.array(v) for k,v in json.loads((FIXTURE/'request.json').read_bytes())['off_state'].items()}


def proof_for(request):
    proof=json.loads((FIXTURE/'screen.json').read_bytes())
    plan=p.CurrentPlan(request)
    proof.update(source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(plan.source_position()))
    return proof


class Bus(FakeBus,current.Bus):
    def __init__(self,fault=None):
        super().__init__(fault);self.state=state()
    def preflight(self):
        return self.snapshot()


class Tests(unittest.TestCase):
    def run_trial(self,b,*,cancel=lambda:None,screener=None):
        report={}
        with tempfile.TemporaryDirectory() as folder:
            def screen(request,output,**kw):
                Path(output).write_text(json.dumps(proof_for(request)),encoding='utf-8')
            with patch.object(runner.checked.Bus,'write',FakeBus.transport),contextlib.redirect_stdout(io.StringIO()):
                try:
                    current.prepare_and_run(b,folder,report,cancel=cancel,sleep=lambda _:None,
                                            screener=screener or screen)
                except BaseException as exc:return report,exc
        return report,None

    def test_complete_full_remaining_thumb_and_frozen_goals(self):
        b=Bus();original={k:v.copy() for k,v in b.state.items()};report,exc=self.run_trial(b)
        self.assertIsNone(exc)
        self.assertEqual(report['status'],p.CurrentPlan.STATUS)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        goals=np.array([v for a,v in b.events if a==116]);count=p.ROOT_STEPS+p.THUMB_STEPS+2
        self.assertEqual(len(goals),count)
        self.assertLessEqual(abs(np.diff(goals,axis=0)).max(),2)
        np.testing.assert_array_equal(goals[-1,:4],p.THUMB_TARGET)
        np.testing.assert_array_equal(goals[:,p.FROZEN],np.tile(original['position'][p.FROZEN],(count,1)))
        self.assertFalse(any(a in (10,11,20,38) for a,_ in b.events))
        self.assertEqual(sum(a==64 and np.any(v) for a,v in b.events),1)
        for key in ('current_cap','acceleration','velocity','watchdog','p_gain','i_gain','d_gain'):
            np.testing.assert_array_equal(b.state[key],original[key])

    def test_no_ram_writes_until_matched_proof(self):
        b=Bus()
        with patch.object(runner.checked.Bus,'write',FakeBus.transport):
            for addr,size,v in ((84,2,np.full(16,600)),(98,1,np.full(16,25)),
                                (64,1,np.ones(16)),(116,4,b.state['position'])):
                with self.assertRaises(RuntimeError):b.write(addr,size,v)
        self.assertEqual(b.events,[])

    def test_screen_failure_and_cancel_before_motion_no_writes(self):
        for error in (RuntimeError('checker failed'),KeyboardInterrupt('Q')):
            b=Bus()
            def fail(*args,**kw):raise error
            report,exc=self.run_trial(b,screener=fail)
            self.assertIs(exc,error);self.assertEqual(b.events,[])

    def test_simulation_time_drift_stops_before_any_write(self):
        b=Bus()
        def screen(request,output,**kw):
            Path(output).write_text(json.dumps(proof_for(request)),encoding='utf-8')
            b.state['position'][3]+=5
        report,exc=self.run_trial(b,screener=screen)
        self.assertIn('four-tick',str(exc));self.assertEqual(b.events,[])

    def test_bad_proof_cannot_approve_bus(self):
        with tempfile.TemporaryDirectory() as folder:
            request=gate.write_request(folder,state());plan=p.CurrentPlan(request)
            original=proof_for(request);out=Path(folder)/'proof.json'
            for fault in ('hash','code','native','accepted','step','self','nan','gain','cases','mapping'):
                proof=copy.deepcopy(original)
                if fault=='hash':proof['source_sha256']='wrong'
                elif fault=='code':proof['script_sha256']='wrong'
                elif fault=='native':proof['native_ranges_unchanged']=False
                elif fault=='accepted':proof['accepted']=False
                elif fault=='step':proof['static_screens'][0]['max_goal_step_ticks']=3
                elif fault=='self':proof['dynamic_sensitivity'][0]['max_self_overlap_mm']=.01
                elif fault=='nan':proof['dynamic_sensitivity'][0]['max_tracking_rad']=float('nan')
                elif fault=='gain':proof['dynamic_sensitivity'][0]['gain_scale']=1.1
                elif fault=='cases':proof['static_screens'].pop()
                else:proof['mapping']=list(range(1,17))
                out.write_text(json.dumps(proof),encoding='utf-8');b=Bus()
                with self.assertRaises(RuntimeError):b.approve(plan,out)
                self.assertFalse(getattr(b,'screen_approved',False));self.assertEqual(b.events,[])

    def test_unstable_off_and_health_reject_without_writes(self):
        for fault in ('moving','torque','voltage'):
            b=Bus();original=b.snapshot;calls=0
            def snapshot():
                nonlocal calls
                s=original();calls+=1
                if fault=='moving':s['position'][3]+=calls*3
                elif fault=='torque':s['torque'][0]=1
                else:s['voltage'][0]=35
                return s
            b.snapshot=snapshot;report={}
            with self.assertRaises((ValueError,RuntimeError)):
                gate.stable_off(b,report,cancel=lambda:None,sleep=lambda _:None,now=lambda:0.)
            self.assertEqual(b.events,[])

    def test_wrong_family_and_raw_wrap(self):
        for index,change in ((2,-4096),(4,40),(3,-800)):
            b=Bus();b.state['position'][index]+=change
            report,exc=self.run_trial(b)
            self.assertIsNotNone(exc);self.assertEqual(b.events,[])

    def test_runtime_faults_restore_settings_and_no_retry(self):
        for fault in ('write_failure','read_failure','tracking','current','voltage','goal_changed','enable_wrap'):
            b=Bus(fault);report,exc=self.run_trial(b)
            self.assertIsNotNone(exc)
            self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
            self.assertEqual(sum(a==64 and np.any(v) for a,v in b.events),1)
            if fault=='write_failure':self.assertEqual(b.goal_attempts,3)

    def test_cancel_during_full_remaining_motion(self):
        b=Bus()
        def cancel():
            if b.goal_attempts>=300:raise KeyboardInterrupt('Q')
        report,exc=self.run_trial(b,cancel=cancel)
        self.assertIsInstance(exc,KeyboardInterrupt);self.assertEqual(b.goal_attempts,300)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])

    def test_request_changed_after_screen_or_unscheduled_goal(self):
        with tempfile.TemporaryDirectory() as folder:
            request=gate.write_request(folder,state());plan=p.CurrentPlan(request)
            out=Path(folder)/'proof.json';out.write_text(json.dumps(proof_for(request)),encoding='utf-8')
            b=Bus();b.approve(plan,out);b.bind(state()['position'])
            with patch.object(runner.checked.Bus,'write',FakeBus.transport):
                wrong=state()['position'].copy();wrong[0]+=1
                with self.assertRaises(ValueError):b.write(116,4,wrong)
            request.write_text('{}',encoding='utf-8')
            with self.assertRaises(ValueError):plan.source_position()
            self.assertEqual(b.events,[])

    def test_process_cancel_timeout_and_nonzero_exit(self):
        class Process:
            def __init__(self,code=None):self.returncode=code;self.stopped=False
            def poll(self):return self.returncode
            def terminate(self):self.stopped=True;self.returncode=-15
            def wait(self,timeout):return self.returncode
            def kill(self):self.stopped=True;self.returncode=-9
        with tempfile.TemporaryDirectory() as folder:
            request=gate.write_request(folder,state());out=Path(folder)/'proof.json'
            process=Process()
            def cancel():raise KeyboardInterrupt('Q')
            with self.assertRaises(KeyboardInterrupt):
                gate.run_screen(request,out,cancel=cancel,sleep=lambda _:None,
                    popen=lambda *a,**kw:process)
            self.assertTrue(process.stopped)
            process=Process();clock=iter([0.,121.])
            with self.assertRaisesRegex(RuntimeError,'120s'):
                gate.run_screen(request,out,cancel=lambda:None,sleep=lambda _:None,
                    now=lambda:next(clock),popen=lambda *a,**kw:process)
            self.assertTrue(process.stopped)
            with self.assertRaisesRegex(RuntimeError,'failed'):
                gate.run_screen(request,out,cancel=lambda:None,popen=lambda *a,**kw:Process(2))
            out.write_text('{}')
            with self.assertRaisesRegex(RuntimeError,'stale'):
                gate.run_screen(request,out,cancel=lambda:None,popen=lambda *a,**kw:Process(0))

    def test_55_second_motion_deadline_off_restores(self):
        with tempfile.TemporaryDirectory() as folder:
            request=gate.write_request(folder,state());plan=p.CurrentPlan(request)
            out=Path(folder)/'proof.json';out.write_text(json.dumps(proof_for(request)),encoding='utf-8')
            b=Bus();b.approve(plan,out);report={};clock=iter([0.,56.])
            with patch.object(runner.checked.Bus,'write',FakeBus.transport):
                with self.assertRaisesRegex(RuntimeError,'55s'):
                    runner.run(b,{k:v.copy() for k,v in b.state.items()},report,
                               cancel=lambda:None,sleep=lambda _:None,now=lambda:next(clock))
            self.assertEqual(b.goal_attempts,2)
            self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])


if __name__=='__main__':unittest.main()
