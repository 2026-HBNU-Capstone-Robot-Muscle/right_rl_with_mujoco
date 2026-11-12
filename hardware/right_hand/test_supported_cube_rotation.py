"""Loaded checkpoint, bounded path, delays and failure restoration; fake devices."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import supported_cube_rotation_plan as p
import leap_thumb_prefix_empty as runner
import leap_thumb_prepare_current_empty as current
from test_grasp_place_trial import Bus,transport
from test_hand_preparation_timing import Clock
from held_joint_guard import HeldJointGuard
from prepared_placement_capture import TerminalKeys
from supported_cube_rotation_capture import SupportedCubeRotationCapture

FIXTURE=p.OUT/'fixture_loaded_supported_latest_off'

class Tests(unittest.TestCase):
    def execute(self,fault=None):
        clock=Clock();bus=Bus(clock);report={};entered=[]
        bus.state={k:np.array(v) for k,v in json.loads((FIXTURE/'request.json').read_bytes())['off_state'].items()}
        original=bus.snapshot;bus.last_instruction=clock.now();delayed_history_reads=[]
        original_bytes=Path.read_bytes
        def disk_read(path):
            if fault=='slow_history' and np.any(bus.state['torque']):
                delayed_history_reads.append(str(path));clock.t+=.26
            return original_bytes(path)
        def watchdog():
            if (fault=='slow_history' and np.any(bus.state['torque']) and
                    np.any(bus.state['watchdog']) and clock.now()-bus.last_instruction>.5):
                bus.state['watchdog'][:]=255
        def snapshot():
            watchdog()
            if report.get('phase')==p.ROTATION_PHASE:
                entered.append(bus.goal_attempts)
                if fault=='read':raise RuntimeError('injected loaded rotation read failure')
            value=original();bus.last_instruction=clock.now();return value
        bus.snapshot=snapshot
        def write(self,addr,size,values,verify=True):
            watchdog()
            if addr==116 and report.get('phase')==p.ROTATION_PHASE and fault=='write':
                self.goal_attempts+=1;raise RuntimeError('injected loaded rotation write failure')
            value=transport(self,addr,size,values,verify);self.last_instruction=clock.now();return value
        def cancel():
            if report.get('phase')==p.ROTATION_PHASE:
                if fault=='q':raise KeyboardInterrupt('Q')
                if fault=='camera':raise RuntimeError('Camera stayed blank')
            clock.cancel()
        def now():return clock.now()+(76 if fault=='deadline' and report.get('phase')==p.ROTATION_PHASE else 0)
        def screen(request,output,**kw):
            plan=p.SupportedCubeRotationPlan(request);proof=json.loads((FIXTURE/'screen.json').read_bytes())
            proof.update(source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(plan.source_position()))
            Path(output).write_text(json.dumps(proof),encoding='utf-8')
        def place(b,c,r,g):
            r['place_goal_attempt']=b.goal_attempts
            if fault!='no_place':r['cube_placement_confirmed']=True
        def release(b,c,r,g):
            r['release_goal_attempt']=b.goal_attempts
            if fault!='no_release':r['contact_release_operator_confirmed']=True
        def finish(b,c,r):r.update(rotation_command_path_completed=True,cube_removal_confirmed=True,physical_rotation_verified=False)
        with tempfile.TemporaryDirectory() as folder,contextlib.redirect_stdout(io.StringIO()), \
            patch.object(Path,'read_bytes',disk_read), \
            patch.object(runner.checked.Bus,'write',write), \
            patch.object(runner.checked.time,'monotonic',clock.now),patch.object(runner.checked.time,'sleep',clock.sleep):
            try:
                current.prepare_and_run(bus,folder,report,plan_type=p.SupportedCubeRotationPlan,screener=screen,
                    cancel=cancel,sleep=clock.sleep,now=now,phase_callback=place,
                    prepared_callback=finish,release_callback=release)
            except BaseException as exc:return bus,report,exc,entered
        return bus,report,None,entered

    def test_torque_on_planning_has_no_file_reads_under_slow_storage(self):
        b,r,error,entered=self.execute('slow_history')
        self.assertIsNone(error);self.assertTrue(entered)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
        self.assertTrue(all(all(v==25 for v in s['watchdog']) for s in r['samples']))

    def test_cached_planning_and_enable_adoption_need_no_files(self):
        from leap_thumb_prefix_empty import Bus as Scheduled
        plan=p.SupportedCubeRotationPlan(FIXTURE/'request.json');raw=plan.source_position()
        b=object.__new__(Scheduled);b.plan_module=plan
        with patch.object(Path,'read_bytes',side_effect=AssertionError('File read while ON')), \
             patch.object(Path,'read_text',side_effect=AssertionError('File read while ON')):
            b.bind(raw);b.pending.pop(0);b.adopt(raw);rows=plan.plan(raw)
            self.assertEqual(len(b.pending),len(rows)+1)
            plan.validate_start(raw)

    def test_cached_path_matches_the_real_stopped_run_and_prior_approved_paths(self):
        stopped=p.ROOT/'cube_supported_rotation_trial_20261023_033108_297056'
        for folder,request,proof in ((stopped,'fresh_off_request.json','current_start_screen.json'),
            (FIXTURE,'request.json','screen.json'),
            (p.OUT/'fixture_loaded_supported_prior','request.json','screen.json')):
            plan=p.SupportedCubeRotationPlan(folder/request);saved=json.loads((folder/proof).read_bytes())
            self.assertEqual(plan.plan_digest(plan.source_position()),saved['plan_sha256'])

    def test_exact_loaded_path_after_placement_one_enable_and_off_restored(self):
        b,r,error,entered=self.execute();self.assertIsNone(error);self.assertTrue(entered)
        self.assertTrue(r['cube_placement_confirmed'] and r['cube_removal_confirmed'])
        self.assertEqual(r['status'],p.SupportedCubeRotationPlan.STATUS)
        goals=np.array([q for a,q in b.events if a==116])
        np.testing.assert_array_equal(goals[-256:],p.candidate())
        self.assertLessEqual(np.max(abs(np.diff(goals,axis=0))),2)
        self.assertEqual(sum(a==64 and np.any(q) for a,q in b.events),1)
        self.assertEqual(len(b.pending),0)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
        self.assertFalse(r['physical_rotation_verified'])
        self.assertLess(r['samples'][-1]['t'],75)

    def test_missing_release_or_placement_confirmation_blocks_targets(self):
        for fault,key in (('no_release','release_goal_attempt'),('no_place','place_goal_attempt')):
            b,r,error,entered=self.execute(fault)
            self.assertIsNotNone(error);self.assertFalse(entered);self.assertEqual(b.goal_attempts,r[key])
            self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_read_write_q_camera_faults_never_retry_or_claim_completion(self):
        for fault in ('read','write','q','camera'):
            with self.subTest(fault=fault):
                b,r,error,entered=self.execute(fault)
                self.assertIsNotNone(error);self.assertGreater(len(b.pending),0)
                if fault=='read':self.assertEqual(b.goal_attempts,entered[0])
                if fault=='write':self.assertEqual(b.goal_attempts,entered[0]+1)
                self.assertFalse(r.get('rotation_command_path_completed',False))
                self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_deadline_stops_before_rotation_and_restores(self):
        b,r,error,entered=self.execute('deadline');self.assertIn('75s',str(error))
        self.assertFalse(entered);self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_candidate_and_initial_range_reentry_remain_bounded(self):
        with patch.object(p,'CANDIDATE_SHA','changed'):
            with self.assertRaisesRegex(ValueError,'candidate changed'):p.candidate()
        state=json.loads((FIXTURE/'request.json').read_bytes())['off_state'];a=np.array(state['position'])
        p.admission(a);limits=json.loads(p.LIMITS.read_bytes())
        bad=a.copy();bad[3]=int(np.floor(limits['lower_ticks'][3]-.081/p.TICK))
        with self.assertRaisesRegex(ValueError,'native-range'):p.admission(bad)
        bad=a.copy();bad[4]=int(np.ceil(limits['upper_ticks'][4]+.061/p.TICK))
        with self.assertRaisesRegex(ValueError,'native-range'):p.admission(bad)
        self.assertEqual(p.SupportedYawEmptyPlan.MAX_START_RANGE_EXCESS_RAD,.06)

    def test_finish_photographs_and_requires_removal_without_writing_goals(self):
        import sequence_flip_checked as checked
        clock=Clock();bus=Bus(clock);raw=p.candidate()[-1]
        bus.state.update(position=raw.copy(),goal=raw.copy(),torque=np.ones(16,dtype=int),
            watchdog=np.full(16,25),current_cap=checked.CAP.copy(),acceleration=np.zeros(16,dtype=int),
            velocity=np.zeros(16,dtype=int),**{k:v.copy() for k,v in checked.GAINS.items()})
        report={'enabled_at':clock.now(),'cube_placement_confirmed':True};photos=[];first={};sent=[]
        def reader():
            phase=report.get('phase');first.setdefault(phase,clock.now())
            if phase=='remove_cube' and not sent and clock.now()-first[phase]>.7:sent.append(1);return ' '
        keys=TerminalKeys(reader=reader,now=clock.now)
        camera=SimpleNamespace(capture_named=lambda name,after:None if clock.now()<after else photos.append(name) or {'image':name})
        protocol=SupportedCubeRotationCapture(camera,keys,cancel=keys.poll,sleep=clock.sleep,now=clock.now)
        protocol.guard=HeldJointGuard(raw,raw)
        with contextlib.redirect_stdout(io.StringIO()):protocol.finish(bus,raw,report)
        self.assertEqual(photos,['cube_after_supported_rotation.jpg'])
        self.assertTrue(report['cube_removal_confirmed'] and report['rotation_command_path_completed'])
        self.assertFalse(report['physical_rotation_verified'] or report['physical_flip_verified'])
        self.assertEqual(bus.events,[])

if __name__=='__main__':unittest.main()
