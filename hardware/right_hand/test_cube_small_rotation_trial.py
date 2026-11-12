"""First-yaw ordering and failure behavior. Fake devices only."""
import contextlib
import hashlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import cube_small_rotation_plan as p
import grasp_place_plan as grasp
import leap_thumb_prefix_empty as runner
import leap_thumb_prepare_current_empty as current
from cube_small_rotation_capture import SmallRotationCapture
from held_joint_guard import HeldJointGuard
from prepared_placement_capture import TerminalKeys
from test_grasp_place_trial import Bus, transport
from test_hand_preparation_timing import Clock

FIXTURE=p.ROOT/'right_hand_rotation_20261022/fixture_completed_off'


class Tests(unittest.TestCase):
    def execute(self,fault=None):
        clock=Clock();bus=Bus(clock);report={};entered=[]
        bus.state={k:np.asarray(v) for k,v in json.loads((FIXTURE/'request.json').read_bytes())['off_state'].items()}
        original_snapshot=bus.snapshot
        def snapshot():
            if report.get('phase')==p.ROTATION_PHASE:
                entered.append(bus.goal_attempts)
                if fault=='read':raise RuntimeError('injected rotation read fault')
            return original_snapshot()
        bus.snapshot=snapshot
        def write(self,addr,size,values,verify=True):
            if addr==116 and report.get('phase')==p.ROTATION_PHASE and fault=='write':
                self.goal_attempts+=1
                raise RuntimeError('injected rotation write fault')
            return transport(self,addr,size,values,verify)
        def cancel():
            if report.get('phase')==p.ROTATION_PHASE and fault=='q':raise KeyboardInterrupt('Q during rotation')
            if report.get('phase')==p.ROTATION_PHASE and fault=='camera':
                from camera_frame_signal import CameraImageUnavailable
                raise CameraImageUnavailable('Camera image stayed black/blank for 0.5s')
            clock.cancel()
        def screen(request,output,**kw):
            plan=p.SmallRotationPlan(request)
            proof=json.loads((FIXTURE/'screen.json').read_bytes())
            proof.update(source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(plan.source_position()))
            Path(output).write_text(json.dumps(proof),encoding='utf-8')
        def place(b,c,r,g):
            r['place_goal_count']=b.goal_attempts
            if fault!='no_place':r['cube_placement_confirmed']=True
        def finish(b,c,r):
            r.update(rotation_command_path_completed=True,cube_removal_confirmed=True,
                     physical_rotation_verified=False)
        with tempfile.TemporaryDirectory() as folder,contextlib.redirect_stdout(io.StringIO()), \
             patch.object(runner.checked.Bus,'write',write), \
             patch.object(runner.checked.time,'monotonic',clock.now), \
             patch.object(runner.checked.time,'sleep',clock.sleep):
            try:
                current.prepare_and_run(bus,folder,report,plan_type=p.SmallRotationPlan,screener=screen,
                    cancel=cancel,sleep=clock.sleep,now=clock.now,phase_callback=place,
                    prepared_callback=finish,release_callback=lambda b,c,r,g:r.update(contact_release_operator_confirmed=True))
            except BaseException as exc:return bus,report,exc,entered
        return bus,report,None,entered

    def test_one_place_then_actual_rotation_goals_and_single_enable(self):
        bus,r,error,entered=self.execute()
        self.assertIsNone(error);self.assertTrue(entered)
        self.assertEqual(r['status'],p.SmallRotationPlan.STATUS)
        goals=np.asarray([v for a,v in bus.events if a==116])
        self.assertLessEqual(np.max(abs(np.diff(goals,axis=0))),2)
        np.testing.assert_array_equal(goals[-80:],p.candidate())
        self.assertTrue(np.any(goals[-1]!=grasp.endpoints()[1]))
        self.assertEqual(sum(a==64 and np.any(v) for a,v in bus.events),1)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
        self.assertFalse(r['physical_rotation_verified'])
        self.assertEqual(len(bus.pending),0)
        self.assertLess(r['samples'][-1]['t'],55)

    def test_absent_placement_prevents_both_loaded_approach_and_rotation(self):
        bus,r,error,entered=self.execute('no_place')
        self.assertIsNotNone(error);self.assertEqual(entered,[])
        self.assertEqual(bus.goal_attempts,r['place_goal_count'])
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_rotation_read_write_and_q_faults_stop_without_goal_retry(self):
        for mode in ('read','write','q','camera'):
            with self.subTest(mode=mode):
                bus,r,error,entered=self.execute(mode)
                self.assertIsNotNone(error)
                if mode=='write':self.assertEqual(bus.goal_attempts,entered[0]+1)
                elif mode=='read':self.assertEqual(bus.goal_attempts,entered[0])
                self.assertGreater(len(bus.pending),0)
                self.assertFalse(r.get('rotation_command_path_completed',False))
                self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
                self.assertEqual(sum(a==64 and np.any(v) for a,v in bus.events),1)

    def test_rotation_cannot_change_completed_approach_or_ring_goals(self):
        request=FIXTURE/'request.json';plan=p.SmallRotationPlan(request)
        raw=plan.source_position()
        # The original loaded/empty path stays byte-for-byte equivalent.
        original=object.__new__(grasp.GraspPlacePlan)
        original.__dict__.update(plan.__dict__)
        original.PLAN_VERSION=p.SmallRotationPlan.PLAN_VERSION
        before=grasp.GraspPlacePlan.plan(original,raw)
        rows=plan.plan(raw)
        self.assertEqual(len(rows),len(before)+80)
        for (phase,a),(old,b) in zip(rows,before):
            self.assertEqual(phase,old);np.testing.assert_array_equal(a,b)
        np.testing.assert_array_equal(p.candidate()[:,12:],np.tile(grasp.endpoints()[1][12:],(80,1)))

    def test_changed_candidate_is_rejected(self):
        with patch.object(p,'CANDIDATE_SHA','changed'):
            with self.assertRaisesRegex(ValueError,'candidate changed'):p.candidate()

    def test_after_photo_and_removal_do_not_claim_physical_success(self):
        import sequence_flip_checked as checked
        clock=Clock();bus=Bus(clock);raw=p.candidate()[-1]
        bus.state.update(position=raw.copy(),goal=raw.copy(),torque=np.ones(16,dtype=int),
            watchdog=np.full(16,25),current_cap=checked.CAP.copy(),acceleration=np.zeros(16,dtype=int),
            velocity=np.zeros(16,dtype=int),**{k:v.copy() for k,v in checked.GAINS.items()})
        report={'enabled_at':clock.now(),'cube_placement_confirmed':True};photos=[];first={};sent=[]
        def reader():
            phase=report.get('phase');first.setdefault(phase,clock.now())
            if phase=='remove_cube' and not sent and clock.now()-first[phase]>.7:
                sent.append(1);return ' '
        keys=TerminalKeys(reader=reader,now=clock.now)
        camera=SimpleNamespace(capture_named=lambda name,after:None if clock.now()<after else photos.append(name) or {'image':name})
        protocol=SmallRotationCapture(camera,keys,cancel=keys.poll,sleep=clock.sleep,now=clock.now)
        protocol.guard=HeldJointGuard(raw,raw)
        with contextlib.redirect_stdout(io.StringIO()):protocol.finish(bus,raw,report)
        self.assertEqual(photos,['cube_after_small_rotation.jpg'])
        self.assertTrue(report['cube_removal_confirmed'] and report['rotation_command_path_completed'])
        self.assertFalse(report['physical_rotation_verified'] or report['physical_flip_verified'])
        self.assertEqual(bus.events,[])


if __name__=='__main__':unittest.main()
