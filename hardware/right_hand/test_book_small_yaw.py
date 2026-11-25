"""Frozen candidate, real proof and PLACE/fault/recording boundary tests."""
import contextlib
import copy
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import book_small_yaw_plan as p
import book_grasp_trial_gate as gate
import leap_book_small_yaw_trial as driver
import leap_thumb_prefix_empty as runner
import leap_thumb_prepare_current_empty as current
from test_leap_thumb_prefix_empty import FakeBus
from test_hand_preparation_timing import Clock
from book_small_yaw_capture import BookSmallYawCapture
from grasp_place_capture import GraspCapture

FIXTURE=p.ROOT/'right_hand_rotation_20261024/book20_current_review/small_yaw_fixture_v1'
PROOF=FIXTURE/'screen.json'


class Bus(FakeBus,driver.BookGraspBus):
    def __init__(self,clock):
        super().__init__();self.clock=clock;self.report={}
        self.state={k:np.array(v) for k,v in json.loads((FIXTURE/'request.json').read_bytes())['off_state'].items()}

    def preflight(self):return self.snapshot()

    def snapshot(self):
        self.clock.t+=.031
        state=super().snapshot()
        self.latest_release_state=state;self.latest_release_read_at=self.clock.now()
        return state


class Tests(unittest.TestCase):
    def plan(self):return p.BookSmallYawPlan(FIXTURE/'request.json')

    def test_reached_grasp_then_baseline_and_exact_frozen_candidate_within_budget(self):
        plan=self.plan();raw=plan.source_position();rows=plan.plan(raw)
        groups={phase:[q for ph,q in rows if ph==phase] for phase in (p.BASELINE,p.YAW,p.AFTER)}
        self.assertEqual([len(v) for v in groups.values()],[60,90,40])
        np.testing.assert_array_equal(groups[p.YAW],p.load_candidate()[1])
        np.testing.assert_array_equal(groups[p.BASELINE],np.tile(p.reached_grasp_goal(),(60,1)))
        self.assertLessEqual(abs(np.diff(np.vstack([raw]+[q for _,q in rows]),axis=0)).max(),2)
        self.assertLessEqual((len(rows)+40)*.06+2,90)
        self.assertEqual(plan.ON_TIMEOUT,90)
        self.assertEqual(p.BookGraspPlan.ON_TIMEOUT,75)
        np.testing.assert_array_equal(plan.frozen_for_phase(p.YAW),np.array(p.FROZEN_IDS)-1)
        np.testing.assert_array_equal(plan.progress_for_phase(p.YAW),p.MOVING)

    def test_changed_candidate_goals_provenance_or_direction_are_rejected(self):
        original,_=p.load_candidate()
        for fault in ('step','root','base','source','direction','offset','count','claim'):
            r=copy.deepcopy(original)
            if fault=='step':r['goal_ticks'][0][0]+=10
            elif fault=='root':r['goal_ticks'][-1][7]+=1
            elif fault=='base':r['base_goal_ticks'][0]+=1
            elif fault=='source':r['source_hashes'][p.SOURCES[0]]='changed'
            elif fault=='direction':r['desired_physical_yaw_degrees']=3
            elif fault=='offset':r['following_error_used_only_in_offline_simulation']=False
            elif fault=='count':r['rotation_samples']=71
            else:r['physical_cube_rotation_verified']=True
            with self.subTest(fault=fault),self.assertRaises(ValueError):p.validate_candidate(r)
        with patch.object(p,'CANDIDATE_SHA','wrong'),self.assertRaises(ValueError):p.load_candidate()

    def test_native_certificate_bound_to_rotation_candidate_and_current_code(self):
        native=gate.validate_proof(self.plan(),PROOF)
        self.assertTrue(native['rotation_included'])
        self.assertFalse(native['cube_contact_verified'])
        self.assertEqual((len(native['static_screens']),len(native['dynamic_sensitivity'])),(201,9))
        for fault in ('rotation','candidate','version','source'):
            r=copy.deepcopy(native)
            if fault=='rotation':r['rotation_included']=False
            elif fault=='candidate':r['extra_code_hashes'][p.CANDIDATE]='changed'
            elif fault=='version':r['plan_version']=p.BookGraspPlan.PLAN_VERSION
            else:r['source_sha256']='changed'
            with tempfile.TemporaryDirectory() as folder:
                path=Path(folder)/'proof.json';path.write_text(json.dumps(r),encoding='utf-8')
                b=Bus(Clock())
                with self.subTest(fault=fault),self.assertRaises(RuntimeError):b.approve(self.plan(),path)
                self.assertEqual(b.events,[])

    def execute(self,mode=None):
        clock=Clock();b=Bus(clock);plan=self.plan();b.approve(plan,PROOF);report={};b.report=report;checkpoint=[]
        initial={k:v.copy() for k,v in b.state.items()}
        def place(bus,command,r,guard):
            checkpoint.append(bus.goal_attempts)
            self.assertEqual(len(bus.pending),469)
            if mode=='Q':raise KeyboardInterrupt('Q')
            if mode=='unconfirmed':return
            clock.t+=70;r['cube_placement_confirmed']=True
        def finish(bus,command,r):
            np.testing.assert_array_equal(command,plan.yaw_goals[-1])
            r['cube_removal_confirmed']=True
        def transport(bus,addr,size,values,verify=True):
            clock.t+=.001
            if mode=='yaw_write' and addr==116 and report.get('phase')==p.YAW:
                bus.goal_attempts+=1;raise RuntimeError('injected yaw write failure')
            return FakeBus.transport(bus,addr,size,values,verify)
        def cancel():
            if mode=='camera' and report.get('phase')==p.YAW:raise RuntimeError('camera stale')
        with (patch.object(runner.checked.Bus,'write',transport),
            patch.object(runner.checked.time,'monotonic',clock.now),
            patch.object(runner.checked.time,'sleep',clock.sleep),contextlib.redirect_stdout(io.StringIO())):
            try:
                runner.run(b,initial,report,cancel=cancel,sleep=clock.sleep,now=clock.now,
                    prepared_callback=finish,phase_callback=place)
            except BaseException as exc:return b,report,exc,checkpoint,initial
        return b,report,None,checkpoint,initial

    def test_confirmed_place_preserves_one_queue_then_turns_off_and_restores(self):
        b,r,error,checkpoint,initial=self.execute();self.assertIsNone(error)
        self.assertEqual(r['status'],p.BookSmallYawPlan.STATUS)
        self.assertEqual(len(checkpoint),1);self.assertEqual(len(b.pending),0)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'] and r['cube_removal_confirmed'])
        self.assertTrue(r['ring_release_measured_verified'] and r['thumb_release_measured_verified'])
        self.assertEqual(sum(a==64 and np.any(v) for a,v in b.events),1)
        for key in ('current_cap','acceleration','velocity','watchdog','p_gain','i_gain','d_gain'):
            np.testing.assert_array_equal(b.state[key],initial[key])

    def test_no_place_confirmation_sends_no_loaded_or_yaw_target(self):
        for mode in ('Q','unconfirmed'):
            b,r,error,checkpoint,_=self.execute(mode)
            with self.subTest(mode=mode):
                self.assertIsNotNone(error);self.assertEqual(b.goal_attempts,checkpoint[0])
                self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_camera_or_transport_fault_during_yaw_stops_once_and_restores(self):
        for mode in ('camera','yaw_write'):
            b,r,error,checkpoint,_=self.execute(mode)
            with self.subTest(mode=mode):
                self.assertIsNotNone(error)
                self.assertNotEqual(r.get('status'),p.BookSmallYawPlan.STATUS)
                self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
                self.assertGreater(len(b.pending),0)
                self.assertEqual(sum(a==64 and np.any(v) for a,v in b.events),1)

    def test_observation_precedes_removal_and_never_certifies_physical_rotation(self):
        protocol=BookSmallYawCapture(None,None,cancel=lambda:None);protocol.guard=object()
        report={};command=p.reached_grasp_goal()
        with patch.object(GraspCapture,'place'):
            with contextlib.redirect_stdout(io.StringIO()):protocol.place(None,command,report,protocol.guard)
        report['cube_placement_confirmed']=True
        state={'position':command.copy()};events=[]
        def photo(*args):events.append('photo');return state,{'camera_read_monotonic':123.5}
        def remove(*args):
            events.append('remove')
            self.assertEqual(report['rotation_evidence_cutoff_camera_monotonic'],123.5)
            self.assertFalse(report['physical_rotation_verified'])
            return state
        with (patch.object(protocol,'_observer',return_value=object()),
            patch.object(protocol,'_photo',side_effect=photo),patch.object(protocol,'_space',side_effect=remove),
            contextlib.redirect_stdout(io.StringIO())):
            protocol.finish(None,command,report)
        self.assertEqual(events,['photo','remove'])
        self.assertTrue(report['cube_removal_confirmed'] and report['rotation_command_path_completed'])
        self.assertFalse(report['removal_motion_is_success_evidence'])

    def test_default_entry_opens_no_devices(self):
        with (patch('sys.argv',['leap_book_small_yaw_trial.py']),
            patch.object(driver.Camera,'__init__',side_effect=AssertionError('camera opened')),
            contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(current.main(p.BookSmallYawPlan,bus_type=driver.BookGraspBus,screener=driver.run_screen),0)


if __name__=='__main__':unittest.main()
