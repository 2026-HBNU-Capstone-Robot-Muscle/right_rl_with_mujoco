"""Device-free checkpoint/fault tests; proof fixtures are real native replays."""
import contextlib
import copy
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import book_grasp_trial_plan as p
import book_grasp_trial_gate as gate
import leap_book_cube_grasp_trial as driver
import leap_thumb_prefix_empty as runner
import leap_thumb_prepare_current_empty as current
from test_leap_thumb_prefix_empty import FakeBus
from test_hand_preparation_timing import Clock

FIXTURE=p.ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v7'
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
    def plan(self):return p.BookGraspPlan(FIXTURE/'request.json')

    def test_tip_releases_first_then_roots_then_place_and_same_reached_grasp(self):
        plan=self.plan();raw=plan.source_position();rows=plan.plan(raw)
        ring=[q for phase,q in rows if phase==p.RING_RELEASE]
        self.assertEqual(len(ring),40)
        self.assertEqual(ring[-1][14],raw[14]-80)
        tip=[q for phase,q in rows if phase==p.THUMB_RELEASE]
        self.assertEqual(len(tip),32)
        np.testing.assert_array_equal(np.array(tip)[:,1:],np.tile(ring[-1][1:],(32,1)))
        self.assertEqual(tip[-1][0],raw[0]-64)
        goals=np.vstack([raw]+[q for _,q in rows]);self.assertLessEqual(abs(np.diff(goals,axis=0)).max(),2)
        before=next(i for i,(phase,q) in enumerate(rows) if phase==p.LOADED_PHASE)
        np.testing.assert_array_equal(rows[before-1][1],p.endpoints()[0])
        np.testing.assert_array_equal(rows[-1][1],p.reached_grasp_goal())
        self.assertFalse(np.array_equal(rows[-1][1],p.endpoints()[1]))
        self.assertEqual(tuple(tuple(q.tolist()) for phase,q in rows[before:]),p.reached_prefix())
        np.testing.assert_array_equal(np.array([q[12:] for phase,q in rows[before:]]),np.tile(p.endpoints()[0][12:],(279,1)))
        self.assertLessEqual((len(rows)+40)*.06+2,75)

    def test_real_native_certificate_and_mutated_contact_scopes_fail_closed(self):
        native=gate.validate_proof(self.plan(),PROOF)
        self.assertEqual(len(native['static_screens']),201);self.assertEqual(len(native['dynamic_sensitivity']),9)
        for fault in ('code','extra','source','path','mapping','self_new','self_deeper','support_deeper','self_late','support_late','self_depth','support_depth','tracking','late_tracking','nan','range','model_policy','support_policy','other_joint_residual','missing_case','duplicate','cube','checkpoint','clear_threshold'):
            r=copy.deepcopy(native)
            if fault=='code':r['code_hashes']['sequence_flip_checked.py']='wrong'
            elif fault=='extra':r['extra_code_hashes']['grasp_place_capture.py']='wrong'
            elif fault=='source':r['source_sha256']='wrong'
            elif fault=='path':r['plan_sha256']='wrong'
            elif fault=='mapping':r['mapping']=list(range(1,17))
            elif fault=='self_new':r['static_screens'][0]['initial_self_contacts_mm']['new_pair']=.01
            elif fault=='self_deeper':r['static_screens'][0]['self_escape_max_unapproved_overlap_mm']=.01
            elif fault=='support_deeper':r['static_screens'][0]['support_escape_max_unapproved_overlap_mm']=.01
            elif fault=='self_late':r['static_screens'][0]['self_escape_complete_at']=73
            elif fault=='support_late':r['static_screens'][0]['support_escape_complete_at']=1000
            elif fault=='self_depth':r['static_screens'][0]['initial_self_contacts_mm'][p.SELF_PAIRS[0]]=4.01
            elif fault=='support_depth':r['static_screens'][0]['initial_support_contacts_mm'][next(iter(gate.ALLOWED_SUPPORT_PAIRS))]=5.01
            elif fault=='tracking':r['dynamic_sensitivity'][0]['max_tracking_rad']=.161
            elif fault=='late_tracking':r['dynamic_sensitivity'][0]['max_tracking_after_clearance_rad']=.121
            elif fault=='nan':r['dynamic_sensitivity'][0]['max_tracking_rad']=float('nan')
            elif fault=='range':r['static_screens'][0]['native_range_violation_rad']=.01
            elif fault=='model_policy':r['observed_model_policy']['contact_tolerance_mm']=.5
            elif fault=='support_policy':r['passive_support_reentry_policy']['new_or_deeper_contact_allowed']=True
            elif fault=='other_joint_residual':r['dynamic_sensitivity'][0]['unmodified_joint_range_violation_rad']=.0001
            elif fault=='missing_case':r['static_screens'].pop()
            elif fault=='duplicate':r['static_screens'][1]['offset_ticks']=[0]*16
            elif fault=='cube':r['cube_contact_verified']=True
            elif fault=='checkpoint':r['placement_checkpoint_required']=False
            else:r['static_screens'][0]['thumb_clear_goal_ticks']+=1
            with tempfile.TemporaryDirectory() as d:
                path=Path(d)/'mutated.json';path.write_text(json.dumps(r),encoding='utf-8');b=Bus(Clock())
                with self.subTest(fault=fault),self.assertRaises(RuntimeError):b.approve(self.plan(),path)
                self.assertFalse(getattr(b,'screen_approved',False));self.assertEqual(b.events,[])

    def execute(self,mode=None):
        clock=Clock();b=Bus(clock);plan=self.plan();b.approve(plan,PROOF);report={};b.report=report;checkpoint=[]
        initial={k:v.copy() for k,v in b.state.items()}
        def place(bus,command,r,guard):
            checkpoint.append(bus.goal_attempts)
            np.testing.assert_array_equal(command,p.endpoints()[0]);self.assertEqual(len(bus.pending),279)
            if mode=='Q':raise KeyboardInterrupt('Q')
            if mode=='unconfirmed':return
            if mode=='timeout':clock.t+=101
            else:clock.t+=70
            r['cube_placement_confirmed']=True
        def finish(bus,command,r):r['cube_removal_confirmed']=True
        def transport(bus,addr,size,values,verify=True):
            clock.t+=.001
            if mode=='loaded_write' and addr==116 and report.get('cube_placement_confirmed'):
                bus.goal_attempts+=1;raise RuntimeError('injected first loaded write failure')
            outcome=FakeBus.transport(bus,addr,size,values,verify)
            if addr==116 and mode=='tip_stall' and bus.goal_attempts>2:
                bus.state['position'][0]=initial['position'][0]
            if addr==116 and mode=='ring_stall' and bus.goal_attempts>2:
                bus.state['position'][14]=initial['position'][14]
            if addr==116 and mode=='other_thumb_drift' and bus.goal_attempts==2+bus._release_writes:
                bus.state['position'][1]=initial['position'][1]+5
            if addr==116 and mode=='observed_ID12_plateau' and report.get('cube_placement_confirmed'):
                bus.state['position'][11]=min(bus.state['position'][11],2543)
            return outcome
        def cancel():
            if mode=='camera' and report.get('cube_placement_confirmed'):raise RuntimeError('camera stale')
        with (patch.object(runner.checked.Bus,'write',transport),
            patch.object(runner.checked.time,'monotonic',clock.now),
            patch.object(runner.checked.time,'sleep',clock.sleep),contextlib.redirect_stdout(io.StringIO())):
            try:
                runner.run(b,initial,report,cancel=cancel,sleep=clock.sleep,now=clock.now,
                    prepared_callback=finish,phase_callback=place)
            except BaseException as exc:return b,report,exc,checkpoint,initial
        return b,report,None,checkpoint,initial

    def test_place_pause_preserves_queue_then_one_grasp_and_off_restore(self):
        b,r,error,checkpoint,initial=self.execute();self.assertIsNone(error)
        self.assertEqual(r['status'],p.BookGraspPlan.STATUS);self.assertEqual(r['placement_pause_seconds'],70)
        self.assertEqual(len(checkpoint),1);self.assertEqual(len(b.pending),0)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'] and r['cube_removal_confirmed'])
        self.assertEqual(sum(a==64 and np.any(v) for a,v in b.events),1)
        for key in ('current_cap','acceleration','velocity','watchdog','p_gain','i_gain','d_gain'):
            np.testing.assert_array_equal(b.state[key],initial[key])

    def test_no_loaded_goal_on_q_missing_confirmation_timeout_or_camera_fault(self):
        for mode in ('Q','unconfirmed','timeout','camera','loaded_write'):
            b,r,error,checkpoint,initial=self.execute(mode)
            with self.subTest(mode=mode):
                self.assertIsNotNone(error);self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
                self.assertEqual(b.goal_attempts,checkpoint[0]+(mode=='loaded_write'))
                self.assertNotEqual(r.get('status'),p.BookGraspPlan.STATUS)

    def test_missing_place_callback_cannot_configure(self):
        b=Bus(Clock());b.approve(self.plan(),PROOF)
        with self.assertRaisesRegex(RuntimeError,'both PLACE and REMOVE'):
            runner.run(b,b.state,{},cancel=lambda:None)
        self.assertEqual(b.events,[])

    def test_recorded_ID12_plateau_stops_closing_before_the_previous_fault(self):
        b,r,error,checkpoint,initial=self.execute('observed_ID12_plateau')
        self.assertIsNone(error)
        self.assertEqual(r['status'],p.BookGraspPlan.STATUS)
        self.assertEqual(r['final_on_state']['goal'][11],2594)
        self.assertEqual(r['final_on_state']['position'][11],2543)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
        self.assertEqual(len(b.pending),0)

    def test_unmeasured_tip_release_or_other_thumb_drift_cannot_advance_roots(self):
        for mode in ('tip_stall','ring_stall','other_thumb_drift'):
            b,r,error,checkpoint,initial=self.execute(mode)
            with self.subTest(mode=mode):
                if mode=='ring_stall':
                    # The existing no-progress guard may stop even before the
                    # measured-release checkpoint. Neither case may send a
                    # single thumb target after the ring release.
                    self.assertIn('no target advance',str(error))
                    self.assertLessEqual(b.goal_attempts,2+40)
                else:
                    self.assertIn('no root target advance',str(error))
                    self.assertEqual(b.goal_attempts,2+72)
                self.assertEqual(checkpoint,[])
                self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
                self.assertFalse(r.get('thumb_release_measured_verified',False))

    def test_default_opens_no_camera_and_no_unapproved_write_can_reach_transport(self):
        with (patch('sys.argv',['leap_book_cube_grasp_trial.py']),
            patch.object(driver.Camera,'__init__',side_effect=AssertionError('camera opened')),
            contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(current.main(p.BookGraspPlan,bus_type=driver.BookGraspBus,screener=gate.run_screen),0)
        b=Bus(Clock())
        with patch.object(runner.checked.Bus,'write',FakeBus.transport):
            for addr,size,v in ((84,2,np.full(16,600)),(64,1,np.ones(16)),(116,4,b.state['position'])):
                with self.assertRaises(RuntimeError):b.write(addr,size,v)
        self.assertEqual(b.events,[])


if __name__=='__main__':unittest.main()
