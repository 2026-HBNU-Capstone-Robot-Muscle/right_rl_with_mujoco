"""Device-free proof mutation and runtime failure injection for book preparation.

The accepted fixture, when available, is produced by the native screener.
Tests never install fabricated acceptance reports for real hardware use.
"""
import contextlib
import copy
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import book_empty_preparation_plan as p
import book_empty_preparation_gate as gate
import leap_book_prepare_empty as driver
import leap_thumb_prepare_current_empty as current
import leap_thumb_prefix_empty as runner
from initial_contact_escape import ContactEscape
from test_leap_thumb_prefix_empty import FakeBus

FIXTURE=p.ROOT/'right_hand_rotation_20261024/book20_current_review/structural_scope_fix_fixture'
PROOF=FIXTURE/'screen.json'


class Bus(FakeBus,driver.BookBus):
    def __init__(self,fault=None):
        super().__init__(fault)
        raw=np.array(json.loads((FIXTURE/'request.json').read_bytes())['off_state']['position'])
        self.state['position']=raw.copy();self.state['goal']=raw.copy()
    def preflight(self):return self.snapshot()


class Tests(unittest.TestCase):
    def plan(self):return p.BookPreparationPlan(FIXTURE/'request.json')

    def test_roots_separate_before_other_joints_and_native_targets(self):
        plan=self.plan();raw=plan.source_position();rows=plan.plan(raw)
        previous=raw;other=np.array([i for i in range(16) if i not in p.ROOTS]);switched=False
        for phase,q in rows:
            self.assertLessEqual(int(max(abs(q-previous))),2)
            if phase==p.LIFT:
                self.assertFalse(switched)
                np.testing.assert_array_equal(q[other],raw[other])
                self.assertTrue(np.all(q[p.ROOTS]>=previous[p.ROOTS]))
            else:
                switched=True
                self.assertTrue(np.all(previous[p.ROOTS]>=2472))
            previous=q
        np.testing.assert_array_equal(rows[-1][1],p.endpoints()[0])
        self.assertLessEqual((len(rows)+20)*.06+2,55)
        self.assertEqual(len(rows),845)

    def test_changed_native_pose_is_screened_instead_of_photo_locked(self):
        current=self.plan().source_position()
        self.assertGreater(int(max(abs(current-p.reference_ticks()))),64)
        np.testing.assert_array_equal(p.admission(current),current)
        b=Bus()
        with patch.object(runner.checked.Bus,'write',FakeBus.transport):
            with self.assertRaises(RuntimeError):b.write(64,1,np.ones(16))
        self.assertEqual(b.events,[])  # numerical admission never grants writes

    def test_raw_wrap_and_native_limit_violations_still_rejected(self):
        current=self.plan().source_position();limits=json.loads(p.LIMITS.read_bytes())
        for index,delta in ((0,-4096),(2,4096)):
            q=current.copy();q[index]+=delta
            with self.assertRaises(ValueError):p.admission(q)
        q=current.copy();q[0]=limits['lower_ticks'][0]-1
        with self.assertRaises(ValueError):p.admission(q)

    def test_initial_contact_is_not_a_new_or_deeper_contact_exemption(self):
        for pairs,at in (({'old':1.1},1),({'new':.01},1),({'old':.01},11)):
            with self.subTest(pairs=pairs,at=at):
                e=ContactEscape({'old':1.},['old'],4.,10,monotonic=True)
                e.observe(pairs,at);self.assertGreater(e.unapproved,0)
        e=ContactEscape({'old':1.},['old'],4.,10,monotonic=True)
        e.observe({'old':.5},1);e.observe({},2);e.observe({'old':.01},3)
        self.assertGreater(e.unapproved,0)  # static recontact is rejected

    def test_all_201_neighborhoods_and_9_dynamics_are_independently_validated(self):
        r=gate.validate_proof(self.plan(),PROOF)
        self.assertEqual(len(r['static_screens']),201)
        self.assertEqual(len(r['dynamic_sensitivity']),9)
        self.assertFalse(r['free_space_certificate'])

    def test_tampering_cannot_approve_bus(self):
        native=json.loads(PROOF.read_bytes());plan=self.plan()
        faults=('code','mapping','source','path','geometry_case','duplicate_offset',
                'deeper','new_pair','late','self','tracking','nan','range','initial_depth')
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'bad.json'
            for fault in faults:
                r=copy.deepcopy(native)
                if fault=='code':r['code_hashes']['book_support_model_20261024.py']='wrong'
                elif fault=='mapping':r['mapping']=list(range(1,17))
                elif fault=='source':r['source_sha256']='wrong'
                elif fault=='path':r['plan_sha256']='wrong'
                elif fault=='geometry_case':r['dynamic_sensitivity'][-1]['book_front_mm']=0.
                elif fault=='duplicate_offset':r['static_screens'][1]['offset_ticks']=[0]*16
                elif fault=='deeper':r['static_screens'][0]['support_escape_max_unapproved_overlap_mm']=.01
                elif fault=='new_pair':r['static_screens'][0]['initial_support_contacts_mm']['new|support_book']=.01
                elif fault=='late':r['dynamic_sensitivity'][0]['support_escape_complete_at']=100.
                elif fault=='self':r['static_screens'][0]['max_self_overlap_mm']=.01
                elif fault=='tracking':r['dynamic_sensitivity'][0]['max_tracking_rad']=.13
                elif fault=='nan':r['dynamic_sensitivity'][0]['max_tracking_rad']=float('nan')
                elif fault=='range':r['static_screens'][0]['native_range_violation_rad']=.01
                else:
                    pair=next(iter(r['static_screens'][0]['initial_support_contacts_mm']))
                    r['static_screens'][0]['initial_support_contacts_mm'][pair]=4.01
                path.write_text(json.dumps(r),encoding='utf-8');b=Bus()
                with self.subTest(fault=fault),self.assertRaises(RuntimeError):b.approve(plan,path)
                self.assertFalse(getattr(b,'screen_approved',False));self.assertEqual(b.events,[])

    def test_no_writes_before_separate_book_proof(self):
        b=Bus()
        with patch.object(runner.checked.Bus,'write',FakeBus.transport):
            for addr,size,v in ((84,2,np.full(16,600)),(64,1,np.ones(16)),(116,4,b.state['position'])):
                with self.assertRaises(RuntimeError):b.write(addr,size,v)
        self.assertEqual(b.events,[])

    def execute(self,b,cancel=lambda:None):
        plan=self.plan();b.approve(plan,PROOF);initial={k:v.copy() for k,v in b.state.items()};report={}
        with patch.object(runner.checked.Bus,'write',FakeBus.transport),contextlib.redirect_stdout(io.StringIO()):
            try:runner.run(b,initial,report,cancel=cancel,sleep=lambda _:None)
            except BaseException as exc:return report,exc
        return report,None

    def test_complete_once_and_restore_settings(self):
        b=Bus();original={k:v.copy() for k,v in b.state.items()};r,exc=self.execute(b)
        self.assertIsNone(exc);self.assertEqual(r['status'],p.BookPreparationPlan.STATUS)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
        self.assertEqual(sum(a==64 and np.any(v) for a,v in b.events),1)
        for key in ('current_cap','acceleration','velocity','watchdog','p_gain','i_gain','d_gain'):
            np.testing.assert_array_equal(b.state[key],original[key])

    def test_fault_or_q_stops_and_restores_without_retry(self):
        for fault in ('write_failure','read_failure','voltage','current','tracking','goal_changed'):
            b=Bus(fault);r,exc=self.execute(b)
            with self.subTest(fault=fault):
                self.assertIsNotNone(exc);self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
                self.assertLessEqual(b.goal_attempts,3)
        b=Bus()
        def cancel():
            if b.goal_attempts>=300:raise KeyboardInterrupt('Q')
        r,exc=self.execute(b,cancel)
        self.assertIsInstance(exc,KeyboardInterrupt);self.assertEqual(b.goal_attempts,300)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_default_opens_no_camera_or_port(self):
        with (patch('sys.argv',['leap_book_prepare_empty.py']),
              patch.object(driver.BookCamera,'__init__',side_effect=AssertionError('camera opened')),
              contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(current.main(p.BookPreparationPlan,bus_type=driver.BookBus,screener=gate.run_screen),0)


if __name__=='__main__':unittest.main()
