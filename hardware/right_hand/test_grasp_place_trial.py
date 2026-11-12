"""Loaded checkpoint ordering, timing and faults, with fake devices only."""
import contextlib
import copy
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
import grasp_place_plan as p
import leap_thumb_prefix_empty as runner
import leap_thumb_prepare_current_empty as current
from grasp_place_capture import GraspCapture
from prepared_placement_capture import TerminalKeys
from held_joint_guard import HeldJointGuard
from test_thumb_current_start import Bus as BaseBus
from test_leap_thumb_prefix_empty import FakeBus
from test_hand_preparation_timing import Clock

FIXTURE=p.ROOT/'prepared_cube_review_20261022/grasp_trial_v6_fixture'

class Bus(BaseBus):
    def __init__(self,clock):
        super().__init__();self.clock=clock
        self.state={k:np.array(v) for k,v in json.loads((FIXTURE/'request.json').read_bytes())['off_state'].items()}
    def snapshot(self):self.clock.t+=.031;return super().snapshot()

def transport(self,addr,size,values,verify=True):
    self.clock.t+=.001
    return FakeBus.transport(self,addr,size,values,verify)

class Tests(unittest.TestCase):
    def execute(self,place,finish=lambda b,c,r:r.update(cube_removal_confirmed=True),fault=None,release=None):
        clock=Clock();b=Bus(clock);report={}
        b.readback_injections=0;b.injection_goal_attempt=None
        original_snapshot=b.snapshot
        def snapshot():
            s=original_snapshot()
            if (fault in ('transient_goal','persistent_goal') and np.any(s['torque'])
                    and report.get('phase')==p.LOADED_PHASE
                    and b.goal_attempts>550 and not np.array_equal(b.previous_goal_command,b.last_goal_command)
                    and (fault=='persistent_goal' or b.readback_injections==0)):
                b.readback_injections+=1
                if b.injection_goal_attempt is None:b.injection_goal_attempt=b.goal_attempts
                s['goal']=b.previous_goal_command.copy()
            return s
        b.snapshot=snapshot
        def screen(request,output,**kw):
            plan=p.GraspPlacePlan(request);proof=json.loads((FIXTURE/'screen.json').read_bytes())
            proof.update(source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(plan.source_position()))
            Path(output).write_text(json.dumps(proof),encoding='utf-8')
        with tempfile.TemporaryDirectory() as folder,contextlib.redirect_stdout(io.StringIO()):
            def callback(bus,command,r,guard):return place(bus,command,r,guard,clock)
            def release_callback(bus,command,r,guard):
                if release is None:r['contact_release_operator_confirmed']=True
                else:release(bus,command,r,guard,clock)
            def write(self,addr,size,values,verify=True):
                if fault=='write' and report.get('cube_placement_confirmed') and addr==116:
                    self.goal_attempts+=1;raise RuntimeError('loaded write failed')
                result=transport(self,addr,size,values,verify)
                if fault=='observed_id12_limit' and addr==116:
                    self.state['position'][11]=min(self.state['position'][11],2460)
                return result
            with patch.object(runner.checked.Bus,'write',write), \
                    patch.object(runner.checked.time,'monotonic',clock.now), \
                    patch.object(runner.checked.time,'sleep',clock.sleep):
                try:
                    current.prepare_and_run(b,folder,report,plan_type=p.GraspPlacePlan,screener=screen,
                        cancel=clock.cancel,sleep=clock.sleep,now=clock.now,
                        phase_callback=callback,prepared_callback=finish,release_callback=release_callback)
                except BaseException as exc:return b,report,exc,clock
        return b,report,None,clock

    def test_transient_old_read_confirms_without_consuming_extra_goals(self):
        def place(b,c,r,g,clock):r['cube_placement_confirmed']=True
        normal,_,err,_=self.execute(place);self.assertIsNone(err)
        b,r,err,_=self.execute(place,fault='transient_goal');self.assertIsNone(err)
        self.assertEqual(b.readback_injections,1)
        self.assertEqual(r['goal_readback_events'][0]['status'],'recovered')
        self.assertEqual(r['goal_readback_events'][0]['confirmation_reads'],2)
        self.assertEqual(b.goal_attempts,normal.goal_attempts)
        self.assertEqual(len(b.pending),0)
        np.testing.assert_array_equal([v for a,v in b.events if a==116],
                                      [v for a,v in normal.events if a==116])
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_persistent_old_goal_stops_with_queue_unchanged_and_off_restored(self):
        def place(b,c,r,g,clock):r['cube_placement_confirmed']=True
        b,r,err,_=self.execute(place,fault='persistent_goal')
        self.assertIn('twice within',str(err))
        self.assertEqual(b.goal_attempts,b.injection_goal_attempt)
        self.assertEqual(b.readback_injections,4)
        self.assertGreater(len(b.pending),0)
        self.assertEqual(r['goal_readback_events'][0]['goal_writes'],0)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_wait_keeps_exact_goal_queue_and_does_not_use_motion_deadline(self):
        observed=[]
        def place(b,c,r,g,clock):
            np.testing.assert_array_equal(c,p.endpoints()[0])
            self.assertEqual(len(b.pending),322)
            before=len(b.events);clock.t+=70
            self.assertEqual(len(b.events),before);r['cube_placement_confirmed']=True
            observed.append(before)
        b,r,error,clock=self.execute(place)
        self.assertIsNone(error);self.assertEqual(len(observed),1)
        self.assertEqual(r['status'],p.GraspPlacePlan.STATUS)
        self.assertEqual(r['placement_pause_seconds'],70)
        self.assertLess(clock.now()-r['enabled_at']-r['placement_pause_seconds'],55)
        self.assertTrue(r['settings_restored'] and r['all_torque_off_verified'])
        goals=np.array([v for a,v in b.events if a==116])
        self.assertLessEqual(abs(np.diff(goals,axis=0)).max(),2)
        np.testing.assert_array_equal(goals[-1],p.endpoints()[1])
        self.assertEqual(sum(a==64 and np.any(v) for a,v in b.events),1)

    def test_no_confirmation_q_or_checkpoint_timeout_cannot_advance_loaded_goal(self):
        counts=[]
        for mode in ('unconfirmed','q','timeout'):
            def place(b,c,r,g,clock):
                counts.append(b.goal_attempts)
                if mode=='q':raise KeyboardInterrupt('Q')
                if mode=='timeout':clock.t+=101;r['cube_placement_confirmed']=True
            b,r,error,_=self.execute(place)
            self.assertIsNotNone(error);self.assertEqual(b.goal_attempts,counts[-1])
            self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
            self.assertNotEqual(r.get('status'),p.GraspPlacePlan.STATUS)

    def test_first_loaded_write_failure_has_no_retry_and_off_restores(self):
        before=[]
        def place(b,c,r,g,clock):before.append(b.goal_attempts);r['cube_placement_confirmed']=True
        b,r,error,_=self.execute(place,fault='write')
        self.assertIn('loaded write failed',str(error));self.assertEqual(b.goal_attempts,before[0]+1)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])

    def test_callback_is_required_before_configuration(self):
        clock=Clock();b=Bus(clock)
        with tempfile.TemporaryDirectory() as folder:
            from thumb_live_start_gate import write_request
            plan=p.GraspPlacePlan(write_request(folder,b.state,p.GraspPlacePlan.PLAN_VERSION,p.healthy_off))
            b.plan_module=plan;b.planner=plan.validate_start
            with self.assertRaisesRegex(RuntimeError,'both PLACE and REMOVE'):
                runner.run(b,b.state,{},cancel=lambda:None)
            self.assertEqual(b.events,[])

    def test_protocol_fresh_photos_three_second_clearance_and_remove_without_writes(self):
        clock=Clock();b=Bus(clock);raw=p.endpoints()[0];state=b.state
        import sequence_flip_checked as checked
        state.update(position=raw.copy(),goal=raw.copy(),torque=np.ones(16,dtype=int),
            watchdog=np.full(16,25),current_cap=checked.CAP.copy(),acceleration=np.zeros(16,dtype=int),
            velocity=np.zeros(16,dtype=int),**{k:v.copy() for k,v in checked.GAINS.items()})
        report={'enabled_at':clock.now()};sent=set();first={};photos=[]
        def reader():
            phase=report.get('phase');first.setdefault(phase,clock.now())
            if phase in ('place_cube','remove_cube') and phase not in sent and clock.now()-first[phase]>.7:
                sent.add(phase);return ' '
        keys=TerminalKeys(reader=reader,now=clock.now)
        camera=SimpleNamespace(capture_named=lambda name,after:None if clock.now()<after else photos.append(name) or {'image':name})
        protocol=GraspCapture(camera,keys,cancel=keys.poll,sleep=clock.sleep,now=clock.now)
        guard=HeldJointGuard(raw,raw)
        with contextlib.redirect_stdout(io.StringIO()):
            protocol.place(b,raw,report,guard)
            self.assertTrue(report['cube_present'] and report['cube_placement_confirmed'])
            self.assertGreaterEqual(clock.now()-first['loaded_countdown'],3)
            protocol.finish(b,raw,report)
        self.assertEqual(photos,['open_before_cube.jpg','cube_before_approach.jpg','cube_after_approach.jpg'])
        self.assertTrue(report['cube_removal_confirmed']);self.assertFalse(report['cube_present'])
        self.assertFalse(report['physical_grasp_verified'] or report['physical_flip_verified'])
        self.assertEqual(b.events,[])

    def test_saved_sag_is_admitted_but_wrap_or_source_change_is_not(self):
        r=p.completed_source();p.admission(r['final_off_snapshots'][-1]['position'])
        bad=np.array(r['final_off_snapshots'][-1]['position']);bad[2]-=4096
        with self.assertRaises(ValueError):p.admission(bad)
        with patch.object(p,'SOURCE_SHA','changed'):
            with self.assertRaises(ValueError):p.admission(r['final_on_state']['position'])

    def test_late_off_id15_is_screened_from_actual_state_not_archived_pose_lock(self):
        state=Bus(Clock()).state
        self.assertGreater(state['position'][14],2724)
        np.testing.assert_array_equal(p.healthy_off(state),state['position'])
        closed=p.endpoints()[1]
        # Admission permits bounded new starts only. Raw wrap, excess travel
        # and unhealthy power still stop before any configuration or goal.
        for i in range(16):
            raw=closed.copy();raw[i]=max(closed[i],p.endpoints()[0][i])+p.MAX_EMPTY_RECOVERY_TICKS+1
            with self.assertRaises(ValueError):p.admission(raw)
        bad={k:v.copy() for k,v in state.items()};bad['voltage'][0]=35
        with self.assertRaises(ValueError):p.healthy_off(bad)
        rows=p.GraspPlacePlan(FIXTURE/'request.json').plan(state['position'])
        self.assertLessEqual(len(rows)*.05+1,50)

    def test_opening_does_not_restore_closed_id12_before_placement(self):
        state=Bus(Clock()).state;plan=p.GraspPlacePlan(FIXTURE/'request.json')
        rows=plan.plan(state['position']);empty=[(phase,q) for phase,q in rows if phase!=p.LOADED_PHASE]
        self.assertLessEqual(max(q[11] for _,q in empty),p.endpoints()[0][11])
        self.assertFalse(any(np.array_equal(q,p.endpoints()[1]) for _,q in empty))
        first=np.array([q for phase,q in rows if phase=='stage_0_empty_recover'])
        frozen=[i for i in range(16) if i not in (3,14)]
        np.testing.assert_array_equal(first[:,frozen],np.tile(state['position'][frozen],(len(first),1)))
        self.assertTrue(np.all(np.diff(first[:,14])<=0))
        thumb=np.array([q for phase,q in rows if phase=='stage_1_thumb_clearance'])
        np.testing.assert_array_equal(thumb[:,4:],np.tile(first[-1,4:],(len(thumb),1)))

    def test_measured_id12_plateau_reaches_place_without_lifting_root_into_closed_grasp(self):
        reached=[]
        def place(b,c,r,g,clock):
            reached.append(1)
            self.assertEqual(c[11],2472)
            self.assertEqual(b.state['position'][11],2460)
            raise KeyboardInterrupt('Stop at PLACE; no cube simulation or loaded goals')
        b,r,error,_=self.execute(place,fault='observed_id12_limit')
        self.assertIsInstance(error,KeyboardInterrupt);self.assertEqual(reached,[1])
        self.assertEqual(len(b.pending),322)
        self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
        self.assertLessEqual(max(v[11] for a,v in b.events if a==116),2472)

    def test_a_stop_in_the_open_pose_is_admitted_for_new_actual_start_screen(self):
        p.admission(p.endpoints()[0])

    def test_native_reentry_proof_rejects_excess_increase_unfinished_or_wrong_phase(self):
        from thumb_live_start_gate import validate_proof
        plan=p.GraspPlacePlan(FIXTURE/'request.json')
        source=json.loads((FIXTURE/'screen.json').read_bytes())
        self.assertGreater(source['static_screens'][0]['initial_native_range_excess_rad'],0)
        validate_proof(plan,FIXTURE/'screen.json')
        with tempfile.TemporaryDirectory() as folder:
            for fault in ('excess','increase','unfinished','wrong_phase','missing','nan'):
                proof=copy.deepcopy(source);row=proof['static_screens'][0]
                if fault=='excess':row['initial_native_range_excess_rad']=.061
                elif fault=='increase':row['native_range_violation_rad']=row['initial_native_range_excess_rad']+.01
                elif fault=='unfinished':row['native_reentry_complete']=False
                elif fault=='wrong_phase':row['native_reentry_phase']=p.LOADED_PHASE
                elif fault=='missing':row.pop('native_reentry_monotonic')
                else:row['unapproved_native_range_violation_rad']=float('nan')
                path=Path(folder)/'proof.json';path.write_text(json.dumps(proof),encoding='utf-8')
                with self.assertRaises(RuntimeError):validate_proof(plan,path)

    def test_protocol_cannot_continue_through_held_drift_or_stale_photo(self):
        import sequence_flip_checked as checked
        for fault in ('drift','camera','q'):
            clock=Clock();b=Bus(clock);raw=p.endpoints()[0]
            b.state.update(position=raw.copy(),goal=raw.copy(),torque=np.ones(16,dtype=int),
                watchdog=np.full(16,25),current_cap=checked.CAP.copy(),acceleration=np.zeros(16,dtype=int),
                velocity=np.zeros(16,dtype=int),**{k:v.copy() for k,v in checked.GAINS.items()})
            guard=HeldJointGuard(raw,raw)
            if fault=='drift':b.state['position'][0]+=21
            keys=TerminalKeys(reader=lambda:'q' if fault=='q' else None,now=clock.now)
            protocol=GraspCapture(SimpleNamespace(capture_named=lambda *a,**kw:None),keys,
                cancel=keys.poll,sleep=clock.sleep,now=clock.now)
            report={'enabled_at':clock.now()}
            with contextlib.redirect_stdout(io.StringIO()),self.assertRaises((RuntimeError,KeyboardInterrupt)):
                protocol.place(b,raw,report,guard)
            self.assertFalse(report.get('cube_placement_confirmed',False));self.assertEqual(b.events,[])

    def test_existing_ring_contact_exception_cannot_allow_new_deeper_or_unfinished_contact(self):
        from thumb_live_start_gate import validate_proof
        plan=p.GraspPlacePlan(FIXTURE/'request.json')
        source=json.loads((FIXTURE/'screen.json').read_bytes())
        self.assertGreater(source['static_screens'][0]['max_self_overlap_mm'],0)
        with tempfile.TemporaryDirectory() as folder:
            for fault in ('new','deep','increase','late','missing','nan'):
                proof=copy.deepcopy(source);row=proof['static_screens'][0]
                if fault=='new':row['initial_self_contacts_mm']['thumb|palm']=.2
                elif fault=='deep':row['contact_clear_pip_travel_ticks']=p.GraspPlacePlan.MAX_CONTACT_RELEASE_TRAVEL_TICKS+1
                elif fault=='increase':row['self_escape_max_step_increase_mm']=.01
                elif fault=='late':row['self_escape_complete_at']=41
                elif fault=='missing':row.pop('self_escape_complete')
                else:row['self_escape_max_unapproved_overlap_mm']=float('nan')
                path=Path(folder)/'proof.json';path.write_text(json.dumps(proof),encoding='utf-8')
                with self.assertRaises(RuntimeError):validate_proof(plan,path)

    def test_release_q_or_missing_confirmation_prevents_thumb_and_opening_goals(self):
        for mode in ('q','unconfirmed'):
            reached=[]
            def release(b,c,r,g,clock):
                reached.append(b.goal_attempts)
                if mode=='q':raise KeyboardInterrupt('Q at release')
            def place(*args):raise AssertionError('PLACE must not be reached')
            b,r,error,_=self.execute(place,release=release)
            self.assertIsNotNone(error);self.assertEqual(b.goal_attempts,reached[0])
            self.assertTrue(r['all_torque_off_verified'] and r['settings_restored'])
            self.assertFalse(r.get('cube_placement_confirmed',False))

    def test_release_protocol_waits_for_observation_without_goal_writes(self):
        import sequence_flip_checked as checked
        clock=Clock();b=Bus(clock);raw=b.state['position'].copy()
        plan=p.GraspPlacePlan(FIXTURE/'request.json')
        command=[q for phase,q in plan.plan(raw) if phase=='stage_0_empty_recover'][-1]
        b.state.update(position=command.copy(),goal=command.copy(),torque=np.ones(16,dtype=int),
            watchdog=np.full(16,25),current_cap=checked.CAP.copy(),acceleration=np.zeros(16,dtype=int),
            velocity=np.zeros(16,dtype=int),**{k:v.copy() for k,v in checked.GAINS.items()})
        report={'enabled_at':clock.now(),'contact_release_threshold_ticks':2700};first={};sent=[];photos=[]
        def reader():
            phase=report.get('phase');first.setdefault(phase,clock.now())
            if phase=='release_contact_confirmation' and not sent and clock.now()-first[phase]>.8:
                sent.append(1);return ' '
        keys=TerminalKeys(reader=reader,now=clock.now)
        camera=SimpleNamespace(capture_named=lambda name,after:None if clock.now()<after else photos.append(name) or {'image':name})
        protocol=GraspCapture(camera,keys,cancel=keys.poll,sleep=clock.sleep,now=clock.now)
        with contextlib.redirect_stdout(io.StringIO()):protocol.release(b,command,report,HeldJointGuard(command,command))
        self.assertTrue(report['contact_release_operator_confirmed'])
        self.assertEqual(photos,['ring_released.jpg']);self.assertEqual(b.events,[])
        self.assertFalse(report.get('cube_placement_confirmed',False))

    def test_short_release_certificate_is_complete_consistent_and_bounded(self):
        from thumb_live_start_gate import validate_proof
        plan=p.GraspPlacePlan(FIXTURE/'request.json');source=json.loads((FIXTURE/'screen.json').read_bytes())
        self.assertGreater(source['dynamic_sensitivity'][0]['max_self_overlap_mm'],2)
        with tempfile.TemporaryDirectory() as folder:
            for fault in ('travel','threshold','missing','nan'):
                proof=copy.deepcopy(source);row=proof['static_screens'][0]
                if fault=='travel':
                    excessive=p.GraspPlacePlan.MAX_CONTACT_RELEASE_TRAVEL_TICKS+1
                    row['contact_clear_pip_travel_ticks']=excessive
                    row['contact_clear_pip_goal_ticks']=proof['start_ticks'][14]-excessive
                elif fault=='threshold':row['contact_clear_pip_goal_ticks']+=1
                elif fault=='missing':row.pop('contact_clear_pip_goal_ticks')
                else:row['contact_clear_pip_travel_ticks']=float('nan')
                path=Path(folder)/'proof.json';path.write_text(json.dumps(proof),encoding='utf-8')
                with self.assertRaises(RuntimeError):validate_proof(plan,path)

if __name__=='__main__':unittest.main()
