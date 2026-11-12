"""Placement protocol, key buffering and callback failure tests; fake devices."""
import contextlib
import io
import json
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import unittest
import numpy as np
import sequence_flip_checked as checked
import leap_thumb_prepare_current_empty as current
import leap_thumb_prefix_empty as runner
import prepared_placement_plan as p
from prepared_placement_capture import TerminalKeys,capture
from test_thumb_current_start import Bus as OriginalBus
from test_leap_thumb_prefix_empty import FakeBus
from test_empty_trial_camera import fake as fake_camera

FIXTURE=p.ROOT/'right_hand_preparation_20261022/placement_fixture'

class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,n):self.t+=n

def source_state():
    return {k:np.array(v) for k,v in json.loads((FIXTURE/'request.json').read_bytes())['off_state'].items()}

class Bus(OriginalBus):
    def __init__(self):super().__init__();self.state=source_state()

class Tests(unittest.TestCase):
    def test_space_survives_poll_and_requires_release_for_next_prompt(self):
        clock=Clock();queue=[]
        keys=TerminalKeys(reader=lambda:queue.pop(0) if queue else None,now=clock.now)
        queue.append(' ');keys.poll();self.assertFalse(keys.consume_space())
        keys.arm();clock.t+=.4;keys.poll()
        queue.append(' ');keys.poll();keys.poll();self.assertTrue(keys.consume_space())
        keys.arm()
        for _ in range(10):
            clock.t+=.1;queue.append(' ');keys.poll()
            self.assertFalse(keys.consume_space())
        clock.t+=.4;keys.poll();queue.append(' ');keys.poll()
        self.assertTrue(keys.consume_space())

    def test_q_escape_ctrlc_always_cancel_even_unarmed(self):
        for key in ('q','Q','\x1b','\x03'):
            queue=[key];keys=TerminalKeys(reader=lambda:queue.pop(0) if queue else None)
            with self.assertRaises(KeyboardInterrupt):keys.poll()

    def test_space_survives_voltage_confirmation_reads(self):
        clock=Clock();queue=[];keys=TerminalKeys(reader=lambda:queue.pop(0) if queue else None,now=clock.now)
        keys.arm();clock.t=.4;keys.poll();queue.append(' ')
        raw=p.FULL_TARGET.copy();base=source_state()
        base.update(position=raw.copy(),goal=raw.copy(),torque=np.ones(16,dtype=int),
            watchdog=np.full(16,25),current_cap=checked.CAP.copy(),
            acceleration=np.zeros(16,dtype=int),velocity=np.zeros(16,dtype=int),
            **{k:v.copy() for k,v in checked.GAINS.items()})
        sequence=[]
        for voltage in (36,38,38):
            s={k:v.copy() for k,v in base.items()};s['voltage'][0]=voltage;sequence.append(s)
        bus=SimpleNamespace(snapshot=lambda:sequence.pop(0),cancel_callback=keys.poll)
        report={'enabled_at':time.monotonic()}
        with patch.object(checked,'check_cancel',side_effect=AssertionError('Must use the bus callback')):
            with contextlib.redirect_stdout(io.StringIO()):checked.sample(bus,raw,report,'place_cube')
        self.assertEqual(report['voltage_events'][-1]['status'],'recovered')
        self.assertTrue(keys.consume_space())

    def run_capture(self,*,key_mode='normal',stale=False):
        clock=Clock();report=dict(enabled_at=0.);events=[];sent=set();first={}
        raw=p.FULL_TARGET.copy();state=source_state()
        state.update(position=raw.copy(),goal=raw.copy(),torque=np.ones(16,dtype=int),
            watchdog=np.full(16,25),current_cap=checked.CAP.copy(),
            acceleration=np.zeros(16,dtype=int),velocity=np.zeros(16,dtype=int),
            **{k:v.copy() for k,v in checked.GAINS.items()})
        def reader():
            phase=report.get('phase');first.setdefault(phase,clock.now())
            if phase not in ('place_cube','remove_cube') or phase in sent:return None
            if clock.now()-first[phase]<.7:return None
            if key_mode=='timeout':return None
            sent.add(phase)
            return 'q' if key_mode=='q' else ' '
        keys=TerminalKeys(reader=reader,now=clock.now)
        def cancel():keys.poll()
        bus=SimpleNamespace(snapshot=lambda:{k:v.copy() for k,v in state.items()},cancel_callback=cancel)
        bus.write=lambda *args:events.append(args)
        photos=[]
        def photo(name,after):
            if stale or clock.now()<after:return None
            photos.append(name);return dict(image=name,registration_verified=False)
        camera=SimpleNamespace(capture_named=photo)
        error=None
        with contextlib.redirect_stdout(io.StringIO()):
            try:capture(bus,raw,report,camera,keys,cancel=cancel,
                        sleep=lambda n:clock.sleep(1 if key_mode=='timeout' else n),now=clock.now)
            except BaseException as exc:error=exc
        return report,error,events,photos

    def test_capture_hold_goals_photo_remove_and_no_goal_writes(self):
        report,error,events,photos=self.run_capture()
        self.assertIsNone(error);self.assertTrue(report['cube_removal_confirmed'])
        self.assertEqual(photos,['prepared_empty.jpg','cube_placed.jpg']);self.assertEqual(events,[])
        self.assertFalse(report['registration_verified'])
        self.assertTrue(all(row['goal']==p.FULL_TARGET.tolist() for row in report['samples']))
        self.assertGreater(len(report['samples']),20)

    def test_capture_cancel_timeout_and_stale_camera(self):
        for key_mode,stale,expected in (('q',False,KeyboardInterrupt),('timeout',False,RuntimeError),('normal',True,RuntimeError)):
            report,error,events,photos=self.run_capture(key_mode=key_mode,stale=stale)
            self.assertIsInstance(error,expected)
            self.assertFalse(report.get('cube_removal_confirmed',False));self.assertEqual(events,[])

    def test_named_photo_needs_fresh_after_press_and_valid_filename(self):
        with tempfile.TemporaryDirectory() as folder:
            camera_dir=Path(folder)/'camera';camera_dir.mkdir();camera=fake_camera(camera_dir)
            stamp=time.monotonic();frame=np.arange(27,dtype=np.uint8).reshape(3,3,3)
            for delta in (-.2,-.1,0):camera.signal.observe(stamp+delta,frame)
            camera.latest=(stamp,frame)
            self.assertIsNone(camera.capture_named('fresh.jpg',after=stamp+1))
            result=camera.capture_named('fresh.jpg',after=stamp-.1)
            self.assertEqual((Path(folder)/'fresh.jpg').read_bytes(),bytes([1,2,3]))
            self.assertFalse(result['registration_verified'])
            camera.latest=(stamp-1,np.zeros((3,3,3),dtype=np.uint8))
            self.assertIsNone(camera.capture_named('old.jpg'))
            with self.assertRaises(ValueError):camera.capture_named('../escape.jpg')

    def run_motion_callback(self,callback):
        b=Bus();report={}
        with tempfile.TemporaryDirectory() as folder:
            def screen(request,output,**kw):
                plan=p.PlacementPlan(request);proof=json.loads((FIXTURE/'screen.json').read_bytes())
                proof.update(source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(plan.source_position()))
                Path(output).write_text(json.dumps(proof),encoding='utf-8')
            with patch.object(runner.checked.Bus,'write',FakeBus.transport),contextlib.redirect_stdout(io.StringIO()):
                try:
                    current.prepare_and_run(b,folder,report,cancel=lambda:None,sleep=lambda _:None,
                        screener=screen,plan_type=p.PlacementPlan,prepared_callback=callback)
                except BaseException as exc:return b,report,exc
        return b,report,None

    def test_callback_runs_on_after_recovery_then_off_restores(self):
        called=[]
        def callback(b,command,report):
            self.assertTrue(np.all(b.state['torque']==1));self.assertEqual(len(b.pending),0)
            np.testing.assert_array_equal(command,p.FULL_TARGET)
            called.append(1);report['cube_removal_confirmed']=True
        b,report,error=self.run_motion_callback(callback)
        self.assertIsNone(error);self.assertEqual(called,[1])
        self.assertEqual(report['status'],p.PlacementPlan.STATUS)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])

    def test_callback_fault_or_q_still_turns_off_and_restores(self):
        for error in (RuntimeError('stale placement camera'),KeyboardInterrupt('Q')):
            def callback(*args):raise error
            b,report,observed=self.run_motion_callback(callback)
            self.assertIs(observed,error)
            self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
            self.assertNotEqual(report.get('status'),p.PlacementPlan.STATUS)

    def test_prepared_admission_accepts_relaxed_source_not_arbitrary_hand(self):
        state=source_state();np.testing.assert_array_equal(p.healthy_off(state),state['position'])
        for key,change in (('position',500),('torque',1),('voltage',-20)):
            bad={k:v.copy() for k,v in state.items()};bad[key][0]+=change
            with self.assertRaises(ValueError):p.healthy_off(bad)
        with patch.object(p,'SOURCE_SHA','changed'):
            with self.assertRaises(ValueError):p.admission(state['position'])

if __name__=='__main__':unittest.main()
