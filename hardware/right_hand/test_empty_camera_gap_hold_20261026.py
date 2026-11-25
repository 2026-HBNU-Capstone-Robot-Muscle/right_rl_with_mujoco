"""Health, cancellation and goal-queue invariants during EMPTY camera gaps."""
import contextlib,io,itertools,threading,unittest
from unittest.mock import Mock,patch
import numpy as np
from camera_frame_signal import FrameSignal,CameraImageUnavailable
from empty_camera_gap_hold_20261026 import CameraGapHold,MAX_SINGLE_HOLD_S,MAX_TOTAL_HOLD_S
from recovering_empty_camera_20261026 import RecoveringEmptyCamera
from quarter_finish_empty_camera_20261025 import QuarterFinishEmptyCamera
import camera_gap_motion_runner_20261026 as runner
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan,FIXTURE,RETURN_PHASE
from test_quarter_support_photo_20261025 import FakeBus

class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,dt):self.t+=dt
class GapChecks(unittest.TestCase):
    def wait(self,guard,clock,reader,**kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            return guard.wait_if_needed(reader,phase='test',command=np.full(16,2200),cancel=kwargs.pop('cancel',lambda:None),now=clock.now,sleep=clock.sleep,**kwargs)
    def test_healthy_camera_does_not_delay_or_call_hold_reader(self):
        c=Clock();reader=Mock();g=CameraGapHold(lambda:dict(ready=True,reason='fresh_scene'),{})
        self.assertEqual(self.wait(g,c,reader),0);reader.assert_not_called()
    def test_transient_gap_recovers_with_health_reads_and_no_target_api(self):
        c=Clock();r={};g=CameraGapHold(lambda:dict(ready=c.t>=.8,reason='black',stamp=c.t),r);calls=[]
        def read():calls.append(c.t);c.t+=.04
        elapsed=self.wait(g,c,read)
        self.assertGreaterEqual(elapsed,.8);self.assertGreater(len(calls),10)
        self.assertTrue(r['camera_gap_holds'][0]['resumed']);self.assertEqual(r['camera_gap_holds'][0]['goal_writes_during_hold'],0)
    def test_persistent_blank_is_still_bounded(self):
        c=Clock();r={};g=CameraGapHold(lambda:dict(ready=False,reason='black'),r)
        with self.assertRaisesRegex(CameraImageUnavailable,'did not recover'):
            self.wait(g,c,lambda:None)
        self.assertLessEqual(c.t,MAX_SINGLE_HOLD_S+.03);self.assertFalse(r['camera_gap_holds'][0]['resumed'])
    def test_motor_fault_during_gap_propagates_immediately(self):
        c=Clock();r={};g=CameraGapHold(lambda:dict(ready=False,reason='stale'),r)
        with self.assertRaisesRegex(RuntimeError,'motor fault'):
            self.wait(g,c,Mock(side_effect=RuntimeError('motor fault')))
        self.assertEqual(c.t,0);self.assertTrue(r['camera_gap_holds'][0]['stopped_without_target_advance'])
    def test_q_still_interrupts_the_hold(self):
        c=Clock();r={};g=CameraGapHold(lambda:dict(ready=False,reason='black'),r);reader=Mock()
        with self.assertRaises(KeyboardInterrupt):self.wait(g,c,reader,cancel=Mock(side_effect=KeyboardInterrupt()))
        reader.assert_not_called()
    def test_motion_deadline_can_shorten_camera_wait(self):
        c=Clock();g=CameraGapHold(lambda:dict(ready=False,reason='stale'),{})
        with self.assertRaises(CameraImageUnavailable):self.wait(g,c,lambda:None,motion_deadline=.2)
        self.assertLessEqual(c.t,.23)
    def test_total_hold_budget_does_not_reset_on_recovery(self):
        c=Clock();r={};recover=[4.];g=CameraGapHold(lambda:dict(ready=c.t>=recover[0],reason='black',stamp=c.t),r)
        self.wait(g,c,lambda:None);recover[0]=c.t+4.;self.wait(g,c,lambda:None)
        recover[0]=c.t+4.
        with self.assertRaises(CameraImageUnavailable):self.wait(g,c,lambda:None)
        self.assertLessEqual(g.total,MAX_TOTAL_HOLD_S+.05)

class SignalChecks(unittest.TestCase):
    def camera(self):
        c=RecoveringEmptyCamera.__new__(RecoveringEmptyCamera);c.lock=threading.Lock();c.error=None;c.report={};c.signal=FrameSignal()
        frame=np.arange(12*12*3,dtype=np.uint8).reshape(12,12,3)
        for t in (99.8,99.9,100.):c.signal.observe(t,frame)
        c.latest=(100.,frame);c.camera=Mock();return c,frame
    def test_old_frame_cannot_resume_even_with_good_streak(self):
        c,_=self.camera()
        with patch('recovering_empty_camera_20261026.time.monotonic',return_value=100.2):self.assertTrue(c.motion_view()['ready'])
        with patch('recovering_empty_camera_20261026.time.monotonic',return_value=100.31):self.assertFalse(c.motion_view()['ready'])
    def test_blank_then_less_than_three_new_good_frames_cannot_resume(self):
        c,frame=self.camera();c.signal.observe(101.,np.zeros_like(frame))
        c.signal.observe(101.1,frame);c.signal.observe(101.2,frame);c.latest=(101.2,frame)
        with patch('recovering_empty_camera_20261026.time.monotonic',return_value=101.2):self.assertFalse(c.motion_view()['ready'])
        c.signal.observe(101.3,frame);c.latest=(101.3,frame)
        with patch('recovering_empty_camera_20261026.time.monotonic',return_value=101.3):self.assertTrue(c.motion_view()['ready'])
    def test_soft_image_check_requires_bound_motion_guard(self):
        c,_=self.camera();c.motion_hold_bound=False;c.report.update(enabled_at=1.,phase='motion')
        with patch.object(QuarterFinishEmptyCamera,'check',side_effect=CameraImageUnavailable('strict')):
            with self.assertRaisesRegex(CameraImageUnavailable,'strict'):c.check()
        c.bind_motion_hold()
        with patch('recovering_empty_camera_20261026.time.monotonic',return_value=105.):c.check()
        self.assertIsNone(c.error);c.camera.check_preview.assert_called_once()

class RunnerChecks(unittest.TestCase):
    def run_case(self,fault=None):
        p=ThumbRootEmptyPlan(FIXTURE/'request.json');r={};b=FakeBus(p,r);initial=b.snapshot();clock=itertools.count();track=dict(triggered=False,holding=False,reads=0,goal_count=None)
        def view():
            if not track['triggered'] and r.get('phase')==RETURN_PHASE:
                track.update(triggered=True,holding=True,goal_count=sum(a==116 for a,q in b.writes))
            return dict(ready=not track['holding'] or track['reads']>=4,reason='black',stamp=next(clock)*.002)
        b.camera_motion_guard=CameraGapHold(view,r)
        def sample(bus,command,report,phase,snapshot_check):
            if phase.startswith('camera_gap_hold:'):
                self.assertEqual(sum(a==116 for a,q in b.writes),track['goal_count'])
                if fault=='health':raise RuntimeError('Synthetic motor fault during camera gap')
                track['reads']+=1
            s=bus.snapshot();snapshot_check(s);return s,np.zeros(16)
        def cancel():
            if fault=='cancel' and r.get('phase','').startswith('camera_gap_hold:'):raise KeyboardInterrupt('Synthetic Q')
        with patch.object(runner,'check_off'),patch.object(runner.checked,'health'),patch.object(runner.checked,'sample',side_effect=sample),contextlib.redirect_stdout(io.StringIO()):
            if fault:
                with self.assertRaises(RuntimeError if fault=='health' else KeyboardInterrupt):
                    runner.run(b,initial,r,cancel=cancel,sleep=lambda _:None,now=lambda:next(clock)*.002)
            else:runner.run(b,initial,r,cancel=cancel,sleep=lambda _:None,now=lambda:next(clock)*.002)
        return p,r,b,track
    def test_resume_preserves_every_target_once_and_keeps_off_restore(self):
        p,r,b,t=self.run_case();self.assertGreaterEqual(t['reads'],4);self.assertEqual(r['status'],p.STATUS)
        self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
        np.testing.assert_array_equal([q for a,q in b.writes if a==116][2:],[q for phase,q in p.plan(p.source_position())])
        self.assertTrue(r['camera_gap_holds'][0]['resumed']);self.assertTrue(r['progress_history_reset_after_camera_hold'])
    def test_health_failure_while_paused_sends_no_next_goal_and_turns_off(self):
        p,r,b,t=self.run_case('health');self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
        self.assertEqual(sum(a==116 for a,q in b.writes),t['goal_count']);self.assertNotEqual(r.get('status'),p.STATUS)
    def test_cancel_while_paused_sends_no_next_goal_and_turns_off(self):
        p,r,b,t=self.run_case('cancel');self.assertTrue(r['all_torque_off_verified']);self.assertTrue(r['settings_restored'])
        self.assertEqual(sum(a==116 for a,q in b.writes),t['goal_count']);self.assertNotEqual(r.get('status'),p.STATUS)
if __name__=='__main__':unittest.main()
