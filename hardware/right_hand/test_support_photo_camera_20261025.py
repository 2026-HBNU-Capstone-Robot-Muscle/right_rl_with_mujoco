"""OFF-only camera adjustment must not weaken any active-motion stops."""
import contextlib,io,threading,unittest
from unittest.mock import Mock,patch
import numpy as np
import support_photo_camera_20261025 as module
from camera_frame_signal import FrameSignal,CameraImageUnavailable
class Clock:
    def __init__(self):self.value=100.7;self.on_sleep=None
    def now(self):return self.value
    def sleep(self,seconds):
        self.value+=seconds
        if self.on_sleep:self.on_sleep()
class CameraChecks(unittest.TestCase):
    def setup_camera(self,phase='screen_current_pose',enabled=None):
        clock=Clock();c=module.SupportPhotoCamera.__new__(module.SupportPhotoCamera)
        c.report={'phase':phase,'enabled_at':enabled,'recording_requested':True};c.lock=threading.Lock();c.error=None;c.started=90.
        c.camera=Mock();c.signal=FrameSignal();black=np.zeros((24,24,3),np.uint8);c.latest=(100.,black);c.signal.observe(100.,black)
        c.empty_evidence=None;c.empty_checked_stamp=None;c.empty_quality=None;c.empty_streak=0
        c.off_wait_cancel=Mock();c.good=np.indices((24,24)).sum(axis=0).astype(np.uint8);c.good=np.repeat((c.good*5)[...,None],3,axis=2)
        return c,clock
    def context(self,clock):return contextlib.ExitStack()
    def test_off_recovery_requires_fresh_clear_evidence_and_resumes(self):
        c,clock=self.setup_camera();seen=[]
        def sleep():
            if clock.value>=100.76:c.signal.observe(clock.value,c.good);c.latest=(clock.value,c.good)
        clock.on_sleep=sleep
        def inherited(_):
            seen.append(clock.value);c.empty_quality=dict(camera_read_monotonic=clock.value,decoded_cube_ids=[],consecutive_frames_without_decoded_cube=len(seen))
        with patch.object(module.time,'monotonic',side_effect=clock.now),patch.object(module.time,'sleep',side_effect=clock.sleep),patch.object(module.OriginalCamera,'check',autospec=True,side_effect=inherited),contextlib.redirect_stdout(io.StringIO()):c.check()
        event=c.report['camera_off_adjustment_waits'][0];self.assertTrue(event['resumed']);self.assertGreaterEqual(event['fresh_empty_check_frames'],10);self.assertIsNone(c.error);self.assertGreaterEqual(c.off_wait_cancel.call_count,10)
    def test_enabled_camera_blank_still_stops_at_original_threshold(self):
        c,clock=self.setup_camera('stage_0_roots',90.)
        with patch.object(module.time,'monotonic',side_effect=clock.now),self.assertRaisesRegex(CameraImageUnavailable,'0.5s'):c.check()
        self.assertNotIn('camera_off_adjustment_waits',c.report);self.assertIsNotNone(c.error)
    def test_configuring_phase_is_not_treated_as_adjustable_off(self):
        c,clock=self.setup_camera('configure_off',None)
        with patch.object(module.time,'monotonic',side_effect=clock.now),self.assertRaisesRegex(CameraImageUnavailable,'0.5s'):c.check()
        self.assertNotIn('camera_off_adjustment_waits',c.report)
    def test_off_timeout_does_not_resume_or_latch_false_success(self):
        c,clock=self.setup_camera()
        with patch.object(module.time,'monotonic',side_effect=clock.now),patch.object(module.time,'sleep',side_effect=clock.sleep),contextlib.redirect_stdout(io.StringIO()),self.assertRaisesRegex(CameraImageUnavailable,'within10s'):c.check()
        self.assertFalse(c.report['camera_off_adjustment_waits'][0]['resumed']);self.assertGreaterEqual(clock.value-100.7,10)
    def test_q_during_off_wait_is_not_suppressed(self):
        c,clock=self.setup_camera();c.off_wait_cancel.side_effect=KeyboardInterrupt('Q')
        with patch.object(module.time,'monotonic',side_effect=clock.now),contextlib.redirect_stdout(io.StringIO()),self.assertRaises(KeyboardInterrupt):c.check()
        self.assertFalse(c.report['camera_off_adjustment_waits'][0]['resumed'])
    def test_inherited_cube_detection_error_is_preserved(self):
        c,clock=self.setup_camera()
        def sleep():c.signal.observe(clock.value,c.good);c.latest=(clock.value,c.good)
        clock.on_sleep=sleep
        with patch.object(module.time,'monotonic',side_effect=clock.now),patch.object(module.time,'sleep',side_effect=clock.sleep),patch.object(module.OriginalCamera,'check',side_effect=CameraImageUnavailable('Cube marker confirmed')),contextlib.redirect_stdout(io.StringIO()),self.assertRaisesRegex(CameraImageUnavailable,'Cube marker confirmed'):c.check()
        self.assertFalse(c.report['camera_off_adjustment_waits'][0]['resumed'])
    def test_healthy_off_uses_unmodified_parent_check(self):
        c,clock=self.setup_camera()
        for s in (100.60,100.62,100.64):c.signal.observe(s,c.good)
        c.latest=(100.64,c.good)
        with patch.object(module.time,'monotonic',side_effect=clock.now),patch.object(module.OriginalCamera,'check') as parent:c.check();parent.assert_called_once()
        self.assertNotIn('camera_off_adjustment_waits',c.report)
if __name__=='__main__':unittest.main()
