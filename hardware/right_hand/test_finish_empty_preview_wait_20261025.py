"""Reproduce the first-frame race and verify the bus-open boundary."""
import contextlib,io,subprocess,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import Mock,patch
from camera_frame_signal import CameraImageUnavailable
from quarter_finish_empty_camera_20261025 import (wait_for_first_space_preview,QuarterFinishEmptyCamera,
    FIRST_FRAME_PENDING)
from support_photo_camera_20261025 import SupportPhotoCamera
class Clock:
    def __init__(self):self.value=0.
    def now(self):return self.value
    def sleep(self,n):self.value+=n
class PreviewWaitChecks(unittest.TestCase):
    def wait(self,attempt,report=None,cancel=lambda:None,timeout=5.):
        report={} if report is None else report;clock=Clock()
        with contextlib.redirect_stdout(io.StringIO()):wait_for_first_space_preview(attempt,report,cancel=cancel,now=clock.now,sleep=clock.sleep,timeout=timeout)
        return report
    def test_no_first_packet_then_fresh_image_retains_space(self):
        attempt=Mock(side_effect=[RuntimeError(FIRST_FRAME_PENDING),None]);r=self.wait(attempt)
        self.assertEqual(attempt.call_count,2);self.assertTrue(r['camera_first_space_waits'][0]['resumed']);self.assertTrue(r['camera_first_space_waits'][0]['first_space_retained'])
    def test_unstable_frames_and_empty_marker_screen_must_become_ready(self):
        attempt=Mock(side_effect=[CameraImageUnavailable('not stable'),CameraImageUnavailable('10 fresh EMPTY frames required'),None]);r=self.wait(attempt)
        self.assertEqual(attempt.call_count,3);self.assertTrue(r['camera_first_space_waits'][0]['resumed'])
    def test_fatal_worker_failure_is_not_retried(self):
        attempt=Mock(side_effect=RuntimeError('Camera recording failed: worker lost'))
        with self.assertRaisesRegex(RuntimeError,'worker lost'):self.wait(attempt)
        self.assertEqual(attempt.call_count,1)
    def test_timeout_is_retryable_without_motor_authorization(self):
        r={'hardware_authorized':False};attempt=Mock(side_effect=RuntimeError(FIRST_FRAME_PENDING))
        with self.assertRaises(CameraImageUnavailable):self.wait(attempt,r,timeout=.06)
        self.assertFalse(r['hardware_authorized']);self.assertTrue(r['camera_first_space_waits'][0]['timed_out'])
    def test_cancel_interrupts_pending_space(self):
        cancel=Mock(side_effect=KeyboardInterrupt('cancel'));attempt=Mock()
        with self.assertRaises(KeyboardInterrupt):self.wait(attempt,cancel=cancel)
        attempt.assert_not_called()
    def test_ready_preview_has_no_wait_event(self):
        r=self.wait(Mock());self.assertNotIn('camera_first_space_waits',r)
    def test_preview_class_uses_wait_only_before_authorization(self):
        camera=QuarterFinishEmptyCamera.__new__(QuarterFinishEmptyCamera);camera.report={'phase':'preview','hardware_authorized':False}
        with patch.object(SupportPhotoCamera,'require_fresh'),patch('quarter_finish_empty_camera_20261025.wait_for_first_space_preview') as waiting:
            camera.require_fresh();waiting.assert_called_once()
    def test_after_authorization_and_on_freshness_errors_still_propagate(self):
        for report in ({'phase':'preflight','hardware_authorized':True},{'phase':'configure_off'},{'phase':'stage_7_empty_root_finish','enabled_at':10.}):
            camera=QuarterFinishEmptyCamera.__new__(QuarterFinishEmptyCamera);camera.report=report
            with patch.object(SupportPhotoCamera,'require_fresh',side_effect=RuntimeError(FIRST_FRAME_PENDING)),patch('quarter_finish_empty_camera_20261025.wait_for_first_space_preview') as waiting:
                with self.assertRaises(RuntimeError):camera.require_fresh()
                waiting.assert_not_called()
    def test_on_check_is_the_unchanged_parent_check(self):
        self.assertIs(QuarterFinishEmptyCamera.check,SupportPhotoCamera.check);self.assertTrue(QuarterFinishEmptyCamera.EMPTY_ONLY)
    def test_default_mode_opens_no_devices(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B','leap_quarter_finish_empty_preview_fixed_20261025.py'],capture_output=True,text=True,encoding='utf-8',timeout=8)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)
    def test_runtime_does_not_open_bus_until_pending_space_succeeds(self):
        import msvcrt
        import quarter_current_runtime_20261025 as runtime
        from quarter_finish_empty_plan_20261025 import QuarterFinishEmptyPlan
        camera=Mock();camera.finish.return_value={'status':'incomplete','frame_count':0};bus_factory=Mock(side_effect=AssertionError('Motor bus opened before preview readiness'))
        camera.require_fresh.side_effect=KeyboardInterrupt('Stop before preview ready')
        with tempfile.TemporaryDirectory() as d,patch.object(runtime,'ROOT',Path(d)),patch.object(QuarterFinishEmptyPlan,'CAMERA_TYPE',Mock(return_value=camera),create=True),patch.object(msvcrt,'kbhit',return_value=True),patch.object(msvcrt,'getwch',return_value=' '),patch.object(sys,'argv',['empty','--execute']),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(runtime.main(QuarterFinishEmptyPlan,bus_type=bus_factory),1)
        bus_factory.assert_not_called()
if __name__=='__main__':unittest.main()
