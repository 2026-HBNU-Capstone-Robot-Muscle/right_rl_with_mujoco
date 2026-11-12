"""Retain FIRST SPACE while a fresh EMPTY preview warms up.

This wait is restricted to preview, before hardware authorization or bus open.
All configuration/ON camera failures and shared camera modules are unchanged.
"""
import time
from camera_frame_signal import CameraImageUnavailable
from support_photo_camera_20261025 import SupportPhotoCamera

FIRST_SPACE_WAIT_SECONDS=5.
FIRST_FRAME_PENDING='Wait for a fresh preview; no motor writes'

def poll_terminal_cancel():
    import msvcrt
    while msvcrt.kbhit():
        if msvcrt.getwch() in ('q','Q','\x1b','\x03'):
            raise KeyboardInterrupt('User stop while waiting for camera preview')

def wait_for_first_space_preview(attempt,report,*,cancel=poll_terminal_cancel,
                                 now=time.monotonic,sleep=time.sleep,
                                 timeout=FIRST_SPACE_WAIT_SECONDS):
    began=now();event=None
    while True:
        cancel()
        try:
            attempt()
        except CameraImageUnavailable as exc:
            reason=str(exc)
        except RuntimeError as exc:
            # Only the known no-first-packet race is transient. Recording,
            # stale-camera, worker and device errors still propagate unchanged.
            if str(exc)!=FIRST_FRAME_PENDING:raise
            reason=str(exc)
        else:
            if event is not None:event.update(resumed=True,elapsed_seconds=now()-began)
            return
        if event is None:
            event=dict(began_monotonic=began,maximum_wait_seconds=timeout,
                       before_motor_port_open=True,first_space_retained=True,resumed=False)
            report.setdefault('camera_first_space_waits',[]).append(event)
            print('Camera preview warming up; FIRST SPACE retained. No motor port opened. Q stops.',flush=True)
        event['last_not_ready_reason']=reason
        elapsed=now()-began
        if elapsed>=timeout:
            event.update(elapsed_seconds=elapsed,timed_out=True)
            raise CameraImageUnavailable('Camera preview is not ready yet. Motors have not been opened. '
                'Keep the hand EMPTY in a clear camera view; press SPACE again when the picture is stable.')
        sleep(min(.02,max(0.,timeout-elapsed)))

class QuarterFinishEmptyCamera(SupportPhotoCamera):
    EMPTY_ONLY=True
    def require_fresh(self):
        attempt=super().require_fresh
        if (self.report.get('phase','preview')=='preview'
                and self.report.get('enabled_at') is None
                and not self.report.get('hardware_authorized',False)):
            return wait_for_first_space_preview(attempt,self.report)
        return attempt()
