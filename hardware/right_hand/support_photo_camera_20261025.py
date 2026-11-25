"""Bounded camera adjustment pause only before any motor configuration.

The shared camera and its ON blank/stale stops are unchanged. No motor API.
"""
import time
from camera_frame_signal import CameraImageUnavailable
from empty_start_marker_check import EmptyStartEvidence
from leap_book_cube_grasp_trial import Camera as OriginalCamera
OFF_ADJUST_PHASES=frozenset(('preflight','stable_off_readings','screen_current_pose','countdown'))
class SupportPhotoCamera(OriginalCamera):
    OFF_VIEW_WAIT_SECONDS=10.
    def _view_ready(self,now):
        with self.lock:
            packet=self.latest;quality=self.signal.latest
            return bool(packet is not None and 0<=now-packet[0]<=.3 and quality is not None and quality['usable'] and self.signal.ready())
    def check(self):
        phase=self.report.get('phase','preview')
        permitted=self.report.get('enabled_at') is None and phase in OFF_ADJUST_PHASES
        if not permitted or self._view_ready(time.monotonic()):return super().check()
        if self.error:raise RuntimeError('Camera recording failed: '+self.error)
        began=time.monotonic();event=dict(phase=phase,began_monotonic=began,torque_enable_not_started=True,
            maximum_wait_seconds=self.OFF_VIEW_WAIT_SECONDS,resumed=False)
        self.report.setdefault('camera_off_adjustment_waits',[]).append(event)
        self.empty_evidence=EmptyStartEvidence();self.empty_checked_stamp=None;self.empty_quality=None;self.empty_streak=0
        print('Camera view unavailable while motors are OFF. Clear lens / finish camera adjustment; waiting up to10s. Q stops.',flush=True)
        while True:
            now=time.monotonic()
            if self.error:raise RuntimeError('Camera recording failed: '+self.error)
            self.camera.check_preview('MOTORS OFF | camera adjustment pause | clear lens | Q stops')
            cancel=getattr(self,'off_wait_cancel',None)
            if cancel is not None:cancel()
            if now-began>=self.OFF_VIEW_WAIT_SECONDS:
                event['elapsed_seconds']=now-began
                raise CameraImageUnavailable('Camera view did not recover within10s while motors remained OFF. Clear the lens and show the hand before FIRST SPACE.')
            if self._view_ready(now):
                # The inherited check still rejects stable cube evidence and
                # camera errors. Restarting evidence requires10 new clear
                # marker-check frames after an adjustment, never a stale pass.
                super().check();quality=self.empty_quality
                if (quality is not None and 0<=now-quality['camera_read_monotonic']<=.3
                    and not quality['decoded_cube_ids'] and quality['consecutive_frames_without_decoded_cube']>=10):
                    event.update(resumed=True,elapsed_seconds=now-began,fresh_empty_check_frames=quality['consecutive_frames_without_decoded_cube'])
                    print('Fresh camera view restored. Continuing the OFF preparation checks.',flush=True);return
            time.sleep(.02)
