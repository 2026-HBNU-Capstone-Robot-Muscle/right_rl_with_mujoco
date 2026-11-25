"""Mode-scoped EMPTY camera; image gaps are handled by a live-health hold."""
import time
from quarter_finish_empty_camera_20261025 import QuarterFinishEmptyCamera
from empty_camera_gap_hold_20261026 import CameraGapHold
class RecoveringEmptyCamera(QuarterFinishEmptyCamera):
    def __init__(self,folder,report):
        self.motion_hold_bound=False
        super().__init__(folder,report)
        self.motion_guard=CameraGapHold(self.motion_view,self.report)
    def bind_motion_hold(self):self.motion_hold_bound=True
    def motion_view(self):
        if self.error:raise RuntimeError('Camera recording failed: '+self.error)
        with self.lock:
            packet=self.latest;quality=self.signal.latest;stable=self.signal.ready()
        self.report['camera_image_signal']=quality
        if packet is None:return dict(ready=False,reason='no_frame',stamp=None)
        stamp,_=packet;age=time.monotonic()-stamp
        ready=bool(0<=age<=.3 and stable and quality is not None and quality['usable'])
        reason='fresh_scene' if ready else 'stale' if age>.3 else (quality or {}).get('reason','unstable')
        return dict(ready=ready,reason=reason,stamp=stamp)
    def check(self):
        if self.motion_hold_bound and self.report.get('enabled_at') is not None and self.report.get('phase')!='post_off_reference':
            observation=self.motion_view()
            text='CAMERA GAP: goals held; live motor checks; Q stops' if not observation['ready'] else 'EMPTY test | '+self.report.get('phase','')
            self.camera.check_preview(text)
            return
        return super().check()
    def finish(self):
        result=super().finish();events=self.report.get('camera_gap_holds',[])
        result['recovered_gap_count']=sum(bool(e['resumed']) for e in events)
        result['continuous_video_verified']=not events and not result.get('image_signal_blank_frames')
        result['image_gap_events_recorded']=bool(events)
        return result
