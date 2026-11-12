"""Bounded EMPTY-only image-gap hold, with uninterrupted health reads.

No motor writes are implemented here. The caller supplies the existing fresh
health reader. A recovered image requires three distinct good frames <=0.3s old.
"""
import time
from camera_frame_signal import CameraImageUnavailable
MAX_SINGLE_HOLD_S=5.
MAX_TOTAL_HOLD_S=10.
def metadata():
    return dict(version='EMPTY_camera_gap_fixed_goal_live_health_v1_20261026',
                empty_only=True,max_single_hold_s=MAX_SINGLE_HOLD_S,max_total_hold_s=MAX_TOTAL_HOLD_S,
                maximum_frame_age_s=.3,required_good_frames=3,goal_advance_during_gap=False,
                goal_resend_during_gap=False,health_reads_during_hold=True,
                current_voltage_tracking_temperature_watchdog_checks_unchanged=True,
                hold_time_included_in_motion_timeout=True,camera_reopened_with_torque_on=False)
class CameraGapHold:
    def __init__(self,view,report):self.view=view;self.report=report;self.total=0.
    def wait_if_needed(self,read_healthy,*,phase,command,cancel,now=time.monotonic,sleep=time.sleep,motion_deadline=None):
        first=self.view()
        if first['ready']:return 0.
        remaining=MAX_TOTAL_HOLD_S-self.total
        if remaining<=0:raise CameraImageUnavailable('Camera recovery hold budget exhausted; OFF')
        began=now();budget=min(MAX_SINGLE_HOLD_S,remaining)
        if motion_deadline is not None:budget=min(budget,max(0.,motion_deadline-began))
        event=dict(phase=phase,began_monotonic=began,maximum_wait_seconds=budget,
                   reason=first['reason'],held_goal=list(map(int,command)),healthy_reads=0,resumed=False,
                   goal_writes_during_hold=0,camera_restarted=False)
        self.report.setdefault('camera_gap_holds',[]).append(event)
        self.report['camera_gap_hold_policy']=metadata()
        print(f'Camera image interrupted. Holding the LAST goal, checking motors; wait up to{budget:g}s. Q stops.',flush=True)
        completed=False
        try:
            while True:
                cancel()
                if now()-began>=budget:raise CameraImageUnavailable(f'Camera image did not recover within{budget:g}s while holding; OFF')
                read_healthy();event['healthy_reads']+=1
                observation=self.view();event['last_reason']=observation['reason']
                if observation['ready']:
                    elapsed=now()-began
                    if elapsed>=budget:raise CameraImageUnavailable('Camera returned after recovery deadline; OFF')
                    event.update(resumed=True,recovered_camera_stamp=observation.get('stamp'))
                    completed=True;print('Fresh camera image restored. Continuing from the next queued goal.',flush=True)
                    return elapsed
                sleep(min(.02,max(0.,budget-(now()-began))))
        finally:
            elapsed=max(0.,now()-began);self.total+=elapsed
            event.update(elapsed_seconds=elapsed,total_hold_seconds=self.total)
            if not completed:event['stopped_without_target_advance']=True
            self.report['camera_gap_total_hold_seconds']=self.total
