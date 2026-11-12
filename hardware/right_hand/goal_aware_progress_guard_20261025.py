"""Detect a commanded stall without treating a static PD error as motion.

Only meaningful movement of the requested target starts a stall decision.
Tracking/current/voltage/temperature and held-joint checks remain separate.
"""
from collections import deque
import numpy as np

VERSION='meaningful_target_advance_progress_v1_20261025'
WINDOW_SECONDS=5.
MINIMUM_SAMPLES=20
MINIMUM_SECONDS=1.
MINIMUM_TARGET_ADVANCE_TICKS=16
MAXIMUM_UNRESPONSIVE_PROGRESS_TICKS=2
MINIMUM_DIRECTED_ERROR_TICKS=64

class GoalAwareProgressFault(RuntimeError):
    def __init__(self,details):
        self.details=details
        super().__init__(f"ID{details['motor_id']} target advanced {details['target_advance_ticks']} ticks but measured progress was {details['measured_progress_ticks']} ticks; no target advance")

class GoalAwareProgress:
    def __init__(self):self.rows={};self.directions={}
    def check(self,phase,indices,command,actual,at):
        command=np.asarray(command);actual=np.asarray(actual)
        if command.shape!=(16,) or actual.shape!=(16,) or not np.isfinite(command).all() or not np.isfinite(actual).all() or not np.isfinite(at):
            raise ValueError('Invalid progress observation')
        for index in indices:
            index=int(index);key=(phase,index);rows=self.rows.setdefault(key,deque(maxlen=512));direction=self.directions.get(key,0)
            if rows:
                if at<rows[-1][0]:raise ValueError('Progress time went backward')
                step=int(command[index])-rows[-1][1]
                new_direction=int(np.sign(step))
                if new_direction and direction and new_direction!=direction:
                    previous=rows[-1];rows.clear();rows.append(previous)
                if new_direction:direction=new_direction;self.directions[key]=direction
            rows.append((float(at),int(command[index]),int(actual[index])))
            while len(rows)>1 and at-rows[0][0]>WINDOW_SECONDS:rows.popleft()
            if not direction or len(rows)<MINIMUM_SAMPLES:continue
            old=rows[0];elapsed=at-old[0]
            target=direction*(int(command[index])-old[1]);measured=direction*(int(actual[index])-old[2])
            error=direction*(int(command[index])-int(actual[index]))
            if elapsed>=MINIMUM_SECONDS and target>=MINIMUM_TARGET_ADVANCE_TICKS and measured<=MAXIMUM_UNRESPONSIVE_PROGRESS_TICKS and error>=MINIMUM_DIRECTED_ERROR_TICKS:
                raise GoalAwareProgressFault(dict(motor_id=index+1,phase=phase,seconds=elapsed,samples=len(rows),
                    target_advance_ticks=target,measured_progress_ticks=measured,directed_error_ticks=error,
                    command_ticks=int(command[index]),actual_ticks=int(actual[index])))

def metadata():return dict(version=VERSION,window_seconds=WINDOW_SECONDS,minimum_samples=MINIMUM_SAMPLES,
    minimum_seconds=MINIMUM_SECONDS,minimum_target_advance_ticks=MINIMUM_TARGET_ADVANCE_TICKS,
    maximum_unresponsive_progress_ticks=MAXIMUM_UNRESPONSIVE_PROGRESS_TICKS,minimum_directed_error_ticks=MINIMUM_DIRECTED_ERROR_TICKS,
    tracking_current_voltage_temperature_and_held_joint_checks_unchanged=True)
