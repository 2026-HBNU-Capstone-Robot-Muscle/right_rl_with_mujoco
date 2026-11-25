"""Offline phase protocol: right motion may occupy the upper-layer space.

Only a ready lower grip grants left access; clearance must then remain fresh
through approach, turn and retreat. Intents are records, never device commands.
No transport adapters, serial imports or camera calls exist in this module.
"""
from dataclasses import dataclass,field
from enum import Enum
import math

class Phase(str,Enum):
    IDLE='idle'
    ROTATE='right_whole_cube_rotation'
    REGRASP='right_regrasp_and_clear_upper_space'
    APPROACH='left_approach'
    TURN='left_upper_layer_turn'
    RETREAT='left_retreat'
    READY='ready_for_next_reviewed_primitive'
    STOPPED='stop_latched'

@dataclass(frozen=True)
class Observation:
    received_at_s:float
    motor_ids:frozenset
    healthy:bool
    targets_constant:bool=False
    settled:bool=False
    lower_grip_confirmed:bool=False
    target_cube_pose_confirmed:bool=False
    upper_sweep_gap_mm:float|None=None
    clearance_frame:str|None=None

    def fresh(self,now):
        return bool(math.isfinite(now) and math.isfinite(self.received_at_s) and self.healthy and self.motor_ids==frozenset(range(1,17)) and 0<=now-self.received_at_s<=.2)

    def handoff_ready(self,now,frame):
        gap=self.upper_sweep_gap_mm
        return bool(self.fresh(now) and self.targets_constant and self.settled and self.lower_grip_confirmed and self.target_cube_pose_confirmed and self.clearance_frame==frame and gap is not None and math.isfinite(gap) and gap>.3)

@dataclass(frozen=True)
class Plan:
    path_sha256:str
    entry_grip:str
    exit_grip:str
    common_frame:str
    layer_degrees:int
    timeouts:dict
    def __post_init__(self):
        if len(self.path_sha256)!=64 or any(c not in '0123456789abcdef' for c in self.path_sha256):raise ValueError('reviewed path SHA256 required')
        if not all((self.entry_grip,self.exit_grip,self.common_frame)) or self.layer_degrees not in (-90,90):raise ValueError('reviewed primitive entry, exit, frame and layer direction required')
        phases={p.value for p in (Phase.ROTATE,Phase.REGRASP,Phase.APPROACH,Phase.TURN,Phase.RETREAT)}
        if set(self.timeouts)!=phases or any(not math.isfinite(float(v)) or float(v)<=0 for v in self.timeouts.values()):raise ValueError('explicit finite positive phase timeouts required')

@dataclass(frozen=True)
class Intent:
    token:str
    recipient:str
    operation:str
    payload:dict=field(default_factory=dict)
    simulated:bool=True

class Coordinator:
    def __init__(self,entry_grip=None):
        self.phase=Phase.IDLE;self.grip=entry_grip;self.frame=None;self.left_clear=False;self.plan=None;self.pending=None;self.deadline=None;self.sequence=0;self.seen=set();self.history=[]
    def _intent(self,recipient,operation,payload):
        self.sequence+=1
        return Intent(f'{self.sequence}:{operation}',recipient,operation,payload)
    def _wait(self,phase,recipient,operation,now,payload):
        self.phase=phase;item=self._intent(recipient,operation,payload);self.pending=item.token;self.deadline=now+float(self.plan.timeouts[phase.value]);self.history.append(dict(event='intent',phase=phase.value,token=item.token,operation=operation,payload=payload));return [item]
    def start(self,plan,now,*,left_clear_confirmed):
        if not math.isfinite(now):raise ValueError('finite time required')
        if self.phase not in (Phase.IDLE,Phase.READY) or not left_clear_confirmed:return []
        if self.grip!=plan.entry_grip or (self.frame is not None and self.frame!=plan.common_frame):
            self.history.append(dict(event='rejected_start',reason='primitive_entry_or_frame_mismatch'));return []
        self.plan=plan;self.frame=plan.common_frame;self.left_clear=True
        return self._wait(Phase.ROTATE,'right','rotate_whole_cube',now,dict(path_sha256=plan.path_sha256,upper_clearance_required=False,left_must_remain_clear=True,automatic_open=False))
    def receive(self,token,kind,now,observation=None,*,left_clear_confirmed=False):
        if not math.isfinite(now):raise ValueError('finite time required')
        if self.phase==Phase.STOPPED:return []
        if self.deadline is not None and now>self.deadline:return self.fault('phase_timeout')
        if token in self.seen or token!=self.pending:return []
        expected={Phase.ROTATE:'rotation_complete',Phase.REGRASP:'lower_grip_and_upper_space_ready',Phase.APPROACH:'at_turn_start',Phase.TURN:'layer_turn_complete',Phase.RETREAT:'left_clear_of_cube'}
        if kind!=expected.get(self.phase):return []
        if self.phase in (Phase.ROTATE,Phase.REGRASP):
            if not left_clear_confirmed:return self.fault('left_not_clear_during_right_motion')
            ready=observation is not None and (observation.fresh(now) if self.phase==Phase.ROTATE else observation.handoff_ready(now,self.frame))
            if not ready:return []
        else:
            if observation is None or not observation.handoff_ready(now,self.frame):return self.fault('right_hold_or_upper_clearance_lost_while_left_active')
            if self.phase==Phase.RETREAT and not left_clear_confirmed:return []
        self.seen.add(token)
        if self.phase==Phase.ROTATE:
            return self._wait(Phase.REGRASP,'right','regrasp_then_clear_upper_space',now,dict(common_frame=self.frame,upper_clearance_required_at_handoff=True,left_must_remain_clear=True))
        if self.phase==Phase.REGRASP:
            self.grip=self.plan.exit_grip;self.left_clear=False
            return self._wait(Phase.APPROACH,'left','approach_cube',now,dict(common_frame=self.frame,right_targets_constant=True))
        if self.phase==Phase.APPROACH:
            return self._wait(Phase.TURN,'left','turn_upper_layer',now,dict(degrees=self.plan.layer_degrees,right_targets_constant=True))
        if self.phase==Phase.TURN:
            return self._wait(Phase.RETREAT,'left','retreat_clear_of_cube',now,dict(right_targets_constant=True,upper_clearance_required_until_left_clear=True))
        self.left_clear=True;self.phase=Phase.READY;self.pending=None;self.deadline=None
        return []
    def tick(self,now,observation=None):
        if not math.isfinite(now):raise ValueError('finite time required')
        if self.phase==Phase.STOPPED:return []
        if self.deadline is not None and now>self.deadline:return self.fault('phase_timeout')
        if self.phase in (Phase.APPROACH,Phase.TURN,Phase.RETREAT) and (observation is None or not observation.handoff_ready(now,self.frame)):
            return self.fault('right_hold_or_upper_clearance_lost_while_left_active')
        return []
    def fault(self,reason):
        if self.phase==Phase.STOPPED:return []
        self.phase=Phase.STOPPED;self.grip=None;self.left_clear=False;self.pending=None;self.deadline=None;self.history.append(dict(event='fault',reason=reason))
        return [self._intent(side,'stop_requested',dict(reason=reason,automatic_retry=False)) for side in ('right','left')]
