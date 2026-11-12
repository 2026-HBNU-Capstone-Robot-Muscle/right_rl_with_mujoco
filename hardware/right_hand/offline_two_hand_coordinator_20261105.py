"""Device-free integration protocol. Intents are data, never motor commands.

This module has no serial, camera, network or transport adapter. Production
admission is deliberately separate from the simulated sequence demonstration.
Existing per-hand current, health, cancellation and torque-OFF logic must stay
inside the actual controllers when a later physical integration is performed.
"""
from dataclasses import dataclass,field,asdict
from enum import Enum

class Phase(str,Enum):
    IDLE='idle'
    RIGHT='waiting_right_rotation_and_hold'
    LEFT_APPROACH='waiting_left_approach'
    LEFT_TURN='waiting_left_layer_turn'
    LEFT_RETREAT='waiting_left_clear'
    COMPLETE='cycle_complete_right_still_holding'
    STOPPED='stop_latched'

@dataclass(frozen=True)
class RightHealth:
    received_at_s:float
    ids:frozenset
    healthy:bool
    targets_constant:bool=False
    settled:bool=False
    cube_held_confirmed:bool=False
    def ready(self,now_s):
        return bool(self.healthy and self.ids==frozenset(range(1,17)) and 0<=now_s-self.received_at_s<=.2 and self.targets_constant and self.settled and self.cube_held_confirmed)

@dataclass(frozen=True)
class CyclePlan:
    path_id:str
    path_sha256:str
    common_cube_frame:str
    layer_turn_degrees:int
    timeout_s:dict
    entry_grip_id:str=None
    exit_grip_id:str=None
    def __post_init__(self):
        if not self.path_id or len(self.path_sha256)!=64 or any(c not in '0123456789abcdef' for c in self.path_sha256):raise ValueError('source-bound path identity required')
        if not self.common_cube_frame or not self.entry_grip_id or not self.exit_grip_id or self.layer_turn_degrees not in (-90,90):raise ValueError('reviewed common frame, primitive entry/exit and one layer turn required')
        required={p.value for p in (Phase.RIGHT,Phase.LEFT_APPROACH,Phase.LEFT_TURN,Phase.LEFT_RETREAT)}
        if set(self.timeout_s)!=required or any(float(v)<=0 for v in self.timeout_s.values()):raise ValueError('provide reviewed per-phase time budgets; there are no implicit live defaults')

@dataclass(frozen=True)
class Intent:
    token:str
    recipient:str
    operation:str
    payload:dict=field(default_factory=dict)
    simulated:bool=True

@dataclass
class FutureLiveEvidence:
    offline_model_review_passed:bool=False
    initial_empty_recovery_review_passed:bool=False
    physical_grip_verified:bool=False
    physical_target_orientation_verified:bool=False
    left_alignment_verified:bool=False
    actual_layer_reaction_verified:bool=False
    loaded_primitive_entry_verified:bool=False
    loaded_primitive_exit_verified:bool=False
    def missing(self):return [k for k,v in asdict(self).items() if not v]
    def live_ready(self):return not self.missing()

class Coordinator:
    def __init__(self,initial_grip_id=None):
        self.phase=Phase.IDLE;self.cycle=0;self.sequence=0;self.pending=None;self.deadline=None;self.plan=None;self.seen=set();self.history=[];self.left_clear=False;self.last_health=None;self.current_grip_id=initial_grip_id;self.common_frame=None
    def _intent(self,recipient,operation,payload):
        self.sequence+=1
        return Intent(f'{self.cycle}:{self.sequence}:{operation}',recipient,operation,payload)
    def _wait(self,phase,recipient,operation,now_s,payload):
        self.phase=phase;intent=self._intent(recipient,operation,payload);self.pending=intent.token;self.deadline=now_s+float(self.plan.timeout_s[phase.value]);self.history.append(dict(event='intent',phase=phase.value,**asdict(intent)));return [intent]
    def start(self,plan,now_s,*,left_clear_confirmed):
        if self.phase not in (Phase.IDLE,Phase.COMPLETE) or not left_clear_confirmed:return []
        if self.current_grip_id!=plan.entry_grip_id or (self.common_frame is not None and self.common_frame!=plan.common_cube_frame):
            self.history.append(dict(event='rejected_start',reason='primitive entry or common frame mismatch',current_grip=self.current_grip_id,required_entry=plan.entry_grip_id));return []
        self.plan=plan;self.cycle+=1;self.left_clear=True;self.common_frame=plan.common_cube_frame
        return self._wait(Phase.RIGHT,'right','rotate_then_hold',now_s,dict(path_id=plan.path_id,path_sha256=plan.path_sha256,common_cube_frame=plan.common_cube_frame,automatic_open=False))
    def receive(self,token,kind,now_s,*,right_health=None,left_clear_confirmed=False):
        if self.phase==Phase.STOPPED:return []
        if self.deadline is not None and now_s>self.deadline:return self.fault('reviewed phase time budget exceeded')
        if token in self.seen:return []
        if right_health is not None:self.last_health=right_health
        if token!=self.pending:
            self.history.append(dict(event='rejected_ack',reason='wrong request token',token=token,kind=kind));return []
        expected={Phase.RIGHT:'rotation_complete_hold_ready',Phase.LEFT_APPROACH:'at_turn_start',Phase.LEFT_TURN:'layer_turn_complete',Phase.LEFT_RETREAT:'clear_of_cube'}
        if kind!=expected.get(self.phase):
            self.history.append(dict(event='rejected_ack',reason='wrong phase acknowledgement',token=token,kind=kind));return []
        if self.phase!=Phase.LEFT_RETREAT and (self.last_health is None or not self.last_health.ready(now_s)):
            self.history.append(dict(event='waiting',reason='right hold is not fresh and ready',token=token));return []
        if self.phase==Phase.LEFT_RETREAT and not left_clear_confirmed:
            self.history.append(dict(event='waiting',reason='left clearance not confirmed',token=token));return []
        self.seen.add(token);self.history.append(dict(event='accepted_ack',phase=self.phase.value,token=token,kind=kind))
        if self.phase==Phase.RIGHT:
            self.current_grip_id=self.plan.exit_grip_id
            self.left_clear=False
            return self._wait(Phase.LEFT_APPROACH,'left','approach_common_frame',now_s,dict(common_cube_frame=self.plan.common_cube_frame))
        if self.phase==Phase.LEFT_APPROACH:
            return self._wait(Phase.LEFT_TURN,'left','turn_upper_layer',now_s,dict(degrees=self.plan.layer_turn_degrees,common_cube_frame=self.plan.common_cube_frame))
        if self.phase==Phase.LEFT_TURN:
            return self._wait(Phase.LEFT_RETREAT,'left','retreat_clear_of_cube',now_s,dict(right_must_keep_holding=True))
        self.left_clear=True;self.phase=Phase.COMPLETE;self.pending=None;self.deadline=None
        return []
    def tick(self,now_s):
        if self.deadline is not None and now_s>self.deadline:return self.fault('reviewed phase time budget exceeded')
        return []
    def fault(self,reason):
        if self.phase==Phase.STOPPED:return []
        self.phase=Phase.STOPPED;self.pending=None;self.deadline=None;self.left_clear=False;self.current_grip_id=None;self.history.append(dict(event='fault',reason=reason))
        # Each real controller would handle its own existing stop/OFF policy.
        # These records do not send OFF, targets or physical stop instructions.
        return [self._intent(side,'stop_requested',dict(reason=reason,no_automatic_retry=True)) for side in ('right','left')]
    def reset(self,*,explicit_operator_reset,right_OFF_verified,left_OFF_verified,left_clear_confirmed):
        if self.phase!=Phase.STOPPED or not all((explicit_operator_reset,right_OFF_verified,left_OFF_verified,left_clear_confirmed)):return False
        self.phase=Phase.IDLE;self.pending=None;self.deadline=None;self.left_clear=True;self.last_health=None;self.history.append(dict(event='explicit_reset_to_idle',automatic_motion=False));return True
