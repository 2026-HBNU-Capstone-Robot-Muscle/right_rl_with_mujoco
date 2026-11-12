"""Screened slow preparation/grasp demo with a 3s fixed-goal observation.

No cube rotation or SAC actions. Existing reached opening/grasp endpoints,
original 450mA/PID and stop thresholds. Every measured start is screened.
"""
import json
import numpy as np

from grasp_place_plan import GraspPlacePlan,endpoints,LOADED_PHASE
from grasp_recovery_plan import interpolate
from supported_yaw_empty_plan import opening_rows
from hand_motor_mapping import TICK
from marker_trial_camera import MarkerTrialCamera
from thumb_live_start_plan import ROOT

OBSERVATION_PHASE='stage_4_fixed_grasp_observation'
OBSERVATION_SAMPLES=60
MOTION_LIMIT_SECONDS=60
LIMITS=ROOT/'right_hand_rotation_20261023/native_motor_limits.json'


def demo_opening(raw,opened):
    rows=opening_rows(raw,opened)
    release=[q for phase,q in rows if phase=='stage_0_empty_recover']
    target=int(release[-1][12]);initial=int(raw[12]);step=0
    for phase,q in rows:
        if phase=='stage_0_empty_recover':
            step+=1
            # Complete the already bounded tip extension early, while the PIP
            # keeps its original <=2-tick schedule. This gives the same ring
            # contact deadline time to settle, without changing any threshold.
            q[12]=initial+int(np.clip(target-initial,-2*step,2*step))
    return rows


def validate_numbers(raw,opened,closed,lower,upper):
    a=np.asarray(raw)
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
        raise ValueError('Invalid raw start; no writes')
    # Each joint must be near either actually reached endpoint. A passive
    # mixed OFF pose need not match one entire historical ON waypoint.
    distance=np.minimum(abs(a-opened),abs(a-closed))
    if np.any(distance>512) or np.any(a<0) or np.any(a>4095):
        raise ValueError('Current joint exceeds 512-tick endpoint family; no writes')
    excess=np.maximum(0,np.maximum(lower-a,a-upper))*TICK
    if np.any(excess>.06+1e-6):raise ValueError('OFF pose exceeds existing 0.06rad native re-entry; no writes')
    rows=demo_opening(a,opened);interpolate(rows,opened,closed,LOADED_PHASE)
    if (len(rows)+OBSERVATION_SAMPLES+20)*.06+2>MOTION_LIMIT_SECONDS:
        raise ValueError('Slow grasp demo exceeds 60s delayed-cycle motion budget; no writes')
    return a.astype(np.int64)


def admission(raw):
    opened,closed=endpoints();limits=json.loads(LIMITS.read_bytes())
    return validate_numbers(raw,opened,closed,np.array(limits['lower_ticks']),np.array(limits['upper_ticks']))


def healthy_off(state):
    a=admission(state['position'])
    for key in ('torque','error','watchdog','voltage','temperature'):
        value=np.asarray(state[key])
        if value.shape!=(16,) or not np.isfinite(value).all() or np.any(value!=np.rint(value)):
            raise ValueError('Invalid fresh OFF telemetry; no writes')
    if (any(np.any(state[k]) for k in ('torque','error','watchdog'))
        or np.any(np.array(state['voltage'])<38) or np.any(np.array(state['voltage'])>65)
        or np.any(np.array(state['temperature'])>45)):
        raise ValueError('Unhealthy OFF state; no writes')
    return a


class SlowGraspDemoPlan(GraspPlacePlan):
    PLAN_VERSION='slow_grasp_demo_measured_endpoint_family_v5'
    TITLE='SLOW GRASP DEMO: EMPTY release/CONFIRM -> OPEN -> PLACE/SPACE -> grasp -> fixed 3s -> REMOVE/SPACE -> OFF.'
    STATUS='slow_grasp_demo_complete'
    FOLDER_PREFIX='slow_grasp_demo'
    COMPLETE_MESSAGE='Slow grasp demo complete; cube removed; OFF/settings restored. Cube rotation NOT attempted.'
    # The 118-sample EMPTY release lasts 5.9s before any other finger moves.
    # Half-gain dynamics briefly recontact the same palm boxes after the old
    # 2.5s deadline (<=0.006mm), with no new/deeper pair. Allow separation to
    # settle by 3s, still inside this EMPTY segment and before operator confirm.
    # The measured +/-4 neighborhood needs at most 43 commands/86 PIP ticks,
    # all with monotonically decreasing original contacts. Bound this demo to
    # 44 commands/88 ticks. No deeper or new contact is permitted. The original
    # GraspPlacePlan's 40/80/2.5 limits are not modified.
    INITIAL_SELF_ESCAPE_DYNAMIC_S=3.
    INITIAL_SELF_ESCAPE_GOALS=44
    MAX_CONTACT_RELEASE_TRAVEL_TICKS=88
    ON_TIMEOUT=MOTION_LIMIT_SECONDS
    CAMERA_TYPE=MarkerTrialCamera
    validate_admission=staticmethod(admission)
    validate_off_state=staticmethod(healthy_off)
    EXTRA_PLANNER_FILES=GraspPlacePlan.EXTRA_PLANNER_FILES+(
        'slow_grasp_demo_plan.py','leap_slow_grasp_demo.py','marker_trial_camera.py','empty_start_marker_check.py',
        'leap_thumb_prepare_current_empty.py',
        'fixed_exposure_camera.py','camera_exposure_sweep.py','cube_marker_readiness.py',
        'supported_yaw_empty_plan.py','right_hand_rotation_20261023/native_motor_limits.json')

    def __init__(self,request):
        super().__init__(request)
        self._opened,self._closed=(v.copy() for v in endpoints())
        limits=json.loads(LIMITS.read_bytes())
        self._lower=np.array(limits['lower_ticks']);self._upper=np.array(limits['upper_ticks'])
        for value in (self._opened,self._closed,self._lower,self._upper):value.flags.writeable=False
        self._release_tip_moves=False

    def validate_start(self,raw):
        a=validate_numbers(raw,self._opened,self._closed,self._lower,self._upper)
        if np.any(abs(a-self.center)>4):raise ValueError('Pose moved beyond screened +/-4 ticks; no writes')
        return a

    def plan(self,raw):
        a=self.validate_start(raw);rows=demo_opening(a,self._opened)
        self._release_tip_moves=any(q[12]!=a[12] for phase,q in rows if phase=='stage_0_empty_recover')
        interpolate(rows,self._opened,self._closed,LOADED_PHASE)
        rows.extend((OBSERVATION_PHASE,self._closed.copy()) for _ in range(OBSERVATION_SAMPLES))
        return rows

    def frozen_for_phase(self,phase):
        if phase=='stage_0_empty_recover':
            moving=(3,12,14) if self._release_tip_moves else (3,14)
            return np.array([i for i in range(16) if i not in moving])
        if phase==OBSERVATION_PHASE:return np.arange(16)
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase=='stage_0_empty_recover':return np.array([3,12,14] if self._release_tip_moves else [3,14])
        if phase==OBSERVATION_PHASE:return np.array([],dtype=int)
        return super().progress_for_phase(phase)
