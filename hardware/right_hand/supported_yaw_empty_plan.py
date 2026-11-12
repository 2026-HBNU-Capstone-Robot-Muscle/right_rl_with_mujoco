"""EMPTY rehearsal of a new ring-involving suffix, never a loaded policy."""
import hashlib
import json
import numpy as np
from grasp_place_plan import GraspPlacePlan,admission as grasp_admission,endpoints,LOADED_PHASE
from grasp_recovery_plan import empty_opening,interpolate
from thumb_live_start_plan import ROOT
from supported_yaw_empty_capture import EmptySupportedYawCapture

SUFFIX_PHASE='stage_4_supported_yaw_EMPTY'
APPROACH_PHASE='stage_3_grasp_rehearsal_EMPTY'
CANDIDATE=ROOT/'right_hand_rotation_20261022/supported_yaw_empty_candidate.json'
CANDIDATE_SHA='bd35396e7542fc70246f6d0c3f33c00c2d2f8eb79dc10413fa557daa72dedd69'
RING_TIP_RELEASE_TICKS=64


def opening_rows(raw,opened):
    # A short PIP release keeps its original path. Add tip extension only for
    # travel beyond the existing first-separation window, smoothly capped at
    # 64 ticks; all exact starts and adopted +/-4-tick states are screened.
    extra=max(0,min(RING_TIP_RELEASE_TICKS,int(raw[14]-opened[14])-
                    GraspPlacePlan.MAX_CONTACT_RELEASE_TRAVEL_TICKS))
    return empty_opening(raw,opened,ring_tip_extension_ticks=extra)


def candidate():
    raw=CANDIDATE.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=CANDIDATE_SHA:raise ValueError('EMPTY suffix candidate changed; no writes')
    r=json.loads(raw);q=np.asarray(r['goal_ticks']);_,base=endpoints()
    if (r['version']!='supported_yaw_empty_rehearsal_v1' or r['cube_allowed'] is not False
            or r['loaded_test_authorized'] is not False or r['live_sac'] is not False
            or q.shape!=(256,16) or not np.isfinite(q).all() or np.any(q!=np.rint(q))
            or r['base_goal_ticks']!=base.tolist() or r['sample_period_s']!=.05
            or np.max(abs(q-base))>256 or np.max(abs(np.diff(np.vstack([base,q]),axis=0)))>2
            or np.any(q[:,[12,14,15]]>base[[12,14,15]])):
        raise ValueError('Invalid EMPTY suffix; no writes')
    screen=ROOT/'right_hand_rotation_20261022/supported_yaw_empty_suffix_screen.json'
    if hashlib.sha256(screen.read_bytes()).hexdigest()!=r['empty_suffix_screen_sha256']:
        raise ValueError('EMPTY suffix screen changed; no writes')
    return q.astype(np.int64)


def admission(raw):
    a=grasp_admission(raw);opened,closed=endpoints()
    rows=opening_rows(a,opened)
    interpolate(rows,opened,closed,APPROACH_PHASE)
    # 60ms includes the measured-delay regression's 10ms scheduling jitter.
    # Final hold plus 2s fixed overhead must fit the same bounded 75s trial.
    seconds=(len(rows)+len(candidate())+20)*.06+2
    if seconds>75:raise ValueError('EMPTY full route exceeds the 75s delay budget; no writes')
    return a


def healthy_off(state):
    GraspPlacePlan.validate_off_state(state)
    return admission(state['position'])


class SupportedYawEmptyPlan(GraspPlacePlan):
    PLAN_VERSION='supported_yaw_empty_rehearsal_v2'
    TITLE='EMPTY RING-INVOLVING REHEARSAL v2: extend ring tip/PIP -> CONFIRM -> preparation -> suffix -> photo -> OFF.'
    STATUS='empty_supported_yaw_rehearsal_complete'
    FOLDER_PREFIX='supported_yaw_empty'
    COMPLETE_MESSAGE='EMPTY rehearsal complete. Torque OFF/settings restored. No cube was allowed; physical rotation NOT tested.'
    ON_TIMEOUT=75
    CAPTURE_PLACEMENT=True  # Buffered terminal Q/SPACE, also during release.
    CAPTURE_BEFORE_PHASE=None
    EMPTY_RELEASE_ONLY=True
    CAPTURE_PROTOCOL_TYPE=EmptySupportedYawCapture
    EXTRA_PLANNER_FILES=GraspPlacePlan.EXTRA_PLANNER_FILES+(
        'supported_yaw_empty_plan.py','supported_yaw_empty_capture.py',
        'leap_supported_yaw_empty.py',
        'right_hand_rotation_20261022/supported_yaw_empty_candidate.json',
        'right_hand_rotation_20261022/supported_yaw_empty_suffix_screen.json')
    validate_admission=staticmethod(admission)
    validate_off_state=staticmethod(healthy_off)

    def __init__(self,request):
        super().__init__(request)
        changed=np.ptp(np.vstack([endpoints()[1],candidate()]),axis=0)>0
        self._suffix_frozen=np.flatnonzero(~changed)
        self._suffix_progress=np.flatnonzero(changed)
        self._release_tip_moves=False

    def plan(self,raw):
        a=self.validate_start(raw);opened,closed=endpoints()
        rows=opening_rows(a,opened)
        # Recomputed for the exact adopted enable hold, before any path cycle.
        self._release_tip_moves=any(q[12]!=a[12] for phase,q in rows if phase=='stage_0_empty_recover')
        interpolate(rows,opened,closed,APPROACH_PHASE)
        rows.extend((SUFFIX_PHASE,q.copy()) for q in candidate())
        return rows

    def frozen_for_phase(self,phase):
        if phase=='stage_0_empty_recover':
            moving=(3,12,14) if self._release_tip_moves else (3,14)
            return np.array([i for i in range(16) if i not in moving])
        if phase==SUFFIX_PHASE:
            return self._suffix_frozen.copy()
        return super().frozen_for_phase(LOADED_PHASE if phase==APPROACH_PHASE else phase)

    def progress_for_phase(self,phase):
        if phase=='stage_0_empty_recover':
            return np.array([3,12,14] if self._release_tip_moves else [3,14])
        if phase==SUFFIX_PHASE:
            return self._suffix_progress.copy()
        return super().progress_for_phase(LOADED_PHASE if phase==APPROACH_PHASE else phase)
