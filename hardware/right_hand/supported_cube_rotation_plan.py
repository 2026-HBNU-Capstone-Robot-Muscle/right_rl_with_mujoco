"""Bounded loaded observation; smaller than the completed EMPTY rehearsal."""
import hashlib
import json
import numpy as np
from supported_yaw_empty_plan import SupportedYawEmptyPlan,opening_rows
from grasp_place_plan import GraspPlacePlan,endpoints,LOADED_PHASE
from grasp_recovery_plan import interpolate
from hand_motor_mapping import TICK
from thumb_live_start_plan import ROOT
from supported_cube_rotation_capture import SupportedCubeRotationCapture

OUT=ROOT/'right_hand_rotation_20261023'
CANDIDATE=OUT/'supported_loaded_candidate.json'
CANDIDATE_SHA='d56504cc5b51eb9e28f575f56274ca9a430c5cdef88c9fd5727c9d50c2ee8d4a'
SCREEN=OUT/'supported_loaded_prefix_256_scale_085.json'
LIMITS=OUT/'native_motor_limits.json'
EMPTY=ROOT/'supported_yaw_empty_20261023_030552_675209/result.json'
REVIEW=EMPTY.parent/'completion_review.json'
ROTATION_PHASE='stage_4_supported_cube_rotation'

def candidate_record():
    blob=CANDIDATE.read_bytes()
    if hashlib.sha256(blob).hexdigest()!=CANDIDATE_SHA:raise ValueError('Loaded candidate changed; no writes')
    r=json.loads(blob);_,base=endpoints();q=np.asarray(r['goal_ticks'])
    if (r['version']!='supported_loaded_observation_v1' or r['live_sac'] is not False
            or r['physical_rotation_verified'] is not False or r['loaded_observation_ready'] is not True
            or r['clearance_tracking_cases_passed']!=154 or r['sample_period_s']!=.05
            or r['pre_rotation_hold_samples']!=0 or r['motion_scale']!=.85
            or r['base_goal_ticks']!=base.tolist() or q.shape!=(256,16)
            or not np.isfinite(q).all() or np.any(q!=np.rint(q))
            or np.max(abs(np.diff(np.vstack([base,q]),axis=0)))>2
            or np.max(abs(q-base))>256 or np.any(q[:,[12,14,15]]>base[[12,14,15]])):
        raise ValueError('Invalid loaded observation candidate; no writes')
    for path,key in ((SCREEN,'independent_screen_sha256'),(LIMITS,'native_limits_sha256'),(REVIEW,'empty_review_sha256')):
        if hashlib.sha256(path.read_bytes()).hexdigest()!=r[key]:raise ValueError('Loaded observation evidence changed; no writes')
    return r

def candidate():return np.asarray(candidate_record()['goal_ticks'],dtype=np.int64)

def admission(raw):
    a=SupportedYawEmptyPlan.validate_admission(raw);record=candidate_record()
    bounds=json.loads(LIMITS.read_bytes());excess=np.maximum(0,np.maximum(np.asarray(bounds['lower_ticks'])-a,a-np.asarray(bounds['upper_ticks'])))*TICK
    # Observed post-OFF ID4 sag was 0.064rad outside the modeled interval.
    # Include the separately screened +/-4-tick neighborhood (<0.071rad),
    # capped at 0.08rad for ID4 only; all other admission bounds stay 0.06.
    allowed=np.full(16,.06);allowed[3]=.08
    if np.any(excess>allowed+1e-6):raise ValueError('OFF pose exceeds bounded native-range re-entry; no writes')
    opened,closed=endpoints();rows=opening_rows(a,opened);interpolate(rows,opened,closed,LOADED_PHASE)
    if (len(rows)+256+20)*.06+2>75:raise ValueError('Loaded observation exceeds 75s delay budget; no writes')
    return a

def healthy_off(state):
    GraspPlacePlan.validate_off_state(state)
    return admission(state['position'])

class SupportedCubeRotationPlan(SupportedYawEmptyPlan):
    PLAN_VERSION='cube_supported_reorientation_observation_v1'
    STATUS='cube_supported_reorientation_path_complete'
    TITLE='CUBE REORIENTATION: EMPTY release/CONFIRM -> OPEN -> PLACE -> grasp -> small rotation -> REMOVE -> OFF.'
    FOLDER_PREFIX='cube_supported_rotation_trial'
    COMPLETE_MESSAGE='Reorientation command path complete; cube removed; OFF/settings restored. Physical rotation needs observation.'
    EMPTY_RELEASE_ONLY=False
    CAPTURE_BEFORE_PHASE=LOADED_PHASE
    CAPTURE_PROTOCOL_TYPE=SupportedCubeRotationCapture
    MAX_START_RANGE_EXCESS_RAD=.08
    validate_admission=staticmethod(admission)
    validate_off_state=staticmethod(healthy_off)
    EXTRA_PLANNER_FILES=SupportedYawEmptyPlan.EXTRA_PLANNER_FILES+(
        'supported_cube_rotation_plan.py','supported_cube_rotation_capture.py','leap_cube_supported_rotation.py',
        'validate_supported_loaded_20261023.py',
        'right_hand_rotation_20261023/supported_loaded_candidate.json',
        'right_hand_rotation_20261023/supported_loaded_prefix_256_scale_085.json',
        'right_hand_rotation_20261023/native_motor_limits.json',
        'supported_yaw_empty_20261023_030552_675209/result.json',
        'supported_yaw_empty_20261023_030552_675209/completion_review.json')

    def __init__(self,request):
        super().__init__(request);record=candidate_record()
        if hashlib.sha256(EMPTY.read_bytes()).hexdigest()!=record['empty_rehearsal_result_sha256']:
            raise ValueError('Completed EMPTY rehearsal changed; no writes')
        q=candidate();changed=np.ptp(np.vstack([endpoints()[1],q]),axis=0)>0
        self._suffix_frozen=np.flatnonzero(~changed);self._suffix_progress=np.flatnonzero(changed)
        # Evidence/JSON work belongs to OFF construction/proof validation.
        # validate_start/plan also run after torque enable and inside the
        # write gate; they must use this immutable, already verified snapshot.
        self._opened,self._closed=(v.copy() for v in endpoints())
        self._candidate=q.copy()
        bounds=json.loads(LIMITS.read_bytes())
        self._lower=np.asarray(bounds['lower_ticks'],dtype=float)
        self._upper=np.asarray(bounds['upper_ticks'],dtype=float)
        for v in (self._opened,self._closed,self._candidate,self._lower,self._upper):v.flags.writeable=False

    def validate_start(self,raw):
        a=np.asarray(raw)
        if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
            raise ValueError('Invalid raw start; no writes')
        nearest=self._opened if max(abs(a-self._opened))<max(abs(a-self._closed)) else self._closed
        if np.any(abs(a-nearest)>512) or np.any(a<0) or np.any(a>4095):
            raise ValueError('Empty recovery travel exceeds 512 ticks; no writes')
        allowed=np.full(16,.06);allowed[3]=.08
        excess=np.maximum(0,np.maximum(self._lower-a,a-self._upper))*TICK
        if np.any(excess>allowed+1e-6):raise ValueError('OFF pose exceeds bounded native-range re-entry; no writes')
        if np.any(abs(a-self.center)>4):
            raise ValueError('Pose moved beyond the screened four-tick neighborhood; no writes')
        prefix_count=len(opening_rows(a,self._opened))+max(20,int(np.ceil(max(abs(self._closed-self._opened))/2)))
        if prefix_count*.05+2>50:raise ValueError('Measured opening exceeds the 50s planning budget; no writes')
        if (prefix_count+256+20)*.06+2>75:raise ValueError('Loaded observation exceeds 75s delay budget; no writes')
        return a.astype(np.int64)

    def plan(self,raw):
        a=self.validate_start(raw);rows=opening_rows(a,self._opened)
        self._release_tip_moves=any(q[12]!=a[12] for phase,q in rows if phase=='stage_0_empty_recover')
        interpolate(rows,self._opened,self._closed,LOADED_PHASE)
        # Keep both former admission budgets, including for an adopted +/-4
        # enable hold. No historical file access is needed to check them.
        if len(rows)*.05+2>50:raise ValueError('Measured opening exceeds the 50s planning budget; no writes')
        if (len(rows)+256+20)*.06+2>75:raise ValueError('Loaded observation exceeds 75s delay budget; no writes')
        rows.extend((ROTATION_PHASE,q.copy()) for q in self._candidate)
        return rows

    def frozen_for_phase(self,phase):
        if phase==ROTATION_PHASE:return self._suffix_frozen.copy()
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase==ROTATION_PHASE:return self._suffix_progress.copy()
        return super().progress_for_phase(phase)
