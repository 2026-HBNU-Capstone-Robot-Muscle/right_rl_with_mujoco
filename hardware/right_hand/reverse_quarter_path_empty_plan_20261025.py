"""Whole proposed quarter-turn hand path, supervised EMPTY testing only."""
import hashlib,json
import numpy as np
from reverse_place_capture_plan_20261025 import (ReversePlacePlan,ROOT,TARGET,PHOTO_HOLD,END_HOLD,BASE_RELEASE,
    CASING_PAIR,CASING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE,extra_hashes as previous_hashes)

START_HOLD='stage_5_empty_quarter_start_hold'
TURN_PHASE='stage_6_empty_quarter_reference_path'
MOTION_FILE='right_hand_rotation_20261024/reverse_quarter_path_empty/quarter_turn_goals_v1.json'
FILES=('reverse_quarter_path_empty_plan_20261025.py','reverse_quarter_path_empty_gate_20261025.py',
    'screen_reverse_quarter_path_empty_20261025.py','leap_reverse_quarter_path_empty_20261025.py',MOTION_FILE)

def extra_hashes():return {**previous_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}}

def motion_goals():
    r=json.loads((ROOT/MOTION_FILE).read_bytes());a=np.asarray(r['raw_goal_ticks'])
    if a.ndim!=2 or a.shape[1]!=16 or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
        raise ValueError('Invalid EMPTY reference goals')
    if r['motor_ids']!=list(range(1,17)) or r['sample_period_seconds']!=.05 or r['loaded_cube_execution_allowed'] is not False:
        raise ValueError('Wrong reference order, timing or loaded claim')
    if not np.array_equal(a[0],TARGET) or np.max(abs(np.diff(a,axis=0)))>2:raise ValueError('Reference start or step mismatch')
    limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
    if np.any(a<np.asarray(limits['lower_ticks'])) or np.any(a>np.asarray(limits['upper_ticks'])):raise ValueError('Motion leaves native ranges')
    return a.astype(np.int64)

class ReverseQuarterEmptyPlan(ReversePlacePlan):
    PLAN_VERSION='reverse_quarter_whole_hand_path_EMPTY_v1_20261025'
    TITLE='EMPTY QUARTER-TURN PATH: fresh recovery -> START -> whole 38s hand path -> OFF. NO CUBE.'
    COMPLETE_MESSAGE='Whole quarter-turn hand path completed EMPTY. OFF/settings restored. Cube rotation was NOT tested.'
    FOLDER_PREFIX='reverse_quarter_path_empty'
    STATUS='reverse_quarter_path_empty_complete'
    CAPTURE_PLACEMENT=False
    CAPTURE_BEFORE_PHASE=None
    ROTATION_INCLUDED=False
    proof_hashes=staticmethod(extra_hashes)

    def plan(self,raw):
        prep=super().plan(raw)
        rows=[(p,q) for p,q in prep if p not in (PHOTO_HOLD,END_HOLD)]
        rows.extend((START_HOLD,TARGET.copy()) for _ in range(40))
        values=motion_goals();rows.extend((TURN_PHASE,q.copy()) for q in values[1:])
        rows.extend((END_HOLD,values[-1].copy()) for _ in range(20))
        if (len(rows)+20)*.06+2>self.ON_TIMEOUT:raise ValueError('Whole EMPTY reference exceeds bounded time budget')
        return rows

    def frozen_for_phase(self,phase):
        if phase in (START_HOLD,END_HOLD):return np.arange(16)
        if phase==TURN_PHASE:return np.array([],dtype=int)
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase in (START_HOLD,END_HOLD):return np.array([],dtype=int)
        if phase==TURN_PHASE:return np.arange(16)
        return super().progress_for_phase(phase)
