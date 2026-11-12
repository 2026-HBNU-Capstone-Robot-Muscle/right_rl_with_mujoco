"""EMPTY test of the user's thumb-retreat / index-middle lift proposal."""
import hashlib
import numpy as np
from slow_forward_plan_20261026 import (SlowForwardPlan,ROOT,extra_hashes as parent_hashes,
    TURN_PHASE as OLD_TURN,FINISH_PHASE as OLD_FINISH,END_HOLD as OLD_HOLD,loaded_basis,finish_goals)
from loaded_quarter_trial_plan_20261025 import motion_goals,empty_return_goals,RETURN_PHASE
from loaded_quarter_reentry_20261025 import validate_path
from grasp_recovery_plan import interpolate
from book_grasp_trial_plan import RING_RELEASE

TURN_PHASE='stage_6_empty_quarter_reference'
FINISH_PHASE='stage_7_empty_known_root_finish'
THUMB_CLEAR='stage_9_empty_thumb_retreat'
THUMB_LIFT='stage_8a_empty_thumb_lift_clearance'
ROOT_UP='stage_10_empty_index_middle_lift'
END_HOLD='stage_11_empty_new_pose_hold'
THUMB_DELTA=np.array([0,-128,256,-128],dtype=np.int64)
ROOT_EXTRA=384
FIXTURE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454/thumb_root_empty_fixture_v5_corner_resolution'
FILES=('thumb_clear_root_empty_plan_20261026.py','thumb_clear_root_empty_gate_20261026.py',
       'screen_thumb_clear_root_empty_20261026.py','leap_thumb_clear_root_empty_20261026.py',
       'leap_thumb_clear_root_empty.cmd','return_corner_resolution_20261026.py')
def extra_hashes():return {**parent_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}}
def tail_goals():
    start=finish_goals()[-1].copy();thumb=start.copy();thumb[:4]+=THUMB_DELTA
    end=thumb.copy();end[[7,11]]+=ROOT_EXTRA
    lifted=start.copy();lifted[3]=thumb[3]
    rows=[];interpolate(rows,start,lifted,THUMB_LIFT);interpolate(rows,lifted,thumb,THUMB_CLEAR);interpolate(rows,thumb,end,ROOT_UP)
    return rows
def final_goal():return tail_goals()[-1][1].copy()
def proposal_metadata():
    return dict(thumb_motor_ids=[1,2,3,4],thumb_delta_ticks=THUMB_DELTA.tolist(),
                additional_root_motor_ids=[8,12],additional_root_ticks=ROOT_EXTRA,
                thumb_lift_goal_count=64,thumb_clear_goal_count=128,root_up_goal_count=192,
                order='thumb CMC lift -> thumb MCP/axial retreat -> index/middle roots',
                native_limit_extension=False,empty_only=True,physical_rotation_verified=False)
class ThumbRootEmptyPlan(SlowForwardPlan):
    PLAN_VERSION='EMPTY_sequential_thumb_root384_return_corner_resolution_v4_20261026'
    TITLE='EMPTY PROPOSAL CHECK: known recovery/reference -> thumb retreat -> raise index/middle -> hold -> OFF.'
    COMPLETE_MESSAGE='New thumb retreat and root lift checked EMPTY. OFF/settings restored. Cube rotation still unverified.'
    STATUS='thumb_clear_root_empty_complete';FOLDER_PREFIX='thumb_clear_root_empty'
    CAPTURE_PLACEMENT=False;CAPTURE_BEFORE_PHASE=None;CAPTURE_PROTOCOL_TYPE=None
    CONTACT_RELEASE_BEFORE_PHASE=None;ROTATION_INCLUDED=False
    proof_hashes=staticmethod(extra_hashes)
    def plan(self,raw):
        base=super().plan(raw);cut=next(i for i,(p,q) in enumerate(base) if p==OLD_HOLD)
        rename={OLD_TURN:TURN_PHASE,OLD_FINISH:FINISH_PHASE}
        rows=[(rename.get(p,p),q.copy()) for p,q in base[:cut]]
        rows.extend(tail_goals());rows.extend((END_HOLD,final_goal()) for _ in range(60))
        validate_path(np.asarray(raw),rows,RING_RELEASE)
        if (len(rows)+40)*.085+2>self.ON_TIMEOUT:raise ValueError('EMPTY proposal exceeds180s motion allowance')
        return rows
    def frozen_for_phase(self,phase):
        if phase==THUMB_LIFT:return np.array([i for i in range(16) if i!=3],dtype=int)
        if phase==THUMB_CLEAR:return np.array([0,*range(3,16)],dtype=int)
        if phase==ROOT_UP:return np.array([i for i in range(16) if i not in (7,11)],dtype=int)
        return super().frozen_for_phase({TURN_PHASE:OLD_TURN,FINISH_PHASE:OLD_FINISH,END_HOLD:OLD_HOLD}.get(phase,phase))
    def progress_for_phase(self,phase):
        if phase==THUMB_LIFT:return np.array([3],dtype=int)
        if phase==THUMB_CLEAR:return np.array([1,2],dtype=int)
        if phase==ROOT_UP:return np.array([7,11],dtype=int)
        return super().progress_for_phase({TURN_PHASE:OLD_TURN,FINISH_PHASE:OLD_FINISH,END_HOLD:OLD_HOLD}.get(phase,phase))
