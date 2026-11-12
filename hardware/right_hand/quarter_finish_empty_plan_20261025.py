"""EMPTY full reference plus an 11.25-degree root continuation; no cube."""
import hashlib,json
import numpy as np
from support_photo_path_plan_20261025 import SupportPhotoPathPlan,ROOT,extra_hashes as previous_hashes
from loaded_quarter_trial_plan_20261025 import (TURN_PHASE as OLD_TURN,END_HOLD as OLD_END,
    START_HOLD,RETURN_PHASE,motion_goals,empty_return_goals,EXTRA_RING_PAIR,
    EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE,THUMB_NEIGHBOR_PAIR,
    THUMB_NEIGHBOR_LIMIT_MM,INITIAL_SELF_SCOPE)
from loaded_quarter_reentry_20261025 import validate_path,release_target
from book_grasp_trial_plan import RING_RELEASE
from book_empty_preparation_plan import LIFT
from grasp_recovery_plan import interpolate
TURN_PHASE='stage_6_empty_quarter_reference';FINISH_PHASE='stage_7_empty_root_finish';END_HOLD='stage_8_empty_observation'
FIXTURE=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/finish_empty_fixture_v3'
FILES=('quarter_finish_empty_plan_20261025.py','quarter_finish_empty_gate_20261025.py',
    'screen_quarter_finish_empty_20261025.py','leap_quarter_finish_empty_20261025.py')
def extra_hashes():return {**previous_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}}
def finish_goals():
    q=motion_goals()[-1];a=np.tile(q,(65,1));a[:,[7,11]]+=np.arange(65)[:,None]*2
    return a
def empty_basis():
    path=ROOT/'loaded_quarter_trial_20261025_171607_019752/result.json';blob=path.read_bytes();r=json.loads(blob)
    expected='778dadb516e249ff65a6389385c9987f4326979fb446048ecd3a8ba23438fa1d'
    if hashlib.sha256(blob).hexdigest()!=expected or not r['all_torque_off_verified'] or not r['settings_restored']:raise ValueError('Prior completed physical reference changed')
    rows=[s['goal'] for s in r['samples'] if s['phase']==OLD_TURN]
    # A health sample records the previous accepted command BEFORE the next
    # write. The final command is separately verified in the subsequent hold.
    if (not np.array_equal(rows,motion_goals()[:-1]) or r.get('last_issued_command')!=motion_goals()[-1].tolist()
            or not any(s['phase']==OLD_END and s['goal']==motion_goals()[-1].tolist() and s['goal_readback']==s['goal'] for s in r['samples'])):
        raise ValueError('Prior physical reference does not match 760 goals and final readback')
    return dict(prior_physical_reference_sha256=expected,original_reference_goal_count=760,
        new_root_finish_goal_count=64,new_root_motor_ids=[8,12],new_root_extra_ticks=128,
        new_root_travel_degrees=11.25,empty_test_only=True,physical_cube_rotation_verified=False)
class QuarterFinishEmptyPlan(SupportPhotoPathPlan):
    PLAN_VERSION='quarter_reference_plus_root128_EMPTY_v1_20261025'
    TITLE='EMPTY FINISH CHECK: recovery -> known reference -> NEW 3.2s root finish -> hold -> OFF. NO CUBE.'
    COMPLETE_MESSAGE='New finish checked EMPTY. Torque OFF/settings restored. No physical cube rotation tested.'
    FOLDER_PREFIX='quarter_finish_empty';STATUS='quarter_finish_empty_complete'
    CAPTURE_PLACEMENT=False;CAPTURE_BEFORE_PHASE=None;CAPTURE_PROTOCOL_TYPE=None;CONTACT_RELEASE_BEFORE_PHASE=None
    ROTATION_INCLUDED=False;MODEL_LIMITS_DESCRIPTION='ID4 user lower -2deg; all other limits and health stops unchanged'
    proof_hashes=staticmethod(extra_hashes)
    def plan(self,raw):
        # The observed OFF thumb MCP settled farther in after power-off.
        # Begin its remaining opening during the existing ring/tip release,
        # rather than folding the thumb tip against its motor housing first.
        a=self.validate_start(raw);q=motion_goals();anchor=int(np.argmin(np.max(abs(q-a),axis=1)));return_path=empty_return_goals(anchor);join=return_path[0]
        release=release_target(a,join);release[1]-=min(64,max(0,int(a[1]-join[1])));self.validate_admission(release)
        rows=[];interpolate(rows,a,release,RING_RELEASE)
        if a[3]<1872:
            count=max(20,int(np.ceil(np.max(abs(join-release))/2)))
            for k in range(1,count+1):
                v=np.rint(release+(join-release)*k/count).astype(np.int64)
                for j in (7,11,15):
                    if join[j]>release[j]:v[j]=min(int(join[j]),int(release[j])+2*k)
                rows.append((LIFT,v))
        else:interpolate(rows,release,join,LIFT)
        rows.extend((RETURN_PHASE,v.copy()) for v in return_path[1:]);rows.extend((START_HOLD,q[0].copy()) for _ in range(40));rows.extend((TURN_PHASE,v.copy()) for v in q[1:])
        rows.extend((FINISH_PHASE,q.copy()) for q in finish_goals()[1:]);rows.extend((END_HOLD,finish_goals()[-1].copy()) for _ in range(60))
        validate_path(np.asarray(raw),rows,RING_RELEASE)
        if (len(rows)+40)*.06+2>self.ON_TIMEOUT:raise ValueError('EMPTY finish exceeds existing bounded motion time')
        return rows
    def frozen_for_phase(self,phase):
        if phase==TURN_PHASE:return np.array([],dtype=int)
        if phase==FINISH_PHASE:return np.array([i for i in range(16) if i not in (7,11)],dtype=int)
        return super().frozen_for_phase(phase)
    def progress_for_phase(self,phase):
        if phase==TURN_PHASE:return np.arange(16)
        if phase==FINISH_PHASE:return np.array([7,11],dtype=int)
        return super().progress_for_phase(phase)
    def release_indices(self):return np.unique(np.r_[super().release_indices(),1]).astype(int)
