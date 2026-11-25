"""Supervised experiment: EMPTY return along known poses, PLACE, full turn.

The command uses the exact previously EMPTY-tested goals. Cube response is a
first supervised experiment, not a robust or identified loaded controller.
"""
import hashlib,json
import numpy as np
from reverse_quarter_resume_empty_plan_20261025 import ReverseQuarterResumeEmptyPlan,ROOT,EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE,extra_hashes as previous_hashes
from reverse_quarter_path_empty_plan_20261025 import motion_goals
from book_grasp_trial_plan import RING_RELEASE
from book_empty_preparation_plan import LIFT
from grasp_recovery_plan import interpolate
from loaded_quarter_reentry_20261025 import release_target,validate_path,cmc_native_min
RETURN_PHASE='stage_3_empty_return_with_tip_clearance'
START_HOLD='stage_5_empty_start_hold'
TURN_PHASE='stage_6_loaded_quarter_turn'
END_HOLD='stage_7_loaded_observation'
THUMB_NEIGHBOR_PAIR='palm_collision_7|th_tip'
THUMB_NEIGHBOR_LIMIT_MM=.5
INITIAL_SELF_SCOPE='OFF_4_tick_same_pad_envelope_early_thumb_ring_release_v1'
FILES=('loaded_quarter_trial_plan_20261025.py','loaded_quarter_reentry_20261025.py','loaded_quarter_joint_limits_20261025.py','loaded_quarter_model_policy_20261025.py','loaded_quarter_id4_lower_setting_20261025.json','loaded_quarter_trial_gate_20261025.py','screen_loaded_quarter_trial_20261025.py',
    'loaded_quarter_trial_protocol_20261025.py','leap_loaded_quarter_trial_20261025.py',
    'right_hand_rotation_20261024/completed_empty_path_review_20261025/completion_review.json',
    'right_hand_rotation_20261024/completed_empty_path_review_20261025/actual_geometry_and_cube_replay_kinematic_v2.json',
    'right_hand_rotation_20261024/completed_empty_path_review_20261025/goal_indexed_actual_path.npz')
def extra_hashes():return {**previous_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}}

def empty_return_goals(anchor):
    original=motion_goals();points=[]
    for i in range(anchor,-1,-1):
        q=original[i].copy();q[0]-=int(round(16*min(1.,i/200.)));points.append(q)
    rows=[points[0]]
    for a,b in zip(points,points[1:]):
        count=max(1,int(np.ceil(np.max(abs(b-a))/2)))
        rows.extend(np.rint(a+(b-a)*k/count).astype(np.int64) for k in range(1,count+1))
    return np.array(rows,dtype=np.int64)

def validate_experiment_basis():
    folder=ROOT/'right_hand_rotation_20261024/completed_empty_path_review_20261025'
    actual=json.loads((folder/'completion_review.json').read_bytes());r=json.loads((folder/'actual_geometry_and_cube_replay_kinematic_v2.json').read_bytes())
    if (actual['status']!='reverse_quarter_resume_empty_complete' or not actual['all_torque_off_verified'] or not actual['settings_restored']
        or actual['source_sha256']!=hashlib.sha256((ROOT/actual['source']/'result.json').read_bytes()).hexdigest()
        or r['source_npz_sha256']!=hashlib.sha256((folder/'goal_indexed_actual_path.npz').read_bytes()).hexdigest()
        or r['raw_actual_records']!=762 or r['raw_actual_self_maxima'] or r['raw_actual_support_maxima']
        or not r['diagnostic_only'] or r['loaded_motor_response_verified'] or r['hardware_execution_allowed']):
        raise ValueError('Actual EMPTY evidence or conditional diagnostic changed')
    moving=next(s for s in r['cases'] if s['mode']=='measured_segment_splice' and s['friction']==.8)
    fixed=next(s for s in r['cases'] if s['mode']=='stationary_control' and s['friction']==.8)
    if (moving['hold_maximum_target_error_deg']>15 or moving['final_target_error_deg']>15
        or moving['external_cube_contact'] or not moving['hand_contact_at_end'] or moving['maximum_no_hand_contact_seconds']>.1
        or moving['maximum_self_overlap_mm']>1e-6 or moving['maximum_support_overlap_mm']>.15
        or fixed['hold_maximum_target_error_deg']<60):raise ValueError('Conditional moving/stationary model review failed')
    return dict(physical_cube_rotation_verified=False,robust_loaded_controller_verified=False,
        experimental_loaded_trial_only=True,measured_empty_trajectory_diagnostic=True,
        prescribed_loaded_hand_motion_is_an_assumption=True,source_actual_sha256=actual['source_sha256'],
        source_measured_trace_sha256=r['source_npz_sha256'],model_nominal_final_error_deg=moving['final_target_error_deg'],
        model_nominal_hold_error_deg=moving['hold_maximum_target_error_deg'],model_nominal_rotation_deg=moving['whole_cube_orientation_change_deg'],
        stationary_control_hold_error_deg=fixed['hold_maximum_target_error_deg'])

class LoadedQuarterTrialPlan(ReverseQuarterResumeEmptyPlan):
    PLAN_VERSION='supervised_loaded_quarter_v6_user_cmc_lower2deg_20261025'
    ON_TIMEOUT=120
    TITLE='CUBE QUARTER-TURN EXPERIMENT: EMPTY return -> PLACE/SPACE -> full 38s turn -> REMOVE/SPACE -> OFF.'
    COMPLETE_MESSAGE='One loaded reference completed; removal confirmed; OFF/settings restored. Actual Cube rotation will be reviewed from video.'
    FOLDER_PREFIX='loaded_quarter_trial'
    STATUS='loaded_quarter_trial_complete'
    CAPTURE_PLACEMENT=True
    CAPTURE_BEFORE_PHASE=TURN_PHASE
    ROTATION_INCLUDED=True
    proof_hashes=staticmethod(extra_hashes)

    def plan(self,raw):
        a=self.validate_start(raw);q=motion_goals()
        anchor=int(np.argmin(np.max(abs(q-a),axis=1)));return_path=empty_return_goals(anchor);join=return_path[0]
        release=release_target(a,join)
        self.validate_admission(release)
        rows=[];interpolate(rows,a,release,RING_RELEASE)
        if a[3]<1872:
            count=max(20,int(np.ceil(np.max(abs(join-release))/2)))
            for k in range(1,count+1):
                v=np.rint(release+(join-release)*k/count).astype(np.int64)
                for j in (7,11,15):
                    if join[j]>release[j]:v[j]=min(int(join[j]),int(release[j])+2*k)
                rows.append((LIFT,v))
        else:interpolate(rows,release,join,LIFT)
        rows.extend((RETURN_PHASE,v.copy()) for v in return_path[1:])
        rows.extend((START_HOLD,q[0].copy()) for _ in range(40))
        rows.extend((TURN_PHASE,v.copy()) for v in q[1:])
        rows.extend((END_HOLD,q[-1].copy()) for _ in range(60))
        validate_path(a,rows,RING_RELEASE)
        if (len(rows)+40)*.06+2>self.ON_TIMEOUT:raise ValueError('Combined experiment exceeds its existing bounded motion budget')
        return rows

    def frozen_for_phase(self,phase):
        if phase==RING_RELEASE:return np.array([i for i in range(16) if i not in self.release_indices()])
        if phase in (LIFT,RETURN_PHASE,TURN_PHASE):return np.array([],dtype=int)
        return np.arange(16)

    def progress_for_phase(self,phase):
        if phase==RING_RELEASE:return self.release_indices()
        if phase in (LIFT,RETURN_PHASE,TURN_PHASE):return np.arange(16)
        return np.array([],dtype=int)

    def release_indices(self):
        a=self.source_position();q=motion_goals();anchor=int(np.argmin(np.max(abs(q-a),axis=1)))
        out=[0,7,11,14]
        if a[3]<1872:out.append(3)
        if q[anchor,15]>a[15]:out.append(15)
        return np.array(out,dtype=int)
