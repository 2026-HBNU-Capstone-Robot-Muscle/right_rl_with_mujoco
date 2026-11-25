"""Frozen measured-baseline small clockwise experiment with bounded index root."""
import hashlib
import json
import numpy as np
from book_grasp_trial_plan import BookGraspPlan,ROOT,reached_grasp_goal,extra_hashes as grasp_hashes
from book_small_yaw_plan import BookSmallYawPlan,YAW
from book_dynamic_yaw_capture import BookDynamicYawCapture

CANDIDATE='right_hand_rotation_20261024/book_yaw_measured_response/dynamic_clockwise_candidate_v2.json'
CANDIDATE_SHA='a8a25f2019833bf7d64f04c7d189dd7462964471c0a244fa162052ed8b293373'
FROZEN_IDS=(4,12,13,14,15,16)
MOVING=np.array([i for i in range(16) if i+1 not in FROZEN_IDS])
SOURCES=('book_small_yaw_trial_20261024_215736_594865/result.json',
    'book_small_yaw_trial_20261024_215736_594865/actual_yaw_review.json',
    'book_small_yaw_trial_20261024_215736_594865/operator_correction.json',
    'right_hand_rotation_20261024/book_yaw_measured_response/measured_baseline_source.json',
    'right_hand_rotation_20261024/book_yaw_measured_response/relative_candidate_replay.json',
    'right_hand_rotation_20261024/book_yaw_measured_response/dynamic_contact_search.json',
    'search_book_yaw_dynamics.py','right_hand_rotation_20261023/native_motor_limits.json')
EXTRA=('book_dynamic_yaw_plan.py','book_dynamic_yaw_capture.py','book_dynamic_yaw_gate.py',
    'screen_book_dynamic_yaw.py','leap_book_dynamic_yaw_trial.py','leap_book_small_yaw_trial.cmd',
    'book_small_yaw_plan.py','book_small_yaw_capture.py',CANDIDATE)


def validate_candidate(r):
    flags={'experimental_trial_only':True,'physical_cube_rotation_verified':False,
        'live_sac':False,'robust_policy':False,'marker_to_model_registration_verified':False,
        'cube_friction_measured':False,'fitted_joint_zero_offsets':False,
        'following_error_used_only_in_offline_simulation':True,'current_or_PID_changed':False,
        'hardware_execution_allowed':False,'source_baseline_precedes_operator_removal_and_OFF':True,
        'source_removal_flag_is_not_physical_removal_proof':True}
    if (r.get('version')!='book_measured_dynamic_clockwise_candidate_v2'
        or r.get('desired_physical_yaw_degrees')!=-3.
        or r.get('motion_direction')!='clockwise_viewed_from_above'
        or r.get('optimizer')!='bounded_cross_entropy_search_not_SAC'
        or any(r.get(k) is not v for k,v in flags.items())
        or r.get('closure_samples')!=26 or r.get('rotation_samples')!=100
        or r.get('moving_motor_ids')!=(MOVING+1).tolist()):
        raise ValueError('Wrong dynamic yaw candidate scope or evidence metadata')
    q=np.asarray(r['goal_ticks']);base=np.asarray(r['base_goal_ticks']);fixed=np.array(FROZEN_IDS)-1
    if (q.shape!=(126,16) or not np.isfinite(q).all() or np.any(q!=np.rint(q))
        or not np.array_equal(base,reached_grasp_goal())):
        raise ValueError('Dynamic yaw must start at the reached shortened grasp')
    if (np.max(abs(np.diff(np.vstack([base,q]),axis=0)))>2 or np.max(abs(q-base))>64
        or not np.array_equal(q[:,fixed],np.tile(base[fixed],(126,1)))
        or not np.array_equal(q[25]-base,r['closure_delta_ticks'])):
        raise ValueError('Dynamic yaw target/step/fixed-joint bound violated')
    limits=json.loads((ROOT/SOURCES[-1]).read_bytes())
    if np.any(q<limits['lower_ticks']) or np.any(q>limits['upper_ticks']):raise ValueError('Dynamic yaw exceeds original native bounds')
    expected={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in SOURCES}
    if r.get('source_hashes')!=expected:raise ValueError('Dynamic yaw evidence changed')
    trial=json.loads((ROOT/SOURCES[0]).read_bytes());baseline=[s for s in trial['samples'] if s['phase']=='stage_4_book_grasp_hold'][-20:]
    normalized=json.loads((ROOT/SOURCES[3]).read_bytes());correction=json.loads((ROOT/SOURCES[2]).read_bytes())
    if (not trial.get('rotation_command_path_completed') or not trial.get('all_torque_off_verified')
        or not trial.get('settings_restored') or len(baseline)!=20
        or baseline[-1]['goal']!=base.tolist()
        or normalized['actual_baseline_ticks']!=np.median([s['actual'] for s in baseline],axis=0).tolist()
        or normalized['source_sha256']!=expected[SOURCES[0]]
        or correction.get('physical_cube_removal_confirmed') is not False
        or correction.get('post_OFF_cube_motion_is_success_evidence') is not False):
        raise ValueError('Measured baseline or removal correction provenance invalid')
    return q.astype(np.int64)


def load_candidate():
    blob=(ROOT/CANDIDATE).read_bytes()
    if hashlib.sha256(blob).hexdigest()!=CANDIDATE_SHA:raise ValueError('Frozen dynamic yaw candidate changed')
    r=json.loads(blob);return r,validate_candidate(r)


def proof_hashes():
    load_candidate()
    return {**grasp_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in (*EXTRA,*SOURCES)}}


class BookDynamicYawPlan(BookSmallYawPlan):
    PLAN_VERSION='book20_measured_dynamic_clockwise_v2'
    TITLE='REVISED SMALL CLOCKWISE: EMPTY recovery -> PLACE -> grasp -> revised turn -> REMOVE -> OFF.'
    FOLDER_PREFIX='book_dynamic_yaw_trial'
    STATUS='book_dynamic_yaw_trial_complete'
    COMPLETE_MESSAGE='Revised small-turn command path complete; operator REMOVE acknowledgement; OFF/settings restored. Physical rotation and removal require observation.'
    CAPTURE_PROTOCOL_TYPE=BookDynamicYawCapture
    proof_hashes=staticmethod(proof_hashes)

    def __init__(self,request_path):
        BookGraspPlan.__init__(self,request_path)
        self.candidate,self.yaw_goals=load_candidate()

    def frozen_for_phase(self,phase):
        if phase==YAW:return np.array(FROZEN_IDS)-1
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase==YAW:return MOVING
        return super().progress_for_phase(phase)
