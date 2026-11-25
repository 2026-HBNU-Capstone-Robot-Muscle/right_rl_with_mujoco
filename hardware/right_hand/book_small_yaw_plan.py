"""One frozen, bounded clockwise experiment after the actually reached grasp.

The active-cube search is exploratory prescribed-hand simulation, not a physical
rotation certificate or a learned policy. Its residuals never enter motor goals.
"""
import hashlib
import json
import numpy as np
from book_grasp_trial_plan import BookGraspPlan,ROOT,reached_grasp_goal,extra_hashes as grasp_hashes
from book_small_yaw_capture import BookSmallYawCapture

CANDIDATE='right_hand_rotation_20261024/book_grasp_yaw_diagnostic/book_small_yaw_candidate.json'
CANDIDATE_SHA='7a1e6b87e916901fb0270ac294209a5bba871e5d83ca2e0b33be708ed504d857'
BASELINE='stage_4_book_grasp_hold'
YAW='stage_5_book_small_clockwise'
AFTER='stage_6_book_yaw_hold'
FROZEN_IDS=(4,8,12,13,14,15,16)
MOVING=np.array([i for i in range(16) if i+1 not in FROZEN_IDS])
SOURCES=('book_cube_grasp_trial_20261024_212855_824047/result.json',
    'right_hand_rotation_20261024/book_grasp_yaw_diagnostic/contact_search_clockwise.json',
    'right_hand_rotation_20261024/book_grasp_yaw_diagnostic/relative_candidate_replay.json',
    'right_hand_rotation_20261023/native_motor_limits.json',
    'search_book_contact_yaw.py','replay_book_relative_yaw_candidate.py')
EXTRA=('book_small_yaw_plan.py','book_small_yaw_capture.py','book_small_yaw_gate.py',
    'screen_book_small_yaw.py','leap_book_small_yaw_trial.py','leap_book_small_yaw_trial.cmd',CANDIDATE)


def validate_candidate(r):
    flags={'experimental_trial_only':True,'physical_cube_rotation_verified':False,
        'live_sac':False,'robust_policy':False,'full_three_degree_amplitude_verified':False,
        'marker_to_model_registration_verified':False,'cube_friction_measured':False,
        'following_error_used_only_in_offline_simulation':True,'fitted_joint_zero_offsets':False,
        'current_or_PID_changed':False,'hardware_execution_allowed':False}
    if (r.get('version')!='book_completed_grasp_clockwise_candidate_v1'
        or r.get('desired_physical_yaw_degrees')!=-3.
        or r.get('motion_direction')!='clockwise_viewed_from_above'
        or any(r.get(k) is not v for k,v in flags.items())
        or r.get('closure_samples')!=20 or r.get('rotation_samples')!=70):
        raise ValueError('Wrong experimental yaw candidate metadata')
    base=np.asarray(r['base_goal_ticks']);q=np.asarray(r['goal_ticks'])
    if (base.shape!=(16,) or q.shape!=(90,16) or not np.isfinite(q).all()
        or np.any(q!=np.rint(q)) or not np.array_equal(base,reached_grasp_goal())):
        raise ValueError('Yaw must start at the completed shortened grasp')
    frozen=np.array(FROZEN_IDS)-1
    if (np.max(abs(np.diff(np.vstack([base,q]),axis=0)))>2
        or np.max(abs(q-base))>64 or np.max(abs(q[:20]-base))>40
        or not np.array_equal(q[:,frozen],np.tile(base[frozen],(90,1)))
        or not np.array_equal(q[19]-base,r['closure_delta_ticks'])):
        raise ValueError('Yaw exceeds bounded contact/turn goals or moves a fixed root/ring joint')
    limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
    if np.any(q<limits['lower_ticks']) or np.any(q>limits['upper_ticks']):
        raise ValueError('Yaw goals outside original native limits')
    expected={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in SOURCES}
    if r.get('source_hashes')!=expected:raise ValueError('Yaw source evidence changed')
    completed=json.loads((ROOT/SOURCES[0]).read_bytes())
    if (completed.get('status')!='book_cube_grasp_trial_complete'
        or not completed.get('all_torque_off_verified') or not completed.get('settings_restored')
        or not completed.get('cube_placement_confirmed') or not completed.get('cube_removal_confirmed')
        or completed.get('grasp_goal_ticks')!=base.tolist()):
        raise ValueError('Yaw base grasp was not completed and removed')
    return q.astype(np.int64)


def load_candidate():
    data=(ROOT/CANDIDATE).read_bytes()
    if hashlib.sha256(data).hexdigest()!=CANDIDATE_SHA:raise ValueError('Frozen yaw candidate changed')
    r=json.loads(data)
    return r,validate_candidate(r)


def proof_hashes():
    load_candidate()
    return {**grasp_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in (*EXTRA,*SOURCES)}}


class BookSmallYawPlan(BookGraspPlan):
    PLAN_VERSION='book20_completed_grasp_small_clockwise_v1'
    TITLE='SMALL CLOCKWISE TRIAL: EMPTY recovery -> PLACE -> reached grasp -> small turn -> REMOVE -> OFF.'
    FOLDER_PREFIX='book_small_yaw_trial'
    STATUS='book_small_yaw_trial_complete'
    COMPLETE_MESSAGE='Small-turn command path completed; removal confirmed; OFF/settings restored. Physical cube rotation awaits video review.'
    ON_TIMEOUT=90
    ROTATION_INCLUDED=True
    CAPTURE_PROTOCOL_TYPE=BookSmallYawCapture
    proof_hashes=staticmethod(proof_hashes)

    def __init__(self,request_path):
        super().__init__(request_path)
        self.candidate,self.yaw_goals=load_candidate()

    def plan(self,raw):
        rows=super().plan(raw)
        base=rows[-1][1]
        rows.extend((BASELINE,base.copy()) for _ in range(60))
        rows.extend((YAW,q.copy()) for q in self.yaw_goals)
        rows.extend((AFTER,self.yaw_goals[-1].copy()) for _ in range(40))
        if (len(rows)+40)*.06+2>self.ON_TIMEOUT:
            raise ValueError('Combined small-turn path exceeds its delayed-cycle 90s budget')
        return rows

    def frozen_for_phase(self,phase):
        if phase==YAW:return np.array(FROZEN_IDS)-1
        if phase in (BASELINE,AFTER):return np.arange(16)
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase==YAW:return MOVING
        if phase in (BASELINE,AFTER):return np.array([],dtype=int)
        return super().progress_for_phase(phase)
