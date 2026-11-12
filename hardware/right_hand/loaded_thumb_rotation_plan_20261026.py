"""Supervised loaded version of the physically completed thumb/root sequence."""
import hashlib,json
import numpy as np
from thumb_clear_root_empty_plan_20261026 import (ThumbRootEmptyPlan,ROOT,extra_hashes as parent_hashes,
    TURN_PHASE as EMPTY_TURN,FINISH_PHASE as EMPTY_FINISH,END_HOLD as EMPTY_END,
    THUMB_LIFT as EMPTY_LIFT,THUMB_CLEAR as EMPTY_CLEAR,ROOT_UP as EMPTY_ROOT,
    tail_goals as empty_tail,finish_goals,motion_goals,RETURN_PHASE,
    proposal_metadata as empty_proposal,loaded_basis as parent_basis)
TURN_PHASE='stage_6_loaded_quarter_reference'
FINISH_PHASE='stage_7_loaded_known_root_finish'
THUMB_LIFT='stage_8a_loaded_thumb_lift_clearance'
THUMB_CLEAR='stage_9_loaded_thumb_retreat'
ROOT_UP='stage_10_loaded_index_middle_lift'
END_HOLD='stage_11_loaded_new_pose_hold'
RENAMES={EMPTY_TURN:TURN_PHASE,EMPTY_FINISH:FINISH_PHASE,EMPTY_LIFT:THUMB_LIFT,
    EMPTY_CLEAR:THUMB_CLEAR,EMPTY_ROOT:ROOT_UP,EMPTY_END:END_HOLD}
REVERSE={v:k for k,v in RENAMES.items()}
from loaded_thumb_return_clearance_20261026 import goals as empty_return_goals,metadata as return_metadata
from book_grasp_trial_plan import RING_RELEASE
from loaded_quarter_reentry_20261025 import validate_path
FIXTURE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/loaded_thumb_root_fixture_v4_portable_proof'
ACTUAL=ROOT/'thumb_clear_root_empty_20261026_125659_859144/result.json'
COMPLETION=ACTUAL.with_name('completion_review.json')
REPLAY=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/measured_thumb_125659_cube_replay/review.json'
FILES=('loaded_thumb_rotation_plan_20261026.py','loaded_thumb_rest_policy_20261026.py',
    'loaded_thumb_rotation_gate_20261026.py','screen_loaded_thumb_rotation_20261026.py',
    'loaded_thumb_rotation_camera_20261026.py','loaded_thumb_rotation_protocol_20261026.py',
    'loaded_thumb_rotation_runtime_20261026.py','loaded_thumb_voltage_sample_20261026.py',
    'leap_loaded_thumb_rotation_20261026.py','leap_loaded_thumb_rotation.cmd',
    'loaded_thumb_return_clearance_20261026.py','loaded_thumb_return_model_resolution_20261026.py')
def extra_hashes():
    return {**parent_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES},
        ACTUAL.relative_to(ROOT).as_posix():hashlib.sha256(ACTUAL.read_bytes()).hexdigest(),
        REPLAY.relative_to(ROOT).as_posix():hashlib.sha256(REPLAY.read_bytes()).hexdigest(),
        COMPLETION.relative_to(ROOT).as_posix():hashlib.sha256(COMPLETION.read_bytes()).hexdigest()}
def tail_goals():return [(RENAMES.get(phase,phase),q.copy()) for phase,q in empty_tail()]
def final_goal():return tail_goals()[-1][1].copy()
def proposal_metadata():
    r=empty_proposal();r.update(empty_only=False,supervised_loaded_experiment=True,physically_completed_empty_source=ACTUAL.parent.name,
                              empty_return_clearance=return_metadata())
    return r
def loaded_basis():
    r=json.loads(ACTUAL.read_bytes());replay=json.loads(REPLAY.read_bytes());completion=json.loads(COMPLETION.read_bytes())
    if (completion.get('accepted') is not True or not completion.get('exact_ordered_goal_queue')
        or not completion.get('exact_saved_goal_readbacks')
        or completion.get('source_sha256')!=hashlib.sha256(ACTUAL.read_bytes()).hexdigest()):
        raise ValueError('Completed EMPTY goal-queue evidence is missing or changed')
    if (r['status']!='thumb_clear_root_empty_complete' or not r['all_torque_off_verified'] or not r['settings_restored']
        or replay['source_sha256']!=hashlib.sha256(ACTUAL.read_bytes()).hexdigest()
        or len(replay['cases'])!=9 or not all(v['candidate_accepted'] for v in replay['cases'])
        or replay['hardware_execution_allowed'] or replay['physical_rotation_verified']):raise ValueError('Incomplete measured EMPTY/cube evidence')
    basis=parent_basis()
    basis.update(new_empty_source_sha256=hashlib.sha256(ACTUAL.read_bytes()).hexdigest(),
        measured_cube_replay_sha256=hashlib.sha256(REPLAY.read_bytes()).hexdigest(),
        conditional_cube_cases=9,physical_cube_rotation_verified=False,
        loaded_motor_response_verified=False,cube_placement_not_registered=True,
        hand_motion_in_cube_replay_prescribed=True,friction_and_electrical_model_not_identified=True,
        empty_return_clearance=return_metadata())
    return basis
class LoadedThumbRotationPlan(ThumbRootEmptyPlan):
    PLAN_VERSION='supervised_loaded_physically_completed_thumb_root_v4_portable_proof_20261026'
    ON_TIMEOUT=210
    TITLE='LOADED THUMB/ROOT TURN: EMPTY recovery -> PLACE/SPACE -> measured-path turn -> REMOVE/SPACE -> OFF.'
    COMPLETE_MESSAGE='Loaded reference and new thumb/root ending complete; removal confirmed; OFF/settings restored. Actual cube rotation awaits video review.'
    FOLDER_PREFIX='loaded_thumb_rotation';STATUS='loaded_thumb_rotation_complete'
    CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE=TURN_PHASE;ROTATION_INCLUDED=True
    CAMERA_GAP_HOLD=False;CAMERA_GAP_HOLD_EARLY_EMPTY=True
    IMMEDIATE_HEALTHY_VOLTAGE_RECHECK=True;SUPERVISED_LOADED_THUMB_MODE=True
    proof_hashes=staticmethod(extra_hashes)
    def plan(self,raw):
        base=super().plan(raw);original=motion_goals()
        anchor=int(np.argmin(np.max(abs(original-np.asarray(raw)),axis=1)))
        replacement=empty_return_goals(anchor)[1:];rows=[];inserted=False
        for phase,q in base:
            if phase==RETURN_PHASE:
                if not inserted:rows.extend((phase,v.copy()) for v in replacement);inserted=True
            else:rows.append((RENAMES.get(phase,phase),q.copy()))
        if len(replacement) and not inserted:raise ValueError('Missing EMPTY return block')
        validate_path(np.asarray(raw),rows,RING_RELEASE)
        if (len(rows)+40)*.085+2>self.ON_TIMEOUT:raise ValueError('Loaded trial exceeds bounded motion allowance')
        return rows
    def frozen_for_phase(self,phase):return super().frozen_for_phase(REVERSE.get(phase,phase))
    def progress_for_phase(self,phase):return super().progress_for_phase(REVERSE.get(phase,phase))
