"""Supervised cube experiment using the completed EMPTY motor targets."""
import hashlib,json
import numpy as np
from quarter_finish_empty_plan_20261025 import (QuarterFinishEmptyPlan,ROOT,extra_hashes as empty_hashes,
    TURN_PHASE as EMPTY_TURN,FINISH_PHASE as EMPTY_FINISH,END_HOLD as EMPTY_END,
    START_HOLD,RETURN_PHASE,motion_goals,empty_return_goals,finish_goals,
    EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE,THUMB_NEIGHBOR_PAIR,
    THUMB_NEIGHBOR_LIMIT_MM,INITIAL_SELF_SCOPE)
TURN_PHASE='stage_6_loaded_quarter_reference';FINISH_PHASE='stage_7_loaded_root_finish';END_HOLD='stage_8_loaded_observation'
FIXTURE=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/loaded_finish_fixture_v2'
ACTUAL=ROOT/'quarter_finish_empty_20261025_234555_682505/result.json'
REPLAY=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/level_book_seating/measured_empty_234555_mu03/seating_grid.json'
FILES=('loaded_quarter_finish_plan_20261025.py','loaded_quarter_finish_gate_20261025.py',
    'screen_loaded_quarter_finish_20261025.py','leap_loaded_quarter_finish_trial_20261025.py',
    'loaded_quarter_finish_protocol_20261025.py','quarter_finish_empty_camera_20261025.py',
    'leap_quarter_finish_empty_preview_fixed_20261025.py')
def extra_hashes():return {**empty_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}}
def loaded_basis():
    blob=ACTUAL.read_bytes();r=json.loads(blob);expected='7a5cbeead95766bd396ee1d5112b7eae48bf7bd344df355bbe0e8a81b46a51e4'
    if (hashlib.sha256(blob).hexdigest()!=expected or r['status']!='quarter_finish_empty_complete'
            or not r['all_torque_off_verified'] or not r['settings_restored'] or not r['current_start_screen_accepted']):raise ValueError('Completed EMPTY evidence changed')
    for phase,goals in ((EMPTY_TURN,motion_goals()),(EMPTY_FINISH,finish_goals())):
        s=[v for v in r['samples'] if v['phase']==phase]
        if not np.array_equal([v['goal'] for v in s],goals[:-1]) or any(v['goal']!=v['goal_readback'] for v in s):raise ValueError('EMPTY accepted target/readback sequence changed')
    if r['last_issued_command']!=finish_goals()[-1].tolist() or r['samples'][-1]['goal_readback']!=finish_goals()[-1].tolist():raise ValueError('EMPTY final command/readback missing')
    replay_blob=REPLAY.read_bytes();replay=json.loads(replay_blob);cases=replay['cases']
    if (replay['source_sha256']!=expected or len(cases)!=9 or not all(c.get('candidate_accepted') for c in cases)
            or replay['hardware_execution_allowed'] or not replay['book_frame_level_is_a_setup_hypothesis']
            or replay['hypothetical_contact_time']!=.004 or replay['physical_friction_identified']):raise ValueError('Conditional replay is incomplete or wrongly claimed as hardware calibration')
    return dict(actual_empty_result_sha256=expected,actual_empty_reference_goals=760,actual_empty_finish_goals=64,
        last_empty_goal_ticks=finish_goals()[-1].tolist(),conditional_actual_empty_replay_sha256=hashlib.sha256(replay_blob).hexdigest(),
        conditional_cases=9,conditional_orientation_retention_passes=9,max_hold_goal_error_deg=max(c['hold_error_deg'] for c in cases),
        prescribed_loaded_hand_motion_is_an_assumption=True,contact_stiffness_and_friction_not_identified=True,
        actual_cube_hand_registration_verified=False,physical_cube_rotation_verified=False,supervised_experimental_loaded_trial_only=True)
class LoadedQuarterFinishPlan(QuarterFinishEmptyPlan):
    PLAN_VERSION='supervised_loaded_quarter_plus_verified_finish_measured_MCP_release_v2_20261026'
    TITLE='CUBE TURN WITH NEW FINISH: EMPTY recovery -> PLACE/SPACE -> 41.2s turn -> REMOVE/SPACE -> OFF.'
    COMPLETE_MESSAGE='Loaded reference and new finish ended; removal confirmed; OFF/settings restored. Cube orientation awaits review.'
    FOLDER_PREFIX='loaded_quarter_finish_trial';STATUS='loaded_quarter_finish_trial_complete'
    CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE=TURN_PHASE;ROTATION_INCLUDED=True
    proof_hashes=staticmethod(extra_hashes)
    def plan(self,raw):
        aliases={EMPTY_TURN:TURN_PHASE,EMPTY_FINISH:FINISH_PHASE,EMPTY_END:END_HOLD}
        return [(aliases.get(p,p),q.copy()) for p,q in super().plan(raw)]
    def frozen_for_phase(self,phase):
        aliases={TURN_PHASE:EMPTY_TURN,FINISH_PHASE:EMPTY_FINISH,END_HOLD:EMPTY_END}
        return super().frozen_for_phase(aliases.get(phase,phase))
    def progress_for_phase(self,phase):
        aliases={TURN_PHASE:EMPTY_TURN,FINISH_PHASE:EMPTY_FINISH,END_HOLD:EMPTY_END}
        return super().progress_for_phase(aliases.get(phase,phase))
