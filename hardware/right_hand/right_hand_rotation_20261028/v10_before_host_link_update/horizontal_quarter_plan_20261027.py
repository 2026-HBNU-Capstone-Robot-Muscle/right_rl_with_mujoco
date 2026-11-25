"""Source-bound operator experiment: EMPTY return, PLACE, yaw, OPEN, REMOVE.

Targets are a fixed reviewed recording. There is no online policy or device
access in this module. Only the user-started operator program can send them.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from new_pair_grasp_plan_20261027 import NewPairGraspPlan,ENTRY,ROOTS,OPEN,SETTLE,GRASP,HOLD,FILES as BASE_FILES,OPEN_PATH
from observed_book_model_policy import admission,healthy_off
from loaded_quarter_reentry_20261025 import validate_path
from horizontal_empty_recovery_20261027 import serial_roots,RING_CLEAR
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/horizontal_quarter_trial_v10_recovery_episode'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json'
DATA_FOLDER=ROOT/'right_hand_rotation_20261027/yaw90_clearance_t8_early_root16'
LOADED_PATH=DATA_FOLDER/'full_loaded_27/loaded_path.npz'
LOADED_REVIEW=DATA_FOLDER/'full_loaded_27/review.json'
STATIC_REVIEW=DATA_FOLDER/'static_201/review.json'
YAW='stage_7_horizontal_yaw';YAW_HOLD='stage_8_horizontal_hold';REOPEN='stage_9_loaded_open';REMOVE='stage_10_remove_hold'
PREFIX_PHASES=(ENTRY,RING_CLEAR,ROOTS,OPEN,SETTLE)
PHASE_MAP={'grasp':GRASP,'grasp_hold':HOLD,'yaw':YAW,'yaw_hold':YAW_HOLD,'release':REOPEN,'remove_hold':REMOVE}
EXTRA_FILES=('horizontal_quarter_plan_20261027.py','horizontal_quarter_gate_20261027.py','horizontal_quarter_capture_20261027.py','leap_horizontal_quarter_20261027.py','leap_horizontal_quarter_trial.cmd','screen_horizontal_quarter_20261027.py','screen_current_yaw_recovery_20261027.py','screen_loaded_yaw_path_20261027.py','screen_loaded_yaw_static_20261027.py','horizontal_quarter_motion_runner_20261027.py','horizontal_preload_confirmation_20261027.py','horizontal_goal_confirmation_20261027.py','horizontal_empty_recovery_20261027.py','horizontal_quarter_health_20261027.py','screen_horizontal_empty_recovery_20261027.py')

EXTRA_FILES+=('rewired_initial_pair_scope_20261027.py','horizontal_thumb_book_numeric_20261027.py')
from horizontal_placement_evidence_20261027 import FILES as PLACEMENT_FILES
EXTRA_FILES+=('horizontal_placement_settle_20261027.py','horizontal_placement_evidence_20261027.py',
    'verify_placement_bound_geometry_20261027.py','screen_weaker_roots_loaded_20261027.py')+PLACEMENT_FILES
EXTRA_FILES+=('horizontal_receive_resync_20261027.py',)
EXTRA_FILES+=('horizontal_sync_receive_window_20261028.py',)
EXTRA_FILES+=('horizontal_read_recovery_20261028.py',)

def hashes():
    names=tuple(dict.fromkeys(BASE_FILES+EXTRA_FILES))
    files=[ROOT/n for n in names]+[LOADED_PATH,LOADED_REVIEW,STATIC_REVIEW,OPEN_PATH]
    return {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}

def loaded_rows():
    z=np.load(LOADED_PATH)
    if not np.array_equal(z['start'],np.load(OPEN_PATH)['open_goal']) or not np.array_equal(z['goals'][-1],z['start']):raise ValueError('Loaded cycle does not join the reviewed OPEN pose')
    return [(PHASE_MAP[str(p)],q.copy()) for p,q in zip(z['phases'],z['goals'])]

def prefix_digest(plan):
    rows=[(p,q.tolist()) for p,q in plan.plan(plan.source_position()) if p in PREFIX_PHASES]
    return hashlib.sha256(json.dumps(rows,separators=(',',':')).encode()).hexdigest()

class HorizontalQuarterPlan(NewPairGraspPlan):
    PLAN_VERSION='reviewed_horizontal_quarter_experiment_v10_recovery_episode_20261028'
    TITLE='HORIZONTAL QUARTER-TURN v10: bounded READ episode; EMPTY -> PLACE/SPACE -> grasp -> large turn -> OPEN -> REMOVE/SPACE -> OFF.'
    STATUS='horizontal_quarter_observation_complete'
    COMPLETE_MESSAGE='Large horizontal turn observation complete; cube removal confirmed; torque OFF/settings restored. Actual cube angle needs video review.'
    FOLDER_PREFIX='horizontal_quarter_trial';ON_TIMEOUT=300
    MODEL_LIMITS_DESCRIPTION='same configured native goals; existing passive ID4 OFF reentry only toward native range; all health stops unchanged'
    validate_admission=staticmethod(admission);validate_off_state=staticmethod(healthy_off);proof_hashes=staticmethod(hashes)
    def plan(self,raw):
        prefix=serial_roots(self.validate_start(raw),[(p,q) for p,q in super().plan(raw) if p in PREFIX_PHASES])
        rows=prefix+loaded_rows();a=self.validate_start(raw);values=validate_path(a,rows,ENTRY)
        if (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise ValueError('Reviewed large turn exceeds its bounded active time')
        if np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL:raise ValueError('Thumb travel exceeds existing bound')
        return rows
    def frozen_for_phase(self,phase):
        if phase==RING_CLEAR:return np.array([i for i in range(16) if i not in (7,11,13,15)])
        if phase in (YAW,REOPEN):return np.array([],int)
        if phase in (YAW_HOLD,REMOVE):return np.arange(16)
        return super().frozen_for_phase(phase)
    def progress_for_phase(self,phase):
        if phase==RING_CLEAR:return np.array([13])
        if phase in (YAW,REOPEN):return np.arange(16)
        if phase in (YAW_HOLD,REMOVE):return np.array([],int)
        return super().progress_for_phase(phase)
