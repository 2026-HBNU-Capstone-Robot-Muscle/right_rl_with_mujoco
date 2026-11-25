"""Current OFF recovery -> PLACE -> same reached grasp -> open -> REMOVE/OFF."""
import hashlib,json
from pathlib import Path
import numpy as np
from thumb_live_start_plan import CurrentPlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from loaded_quarter_reentry_20261025 import limits,validate_path
from observed_book_model_policy import healthy_off as observed_off
from fresh_empty_grasp_plan_20261027 import interpolate
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2'
REQUEST=OUT/'request.json';PROOF=OUT/'review_v2.json'
OPEN_PATH=ROOT/'right_hand_rotation_20261027/loaded_grasp_recovery/open_path.npz'
GRASP_PATH=ROOT/'right_hand_rotation_20261026/fresh_empty_start_20261026_235258/recovery_plan_v2_ring_first/path.npz'
SEED=ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json'
ENTRY='stage_0_cmc_return';ROOTS='stage_1_roots_clear';JOIN='stage_2_current_join';OPEN='stage_3_empty_open';SETTLE='stage_4_place_hold';GRASP='stage_5_loaded_grasp';HOLD='stage_6_grasp_hold';REOPEN='stage_7_loaded_open';REMOVE='stage_8_remove_hold'
FILES=('new_pair_grasp_plan_20261027.py','new_pair_grasp_gate_20261027.py','screen_new_pair_grasp_20261027.py','new_pair_grasp_capture_20261027.py','leap_new_pair_grasp_20261027.py','leap_new_pair_grasp.cmd','camera_gap_current_runtime_20261026.py','camera_gap_motion_runner_20261026.py','leap_thumb_prepare_current_empty.py','leap_thumb_prefix_empty.py','sequence_flip_checked.py','loaded_repeat_voltage_20261026.py','read_recovery_confirmation.py','goal_readback_confirmation.py','held_joint_guard.py','goal_aware_progress_guard_20261025.py','empty_trial_camera.py','prepared_placement_capture.py','grasp_place_capture.py','quarter_finish_empty_camera_20261025.py','loaded_quarter_reentry_20261025.py','loaded_quarter_model_policy_20261025.py','loaded_quarter_joint_limits_20261025.py','loaded_quarter_id4_lower_setting_20261025.json','observed_book_model_policy.py','fingertip_cad_model_20261025.py','book_support_model_20261024.py','book_support_contact_scope.py','book_passive_support_reentry_policy.py','sac_cube_model.py','hand_motor_mapping.py')
FILES+=('fresh_empty_grasp_plan_20261027.py','thumb_live_start_plan.py','thumb_live_start_gate.py','thumb_root_off_screen_process_20261026.py','marker_trial_camera.py','camera_frame_signal.py','camera_stream.py','empty_start_marker_check.py','book_support_escape_scope_20261024.json','current_book_support_measurements_20261024.json','sac_wood_glass_model.py','wood_glass_support_measurements.json','right_hand_rotation_20261023/native_motor_limits.json','screen_loaded_repeat_20261026.py','book_empty_preparation_plan.py')
def hashes():return {**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES},**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in (OPEN_PATH,GRASP_PATH,SEED)}}
def loaded_digest(plan):
    # This part starts from the same settled open pose, independently of OFF
    # recovery. Reuse is permitted only with all code/assets still identical.
    rows=plan.plan(plan.source_position())
    payload=dict(open_goal=next(q.tolist() for phase,q in rows if phase==SETTLE),rows=[(phase,q.tolist()) for phase,q in rows if phase in (GRASP,HOLD,REOPEN,REMOVE)])
    return hashlib.sha256(json.dumps(payload,separators=(',',':')).encode()).hexdigest()
def admission(raw):
    a=np.asarray(raw);lo,hi=limits();lower=lo.copy();lower[3]-=32
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)) or np.any(a<lower) or np.any(a>hi):raise ValueError('Current OFF angles exceed the narrow passive recovery envelope; no writes')
    return a.astype(int)
def healthy_off(s):observed_off(s);return admission(s['position'])
class NewPairGraspPlan(CurrentPlan):
    PLAN_VERSION='current_off_direct_open_same_pair_grasp_v2_20261027'
    TITLE='NEW PAIR GRASP v2: direct EMPTY opening -> PLACE/SPACE (5min) -> reached grasp/3s -> automatic OPEN -> REMOVE/SPACE -> OFF.'
    COMPLETE_MESSAGE='Cube approach/open observation completed; removal confirmed; torque OFF/settings restored. Rotation was not run.'
    STATUS='new_pair_grasp_observation_complete';FOLDER_PREFIX='new_pair_grasp';MAX_THUMB_TRAVEL=1300;MAX_GOAL_STEP=2;SAMPLE_PERIOD=.075;ON_TIMEOUT=180
    CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE=GRASP;CHECKPOINT_SETTLE_SAMPLES=10;CHECKPOINT_MAX_ERROR_TICKS=64;PLACEMENT_PAUSE_LIMIT=320
    HELD_JOINT_GUARD=True;CONFIRM_PREVIOUS_GOAL_READBACK=True;CONFIRM_FRESH_AFTER_READ_RECOVERY=True;CAMERA_GAP_HOLD=False;NEW_PAIR_GRASP_MODE=True
    MODEL_LIMITS_DESCRIPTION='existing measured OFF ID4 reentry only toward configured range; other native limits and health stops unchanged'
    validate_admission=staticmethod(admission);validate_off_state=staticmethod(healthy_off);proof_hashes=staticmethod(hashes)
    def plan(self,raw):
        a=self.validate_start(raw);data=np.load(OPEN_PATH);entry=a.copy();entry[3]=max(int(a[3]),1872);root=entry.copy();root[[7,11]]=np.maximum(root[[7,11]],2150);root[15]=max(int(root[15]),2053);prefix=entry.copy();delta=root[[7,11,15]]-entry[[7,11,15]];fraction=min(1.,120/max(1,int(max(delta))));prefix[[7,11,15]]+=np.rint(delta*fraction).astype(int)
        # Go directly from the measured, cleared roots to OPEN. Revisiting a
        # historical grasp/OFF waypoint would curl the hand and undo that
        # movement again, especially after an interrupted PLACE wait.
        rows=[(ENTRY,q) for q in interpolate(a,entry)];rows.extend((ROOTS,q) for q in interpolate(entry,prefix,1.5));rows.extend((ROOTS,q) for q in interpolate(prefix,root,1));opened=data['open_goal'];rows.extend((OPEN,q) for q in interpolate(root,opened));rows.extend((SETTLE,opened.copy()) for _ in range(20));z=np.load(GRASP_PATH);close=z['goals'][z['phases']=='empty_new_grasp'].copy();close[:,7]=np.maximum(close[:,7],2048);rows.extend((GRASP,q.copy()) for q in close);rows.extend((HOLD,close[-1].copy()) for _ in range(40));rows.extend((REOPEN,q.copy()) for q in close[-2::-1]);rows.extend((REOPEN,q) for q in interpolate(close[0],opened));rows.extend((REMOVE,opened.copy()) for _ in range(20));values=validate_path(a,rows,ENTRY)
        if np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+10>self.ON_TIMEOUT:raise ValueError('Grasp cycle exceeds bounded travel/time; no writes')
        return rows
    def frozen_for_phase(self,phase):
        if phase==ENTRY:return np.array([i for i in range(16) if i!=3])
        if phase==ROOTS:return np.array([i for i in range(16) if i not in (7,11,15)])
        if phase in (GRASP,REOPEN):return np.array([4,5,6,7,12,13,14,15])
        return np.arange(16) if phase in (SETTLE,HOLD,REMOVE,'placement_checkpoint','stage_2_fixed_hold') else np.array([],int)
    def progress_for_phase(self,phase):
        return np.array([3]) if phase==ENTRY else np.array([7,11,15]) if phase==ROOTS else np.array([0,1,2,3,8,9,10,11]) if phase in (GRASP,REOPEN) else np.arange(16) if phase in (JOIN,OPEN) else np.array([],int)
