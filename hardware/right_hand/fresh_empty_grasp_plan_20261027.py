"""Pure queue for the currently reviewed EMPTY recovery and grasp posture."""
import hashlib,json
from pathlib import Path
import numpy as np
from thumb_live_start_plan import CurrentPlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from loaded_quarter_reentry_20261025 import limits
ROOT=Path(__file__).resolve().parent
CANONICAL=ROOT/'right_hand_rotation_20261026/fresh_empty_start_20261026_235258'
PATH_FILE=CANONICAL/'recovery_plan_v2_ring_first/path.npz'
PROOF=CANONICAL/'empty_execution_review_v7.json'
REQUEST=CANONICAL/'empty_execution_request_v7.json'
RING_PHASE='stage_0_ring_release';ROOT_PHASE='stage_1_support_release';JOIN='stage_2_clear_join';RECOVER='stage_3_empty_recovery';GRASP='stage_4_empty_grasp';HOLD='stage_5_empty_hold'
FILES=('fresh_empty_grasp_plan_20261027.py','screen_fresh_empty_grasp_20261027.py','fresh_empty_grasp_gate_20261027.py','leap_fresh_empty_grasp_20261027.py','leap_fresh_empty_grasp.cmd','empty_precise_voltage_20261027.py','sequence_flip_checked.py','leap_thumb_prefix_empty.py','leap_thumb_prepare_current_empty.py','camera_gap_current_runtime_20261026.py','camera_gap_motion_runner_20261026.py','recovering_empty_camera_20261026.py','empty_camera_gap_hold_20261026.py','goal_aware_progress_guard_20261025.py','held_joint_guard.py','goal_readback_confirmation.py','read_recovery_confirmation.py','loaded_repeat_voltage_20261026.py','hand_motor_mapping.py','fingertip_cad_model_20261025.py','book_support_model_20261024.py','loaded_quarter_joint_limits_20261025.py','loaded_quarter_id4_lower_setting_20261025.json','book_passive_support_reentry_policy.py','book_grasp_trial_plan.py','book_support_contact_scope.py','book_support_escape_scope_20261024.json')
def hashes():return {**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES},PATH_FILE.relative_to(ROOT).as_posix():hashlib.sha256(PATH_FILE.read_bytes()).hexdigest()}
def interpolate(a,b,step=2):
    if step not in (1,1.5,2):raise ValueError('Unreviewed interpolation step')
    n=int(np.ceil(np.max(abs(b-a))/step))
    return [] if n==0 else list(np.rint(a+(b-a)*np.arange(1,n+1)[:,None]/n).astype(int))
def canonical_start():return np.load(PATH_FILE)['start'].copy()
def admission(raw):
    a=np.asarray(raw);lo,hi=limits()
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)) or np.any(a<lo) or np.any(a>hi):raise ValueError('Fresh angles outside configured ranges; no writes')
    return a.astype(int)
def healthy_off(s):
    a=admission(s['position'])
    if any(np.any(s[k]) for k in ('torque','error','watchdog')) or np.any(np.asarray(s['voltage'])<38) or np.any(np.asarray(s['voltage'])>65) or np.any(np.asarray(s['temperature'])>45):raise ValueError('Unhealthy OFF state; no writes')
    return a
class FreshEmptyGraspPlan(CurrentPlan):
    PLAN_VERSION='fresh_empty_dual_ring_thumb_first_slow_roots_v7_20261027'
    TITLE='EMPTY GRASP POSTURE CHECK: ring tip + middle release / small thumb clearance -> slow roots -> grasp pose -> OFF.'
    COMPLETE_MESSAGE='Empty grasp posture complete. Torque OFF/settings restored. Cube rotation was not run.'
    FOLDER_PREFIX='fresh_empty_grasp';STATUS='fresh_empty_grasp_complete'
    ON_TIMEOUT=180;SAMPLE_PERIOD=.075;MAX_GOAL_STEP=2;MAX_THUMB_TRAVEL=1300
    ROOT_INITIAL_AVERAGE_STEP=1.5;ROOT_INITIAL_TRAVEL=120;ROOT_TAIL_GOAL_STEP=1
    HELD_JOINT_GUARD=True;CONFIRM_PREVIOUS_GOAL_READBACK=True;CONFIRM_FRESH_AFTER_READ_RECOVERY=True
    CAMERA_GAP_HOLD=True;CAPTURE_PLACEMENT=False;EMPTY_ONLY=True;FRESH_EMPTY_GRASP_MODE=True
    MODEL_LIMITS_DESCRIPTION='user ID4 lower -2deg only; all other native ranges and existing health stops'
    validate_admission=staticmethod(admission);validate_off_state=staticmethod(healthy_off);proof_hashes=staticmethod(hashes)
    def plan(self,raw):
        a=self.validate_start(raw);p=np.load(PATH_FILE);ring=a.copy();ring[[12,14]]-=80;ring[3]+=16;rows=[(RING_PHASE,q) for q in interpolate(a,ring)];root=ring.copy();root[[7,11]]=p['root_end'][[7,11]];prefix=ring.copy();delta=root[[7,11]]-ring[[7,11]];prefix[[7,11]]+=np.sign(delta)*np.minimum(abs(delta),self.ROOT_INITIAL_TRAVEL);rows.extend((ROOT_PHASE,q) for q in interpolate(ring,prefix,self.ROOT_INITIAL_AVERAGE_STEP));rows.extend((ROOT_PHASE,q) for q in interpolate(prefix,root,self.ROOT_TAIL_GOAL_STEP))
        # Put the thumb safely inside its existing native limit promptly, so
        # a fixed-goal camera pause early in release can hold this same pose.
        for i,(phase,q) in enumerate(rows):
            if phase==RING_PHASE:q[3]=int(a[3])+min(16,2*(i+1))
        rows.extend((JOIN,q) for q in interpolate(root,p['root_end']))
        rows.extend((RECOVER,q.copy()) for q in p['goals'][p['phases']=='empty_recovery_to_open'])
        rows.extend((GRASP,q.copy()) for q in p['goals'][p['phases']=='empty_new_grasp'])
        rows.extend((HOLD,p['grasp_goal'].copy()) for _ in range(27))
        # Index is not a grasping contact in this candidate. Avoid the backward
        # root tilt that grazed the forward book-front hypothesis.
        for phase,q in rows:
            if phase not in (RING_PHASE,ROOT_PHASE):q[7]=max(int(q[7]),2048)
        values=np.vstack([a]+[q for _,q in rows]);lo,hi=limits()
        if np.max(abs(np.diff(values,axis=0)))>2 or np.any(values<lo) or np.any(values>hi) or np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+10>self.ON_TIMEOUT:raise ValueError('Queue violates its step/range/time bounds')
        return rows
    def frozen_for_phase(self,phase):
        if phase==RING_PHASE:return np.asarray([i for i in range(16) if i not in (3,12,14)])
        if phase==ROOT_PHASE:return np.asarray([i for i in range(16) if i not in (7,11)])
        return np.arange(16) if phase in (HOLD,'stage_2_fixed_hold') else np.array([],int)
    def progress_for_phase(self,phase):
        return np.asarray([3,12,14]) if phase==RING_PHASE else np.asarray([7,11]) if phase==ROOT_PHASE else np.arange(16) if phase in (JOIN,RECOVER,GRASP) else np.array([],int)
