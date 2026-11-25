"""Fresh OFF recovery to the actual stop, then the unexecuted EMPTY suffix."""
import hashlib,json
import numpy as np
from reverse_quarter_path_empty_plan_20261025 import ReverseQuarterEmptyPlan,ROOT,START_HOLD,TURN_PHASE,END_HOLD,extra_hashes as previous_hashes
from reverse_place_capture_plan_20261025 import CASING_PAIR,CASING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE
from book_grasp_trial_plan import RING_RELEASE
from book_empty_preparation_plan import LIFT
from grasp_recovery_plan import interpolate

GOALS='right_hand_rotation_20261024/reverse_quarter_resume_empty/resume_goals_v1.json'
EXTRA_RING_PAIR='palm_collision_8|rf_tip'
EXTRA_RING_INITIAL_LIMIT_MM=1.5
INITIAL_SUPPORT_SCOPE='OFF_4_tick_fixed_enable_hold_nominal_contact_origin_v2'
FILES=('reverse_quarter_resume_empty_plan_20261025.py','reverse_quarter_resume_empty_gate_20261025.py',
    'screen_reverse_quarter_resume_empty_20261025.py','leap_reverse_quarter_resume_empty_20261025.py',
    'quarter_motion_runner_20261025.py','quarter_current_runtime_20261025.py','goal_aware_progress_guard_20261025.py',GOALS)

def extra_hashes():return {**previous_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}}

def resume_goals():
    meta=json.loads((ROOT/GOALS).read_bytes());a=np.asarray(meta['raw_goal_ticks'])
    source=ROOT/meta['source_actual_trial'].replace('\\','/');blob=source.read_bytes();actual=json.loads(blob)
    if hashlib.sha256(blob).hexdigest()!=meta['source_actual_trial_sha256'] or not actual['all_torque_off_verified'] or not actual['settings_restored']:
        raise ValueError('Interrupted actual record changed')
    last=actual['samples'][-1]
    if last['goal']!=last['goal_readback'] or last['goal']!=meta['verified_last_issued_goal'] or not np.array_equal(a[0],last['goal']):
        raise ValueError('Resume does not match the verified last command')
    full_path=ROOT/meta['source_full_goals'].replace('\\','/');blob=full_path.read_bytes();full=np.asarray(json.loads(blob)['raw_goal_ticks'])
    if hashlib.sha256(blob).hexdigest()!=meta['source_full_goals_sha256'] or not np.array_equal(a,full[meta['resumed_original_waypoint']:]):
        raise ValueError('Resume is not the exact unexecuted suffix')
    if a.ndim!=2 or a.shape[1]!=16 or not np.isfinite(a).all() or np.any(a!=np.rint(a)) or np.max(abs(np.diff(a,axis=0)))>2:
        raise ValueError('Invalid continuation targets')
    return a.astype(np.int64)

class ReverseQuarterResumeEmptyPlan(ReverseQuarterEmptyPlan):
    PLAN_VERSION='interrupted_quarter_EMPTY_suffix_goal_aware_progress_v2_20261025'
    TITLE='EMPTY CONTINUATION: measured release/re-entry -> remaining 14.4s reference -> OFF. NO CUBE.'
    COMPLETE_MESSAGE='Remaining EMPTY reference completed. OFF/settings restored. Physical Cube rotation was NOT tested.'
    FOLDER_PREFIX='reverse_quarter_resume_empty'
    STATUS='reverse_quarter_resume_empty_complete'
    proof_hashes=staticmethod(extra_hashes)

    def plan(self,raw):
        a=self.validate_start(raw);motion=resume_goals();join=motion[0]
        release=a.copy();release[14]-=64
        roots=np.array([7,11]);release[roots]+=np.minimum(64,np.maximum(0,join[roots]-a[roots]))
        self.validate_admission(release)
        rows=[];interpolate(rows,a,release,RING_RELEASE);interpolate(rows,release,join,LIFT)
        rows.extend((START_HOLD,join.copy()) for _ in range(40))
        rows.extend((TURN_PHASE,q.copy()) for q in motion[1:]);rows.extend((END_HOLD,motion[-1].copy()) for _ in range(20))
        values=np.vstack([a]+[q for p,q in rows]);limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
        if np.max(abs(np.diff(values,axis=0)))>2 or np.any(values<limits['lower_ticks']) or np.any(values>limits['upper_ticks']):
            raise ValueError('Recovery/continuation violates native bounds or step limit')
        if (len(rows)+20)*.06+2>self.ON_TIMEOUT:raise ValueError('Continuation exceeds bounded time budget')
        return rows

    def frozen_for_phase(self,phase):
        if phase==RING_RELEASE:return np.array([i for i in range(16) if i not in (7,11,14)])
        if phase==LIFT:return np.array([],dtype=int)
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase==RING_RELEASE:return np.array([7,11,14])
        if phase==LIFT:return np.arange(16)
        return super().progress_for_phase(phase)
