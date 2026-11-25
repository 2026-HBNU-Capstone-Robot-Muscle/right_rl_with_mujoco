"""Archived complete-cycle diagnostic; execution requires explicit scope approval."""
import hashlib
import json
from pathlib import Path
import numpy as np
from forward_hold_plan_20261104 import ForwardHoldPlan, hashes as old_hashes
from single_forward_plan_20261030 import RawBase
from calibrated_empty_ring_margin_20261028 import rows as old_prefix, interpolate
from loaded_quarter_reentry_20261025 import limits, validate_path
from new_pair_grasp_plan_20261027 import ENTRY, OPEN, SETTLE

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110'
OUT=ROOT/'right_hand_rotation_20261112/nominal_cycle_trial_preparation'
SOURCE=OUT/'rf16_2177_v2/candidate.npz'
SOURCE_REVIEW=OUT/'rf16_2177_v2/model_review.json'
REQUEST=OUT/'request.json'; PROOF=OUT/'review.json'; MANIFEST=OUT/'manifest.json'; APPROVAL=OUT/'operator_scope_approval.json'
FIRST='stage_13_cycle_first_rotation'; FIRST_HOLD='stage_14_cycle_first_hold'
SECOND='stage_15_cycle_second_rotation'; FINAL_HOLD='stage_16_cycle_final_hold'
FILES=('nominal_cycle_plan_20261112.py','nominal_cycle_gate_20261112.py','nominal_cycle_capture_20261112.py',
       'leap_cycle_nominal_two_turns_20261112.py','leap_cycle_nominal_two_turns.cmd','screen_nominal_cycle_current_20261112.py')

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def candidate():
    manifest=json.loads(MANIFEST.read_bytes()); z=dict(np.load(SOURCE,allow_pickle=False)); r=json.loads(SOURCE_REVIEW.read_bytes())
    if sha(SOURCE)!=manifest['source_goals_sha256'] or sha(SOURCE_REVIEW)!=manifest['source_review_sha256']:
        raise RuntimeError('Complete-cycle source changed')
    if not r['two_rotations_and_clear_holds_at_reached_locations_passed'] or r['actual_state_resets_after_initial_placement']!=0:
        raise RuntimeError('No complete nominal model cycle')
    q=z['goals']; lo,hi=limits(); allq=np.vstack([z['start'],q])
    original=dict(np.load(BASE/'initial_second_turn_hold_replay/research_goals.npz',allow_pickle=False))
    if (r.get('candidate_sha256')!=sha(SOURCE) or not np.array_equal(z['start'],original['start'])
            or not np.array_equal(q[:,:15],original['goals'][:,:15])
            or not np.array_equal(q[:,15],np.minimum(original['goals'][:,15],2177))):
        raise RuntimeError('RF16-only correction contract changed')
    for name,digest in r['source_files_sha256'].items():
        if sha(ROOT/name)!=digest: raise RuntimeError('Corrected cycle provenance changed: '+name)
    if len(q)!=5089 or np.any(allq!=np.rint(allq)) or np.any(allq<lo) or np.any(allq>hi) or np.max(abs(np.diff(allq,axis=0)))>2:
        raise RuntimeError('Native/rate/source scope mismatch')
    phases=np.full(len(q),FIRST,dtype='<U48'); phases[3351:3405]=FIRST_HOLD; phases[3405:-54]=SECOND; phases[-54:]=FINAL_HOLD
    if not np.all(q[3351:3405]==q[3351]) or not np.all(q[-54:]==q[-1]): raise RuntimeError('Archived hold scope mismatch')
    return {**z,'phases':phases}

def scope_approved():
    if not APPROVAL.exists(): return False
    r=json.loads(APPROVAL.read_bytes())
    return bool(r.get('user_explicitly_approved') is True and r.get('maximum_on_seconds')==500
                and r.get('source_goals_sha256')==sha(SOURCE)
                and r.get('acknowledges_nominal_only_and_hand_only_failures') is True)

def hashes():
    files=[*(ROOT/f for f in FILES),SOURCE,SOURCE_REVIEW,REQUEST,MANIFEST]
    files+=sorted(SOURCE.parent.glob('hand_gain*.json'))
    return {**old_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in files}}

class NominalCyclePlan(ForwardHoldPlan):
    PLAN_VERSION='archived_complete_two_turns_supervised_nominal_diagnostic_20261112'
    TITLE='연속 동작 진단 ID16 보정v2: ID16목표 최대2177틱 / 최대500초 / 사용자SPACE 시작.'
    FOLDER_PREFIX='nominal_two_turn_trial'; STATUS='nominal_two_turn_observation_complete'
    COMPLETE_MESSAGE='전체 경로 관찰 종료. OFF/설정 복구 뒤 실제 회전·유지를 저장 영상으로 검토합니다.'
    ON_TIMEOUT=500
    CAPTURE_BEFORE_PHASE=FIRST
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        RawBase.__init__(self,request); self.raw_off_ticks=self.center.copy(); self.review_center_reused=False
    def plan(self,raw):
        a=self.validate_start(raw); z=candidate(); prefix=old_prefix(self,a)
        # All original empty recovery is retained. The archived placement pose
        # differs by at most a few ticks; this join is part of the EMPTY proof.
        prefix=[(p,q) for p,q in prefix if p!=SETTLE]
        prefix += [(OPEN,q) for q in interpolate(prefix[-1][1],z['start'])]
        prefix += [(SETTLE,z['start'].copy()) for _ in range(20)]
        rows=prefix+[(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])]
        values=validate_path(a,rows,ENTRY)
        if np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*.075+20>self.ON_TIMEOUT:
            raise RuntimeError('Bounded travel/time exceeded')
        return rows
    def frozen_for_phase(self,phase):
        if phase in (FIRST,SECOND): return np.array([],dtype=int)
        if phase in (FIRST_HOLD,FINAL_HOLD): return np.arange(16)
        return super().frozen_for_phase(phase)
    def progress_for_phase(self,phase):
        if phase in (FIRST,SECOND): return np.arange(16)
        if phase in (FIRST_HOLD,FINAL_HOLD): return np.array([],dtype=int)
        return super().progress_for_phase(phase)
