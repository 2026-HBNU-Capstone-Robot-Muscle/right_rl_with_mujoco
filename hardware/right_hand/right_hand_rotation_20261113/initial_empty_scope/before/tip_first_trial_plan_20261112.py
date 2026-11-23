"""Source-bound first turn -> receiving pads -> translation -> supported grip."""
import hashlib
import json
from pathlib import Path
import numpy as np
from forward_hold_plan_20261104 import ForwardHoldPlan,hashes as original_hashes
from single_forward_plan_20261030 import RawBase
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import limits,validate_path
from new_pair_grasp_plan_20261027 import ENTRY

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261112/tip_first_trial_v2'
SOURCE=OUT/'research_candidate.npz'
CANDIDATE=OUT/'candidate.npz';MANIFEST=OUT/'manifest.json';REQUEST=OUT/'request.json';PROOF=OUT/'review.json';APPROVAL=OUT/'operator_scope_approval.json'
RECEIVE='stage_17_middle_upper_withdrawal';RECEIVE_HOLD='stage_18_receiving_hold'
PUSH='stage_19_thumb_middleward';REAR='stage_20_rearward_thumb_contact'
TIPS='stage_21_thumb_middle_tips_first';NEXT='stage_22_thumb_middle_next_links';HOLD='stage_23_tip_first_observation'
ACTIVE={RECEIVE:np.arange(8,12),PUSH:np.arange(4),REAR:np.arange(4),
        TIPS:np.r_[np.arange(4),np.arange(8,12)],NEXT:np.r_[np.arange(4),np.arange(8,12)]}
HOLDS={RECEIVE_HOLD,HOLD}
FILES=('tip_first_trial_plan_20261112.py','tip_first_trial_gate_20261112.py','tip_first_trial_capture_20261112.py',
       'leap_cycle_tip_first_transfer_20261112.py','leap_cycle_tip_first_transfer.cmd','screen_tip_first_current_20261112.py')

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def candidate():
    manifest=json.loads(MANIFEST.read_bytes())
    if sha(CANDIDATE)!=manifest['candidate_sha256']:raise RuntimeError('New transfer candidate changed')
    for name,digest in manifest['source_files_sha256'].items():
        if sha(ROOT/name)!=digest:raise RuntimeError('New transfer source changed: '+name)
    z=dict(np.load(CANDIDATE,allow_pickle=False));source=dict(np.load(SOURCE,allow_pickle=False));q=z['goals'];ph=z['phases']
    lo,hi=limits();allq=np.vstack([z['start'],q])
    count=manifest['loaded_goals']
    if (q.shape!=(count,16) or ph.shape!=(count,) or not np.isfinite(allq).all()
            or np.any(allq!=np.rint(allq)) or np.any(allq<lo) or np.any(allq>hi)
            or np.max(abs(np.diff(allq,axis=0)))>2 or not np.array_equal(q,source['goals']) or not np.array_equal(z['start'],source['start'])):
        raise RuntimeError('Transfer source/range/rate contract changed')
    first=np.load(ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz',allow_pickle=False)
    if not np.array_equal(q[:1417],first['goals']) or not np.array_equal(ph[:1417],first['phases']):
        raise RuntimeError('Physically observed first turn changed')
    if ph.tolist()!=manifest['phases']:raise RuntimeError('Transfer phase boundaries changed')
    for phase in HOLDS:
        held=q[ph==phase]
        if len(held)!=54 or not np.all(held==held[0]):raise RuntimeError('Transfer hold changed')
    if set(ph[1417:]) != set(ACTIVE)|HOLDS:raise RuntimeError('Lower-grip trial phase coverage changed')
    prior=z['start']
    for phase,command in zip(ph,q):
        if phase in ACTIVE and np.any(command[np.setdiff1d(np.arange(16),ACTIVE[phase])]!=prior[np.setdiff1d(np.arange(16),ACTIVE[phase])]):
            raise RuntimeError('Finger outside reviewed active group moved')
        prior=command
    return z

def scope_approved():
    if not APPROVAL.exists():return False
    a=json.loads(APPROVAL.read_bytes());old=ROOT/a['inherited_approval_path']
    if sha(old)!=a['inherited_approval_sha256']:return False
    approved=json.loads(old.read_bytes())
    return bool(a.get('maximum_on_seconds')==500 and a.get('candidate_sha256')==sha(CANDIDATE)
                and a.get('operator_SPACE_required') is True and approved.get('user_explicitly_approved') is True
                and approved.get('maximum_on_seconds')==500)

def hashes():
    paths=[*(ROOT/name for name in FILES),CANDIDATE,MANIFEST,REQUEST,APPROVAL]
    paths+=sorted((OUT/'models').glob('*.json'))
    return {**original_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in paths}}

class TipFirstTrialPlan(ForwardHoldPlan):
    PLAN_VERSION='observed_first_turn_rearward_thumb_tip_first_then_next_link_transfer_probe_v2_20261112'
    TITLE='끝마디 우선 이동 시험: 첫 회전 -> 중지 윗마디 빼기 -> 엄지 이송 -> 뒤쪽 접촉 -> 끝마디 -> 다음 마디 -> 유지. 약지 닫힘·다음 회전 미포함.'
    FOLDER_PREFIX='tip_first_trial';STATUS='tip_first_transfer_observation_complete'
    COMPLETE_MESSAGE='새 파지 전환 관찰 종료. 실제 지지와 공간은 저장 영상으로 검토합니다.'
    ON_TIMEOUT=500;proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
    def plan(self,raw):
        a=self.validate_start(raw);z=candidate();rows=prefix_rows(self,a)+[(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])]
        values=validate_path(a,rows,ENTRY)
        if np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*.075+20>self.ON_TIMEOUT:
            raise RuntimeError('Existing travel/approved time bound exceeded')
        return rows
    def frozen_for_phase(self,phase):
        if phase in ACTIVE:return np.setdiff1d(np.arange(16),ACTIVE[phase])
        if phase in HOLDS:return np.arange(16)
        return super().frozen_for_phase(phase)
    def progress_for_phase(self,phase):
        if phase in ACTIVE:return ACTIVE[phase]
        if phase in HOLDS:return np.array([],dtype=int)
        return super().progress_for_phase(phase)
