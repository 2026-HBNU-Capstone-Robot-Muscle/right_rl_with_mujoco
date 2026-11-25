"""Exact observed forward route followed by fixed hold; no OPEN or inverse."""
import hashlib,json
from pathlib import Path
import numpy as np
from single_forward_plan_20261030 import SingleForwardPlan,RawBase,validate_candidate as original_candidate,LOADED as ORIGINAL
from four_reply_yaw_plan_20261029 import hashes as transport_hashes
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import validate_path,limits
from new_pair_grasp_plan_20261027 import ENTRY
from hand_motor_mapping import SIM_TO_MOTOR_IDS

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261104/forward_hold';LOADED=OUT/'candidate.npz';REQUEST=OUT/'request.json';PROOF=OUT/'review.json'
HOLD_SAMPLES=134
FILES=('forward_hold_plan_20261104.py','forward_hold_gate_20261104.py','forward_hold_capture_20261104.py','leap_forward_hold_observe_20261104.py','leap_forward_hold_observe.cmd','screen_forward_hold_current_20261104.py','screen_forward_hold_hand_20261104.py','screen_yaw_roundtrip_model_20261030.py','verified_off_restore_context_20261105.py')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def validate_candidate():
    source=original_candidate();z=np.load(LOADED);cut=int(np.flatnonzero(source['phases']=='stage_8_horizontal_hold')[0]);last=source['goals'][cut]
    expected=np.vstack([source['goals'][:cut],np.tile(last,(HOLD_SAMPLES,1))]);phases=np.r_[source['phases'][:cut],['stage_8_horizontal_hold']*HOLD_SAMPLES]
    if not np.array_equal(z['start'],source['start']) or not np.array_equal(z['goals'],expected) or not np.array_equal(z['phases'],phases):raise RuntimeError('Forward route differs from physically observed source or fixed hold')
    lo,hi=limits();values=np.vstack([z['start'],z['goals']])
    if np.any(values<lo) or np.any(values>hi) or max(abs(np.diff(values,axis=0)).ravel())>2:raise RuntimeError('Original native/step bounds exceeded')
    return z
def hashes():
    paths=[*(ROOT/n for n in FILES),ORIGINAL,LOADED,REQUEST,OUT/'loaded_hand_dynamics/review.json',*(OUT/f'material_front_{f}/review.json' for f in (-15,-10,-5))]
    return {**transport_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in paths}}
class ForwardHoldPlan(SingleForwardPlan):
    PLAN_VERSION='same_observed_forward_fixed10s_no_open_v1_20261104'
    TITLE='빈손 복귀 -> PLACE -> 기존 정회전 -> 같은 목표10초 -> 받치고 SPACE OFF. 자동OPEN/역회전 없음.'
    FOLDER_PREFIX='forward_hold_observe';STATUS='forward_hold_observation_complete'
    COMPLETE_MESSAGE='기존 정회전과10초 고정 관찰 완료. 토크 OFF/설정 복구 후 큐브를 제거하세요. 실제 회전과 파지는 영상 검토가 필요합니다.'
    proof_hashes=staticmethod(hashes)
    CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE='stage_5_loaded_grasp';CONTACT_RELEASE_BEFORE_PHASE=None;EMPTY_RELEASE_ONLY=False
    def __init__(self,request):
        RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            reference=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],int)
            if max(abs(self.raw_off_ticks-reference))<=4:self.center=reference;self.review_center_reused=True
    def plan(self,raw):
        start=self.validate_start(raw);z=validate_candidate();rows=prefix_rows(self,start)+[(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])];values=validate_path(start,rows,ENTRY)
        if max(abs(values[:,:4]-start[:4]).ravel())>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise RuntimeError('Existing travel/time bound exceeded')
        return rows
def prepare():
    OUT.mkdir(parents=True,exist_ok=True);source=original_candidate();cut=int(np.flatnonzero(source['phases']=='stage_8_horizontal_hold')[0]);goals=np.vstack([source['goals'][:cut],np.tile(source['goals'][cut],(HOLD_SAMPLES,1))]);phases=np.r_[source['phases'][:cut],['stage_8_horizontal_hold']*HOLD_SAMPLES]
    if REQUEST.exists() or LOADED.exists():raise RuntimeError('Refuse to overwrite frozen forward hold request/candidate')
    np.savez_compressed(LOADED,start=source['start'],goals=goals,phases=phases,forward_end_sample=len(goals))
    path=ROOT/'resume_off_readonly_20261104_205710_008253/result.json';r=json.loads(path.read_bytes())
    if len(r['snapshots'])!=3 or r['maximum_drift_ticks']>2 or not r['all_torque_off_read'] or r['servo_writes'] or r.get('pose_health_validation_errors'):raise RuntimeError('Fresh OFF source rejected')
    REQUEST.write_text(json.dumps(dict(plan_version=ForwardHoldPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=path.relative_to(ROOT).as_posix(),source_sha256=sha(path),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    p=ForwardHoldPlan(REQUEST);rows=p.plan(p.source_position());print(json.dumps(dict(goals=len(goals),whole_goals=len(rows),seconds=len(rows)*.075,new_joint_vectors=0,automatic_open=False,automatic_inverse=False,hold_seconds=HOLD_SAMPLES*.075,devices_opened=False)))
if __name__=='__main__':prepare()
