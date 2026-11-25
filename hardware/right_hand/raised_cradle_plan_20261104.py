"""Supervised full-reference observation with a prepared natural support.

Model predicts about65degrees, not a certified90degree result. No inverse or
automatic OPEN is sent after the turn. Operational limits remain unchanged.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from single_forward_plan_20261030 import SingleForwardPlan,RawBase
from four_reply_yaw_plan_20261029 import hashes as transport_hashes
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from calibrated_quarter_plan_20261028 import PREFIX_PHASES
from loaded_quarter_reentry_20261025 import limits,validate_path
from new_pair_grasp_plan_20261027 import ENTRY,OPEN,SETTLE
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261104/raised_cradle_160_yaw0';LOADED=OUT/'candidate.npz';REQUEST=OUT/'request.json';PROOF=OUT/'operator_review.json';SEED=ROOT/'right_hand_rotation_20261030/stable_support_research/natural_recess_hypothesis_seed.json'
FILES=('raised_cradle_plan_20261104.py','raised_cradle_gate_20261104.py','raised_cradle_capture_20261104.py','leap_raised_cradle_turn_20261104.py','leap_raised_cradle_turn.cmd','screen_raised_cradle_current_20261104.py','screen_raised_cradle_hand_20261104.py','screen_yaw_roundtrip_model_20261030.py')
FIXED=np.array([4,5,6,7,12,13,14,15]);ACTIVE=np.array([0,1,2,3,8,9,10,11]);ALLOWED=('stage_5_loaded_grasp','stage_6_grasp_hold','stage_7_horizontal_yaw','stage_8_horizontal_hold')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def interpolate(a,b):
 n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)
def hashes():
 paths=[*(ROOT/n for n in FILES),LOADED,REQUEST,SEED,OUT/'loaded_hand_dynamics/review.json',*(OUT/f'material_front_{f}/review.json' for f in (-15,-10,-5))];return {**transport_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in paths}}
def validate_candidate():
 z=np.load(LOADED);q=z['goals'];ph=z['phases'];lo,hi=limits()
 if q.shape!=(1470,16) or not np.isfinite(q).all() or np.any(q!=np.rint(q)) or np.any(q<lo) or np.any(q>hi):raise RuntimeError('Raised cradle candidate/native bounds changed')
 if list(dict.fromkeys(ph.tolist()))!=list(ALLOWED) or max(abs(np.diff(np.vstack([z['start'],q]),axis=0)).ravel())>2:raise RuntimeError('Unexpected phase or goal step')
 if not np.all(q[:,FIXED]==z['start'][FIXED]):raise RuntimeError('Fixed index/ring commands changed')
 if not np.all(q[-67:]==q[-1]) or not np.all(ph[-67:]=='stage_8_horizontal_hold'):raise RuntimeError('Final5s fixed hold missing')
 return z
class RaisedCradlePlan(SingleForwardPlan):
 PLAN_VERSION='raised_natural_support_full_reference_observation_v1_20261104';TITLE='EMPTY 준비 -> 자연 PLACE -> 기존 전체 정회전/5초 관찰 -> 받치고 SPACE OFF. 90도 미검증.'
 FOLDER_PREFIX='raised_cradle_turn';STATUS='raised_cradle_turn_observation_complete';COMPLETE_MESSAGE='회전 관찰과 토크 OFF 및 설정 복구를 기록했습니다. 큐브를 제거하세요. 90도 성공은 영상에서 확인합니다.'
 CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE=ALLOWED[0];CONTACT_RELEASE_BEFORE_PHASE=None;EMPTY_RELEASE_ONLY=False;proof_hashes=staticmethod(hashes)
 def __init__(self,request):RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
 def plan(self,raw):
  a=self.validate_start(raw);z=validate_candidate();prefix=prefix_rows(self,a);moving=[(p,q) for p,q in prefix if p!=SETTLE];previous=moving[-1][1];moving.extend((OPEN,q) for q in interpolate(previous,z['start']));moving.extend((SETTLE,z['start'].copy()) for _ in range(20));rows=moving+[(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])];values=validate_path(a,rows,ENTRY)
  if max(abs(values[:,:4]-a[:4]).ravel())>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise RuntimeError('Existing thumb travel/time bound exceeded')
  return rows
 def frozen_for_phase(self,phase):
  if phase in (ALLOWED[0],ALLOWED[2]):return FIXED
  if phase in (ALLOWED[1],ALLOWED[3]):return np.arange(16)
  return super().frozen_for_phase(phase)
 def progress_for_phase(self,phase):
  if phase in (ALLOWED[0],ALLOWED[2]):return ACTIVE
  if phase in (ALLOWED[1],ALLOWED[3]):return np.array([],int)
  return super().progress_for_phase(phase)
def prepare():
 source=ROOT/'resume_off_readonly_20261104_180154_900670/result.json';r=json.loads(source.read_bytes());assert len(r['snapshots'])==3 and r['maximum_drift_ticks']<=2 and r['all_torque_off_read'] and not r['servo_writes'] and not r.get('pose_health_validation_errors')
 if REQUEST.exists():raise RuntimeError('Do not overwrite a frozen request')
 REQUEST.write_text(json.dumps(dict(plan_version=RaisedCradlePlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8');p=RaisedCradlePlan(REQUEST);rows=p.plan(p.source_position());print(json.dumps(dict(rows=len(rows),seconds=len(rows)*.075,motor_writes=0)))
if __name__=='__main__':prepare()
