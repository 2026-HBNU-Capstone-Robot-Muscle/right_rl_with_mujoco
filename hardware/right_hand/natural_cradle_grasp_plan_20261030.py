"""Natural root-recess grasp/hold/OPEN, with fresh OFF recovery. No rotation."""
import hashlib,json,shutil
from pathlib import Path
import numpy as np
from single_forward_plan_20261030 import SingleForwardPlan,RawBase
from four_reply_yaw_plan_20261029 import hashes as transport_hashes
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from calibrated_quarter_plan_20261028 import PREFIX_PHASES
from new_pair_grasp_plan_20261027 import OPEN_PATH,ENTRY
from loaded_quarter_reentry_20261025 import validate_path,limits
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261030/natural_cradle_grasp';REQUEST=OUT/'request.json';PROOF=OUT/'review.json';LOADED=OUT/'candidate.npz'
RESEARCH=ROOT/'right_hand_rotation_20261030/stable_support_research';SEED=RESEARCH/'natural_recess_hypothesis_seed.json';SOURCE=ROOT/'fast_off_readonly_20261030_102710_504377/result.json'
FIXED=np.array([4,5,6,7,12,13,14,15]);ACTIVE=np.array([0,1,2,3,8,9,10,11]);ALLOWED=('stage_5_loaded_grasp','stage_6_grasp_hold','stage_9_loaded_open','stage_10_remove_hold')
FILES=('natural_cradle_grasp_plan_20261030.py','natural_cradle_grasp_gate_20261030.py','natural_cradle_grasp_capture_20261030.py','leap_natural_cradle_grasp_20261030.py','leap_natural_cradle_grasp.cmd','screen_natural_cradle_grasp_current_20261030.py','screen_natural_cradle_grasp_hand_20261030.py','screen_natural_grasp_hold_20261030.py')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes():
 paths=[*(ROOT/n for n in FILES),REQUEST,LOADED,SEED,RESEARCH/'natural_placement_measurement.json',RESEARCH/'retargeted_grasp_native/geometry_review.json',OUT/'loaded_hand_dynamics/review.json',*(OUT/f'free_front_{f}/review.json' for f in (-15,-10,-5))]
 return {**transport_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in paths}}
def validate_candidate():
 z=np.load(LOADED);start=np.load(OPEN_PATH)['open_goal'];q=z['goals'];p=z['phases'];sequence=list(dict.fromkeys(p.tolist()));lo,hi=limits()
 if q.shape!=(1808,16) or not np.isfinite(q).all() or np.any(q!=np.rint(q)) or np.any(q<lo) or np.any(q>hi):raise RuntimeError('Natural grasp goal shape/native range changed')
 if not np.array_equal(z['start'],start) or not np.array_equal(q[-1],start) or sequence!=list(ALLOWED) or len(q)!=1808:raise RuntimeError('Natural grasp scope/path changed')
 if not np.all(q[:,FIXED]==start[FIXED]):raise RuntimeError('Fixed index/ring targets changed')
 h=q[p==ALLOWED[1]]
 if len(h)!=67 or not np.all(h==h[0]) or np.sum(p==ALLOWED[3])!=20:raise RuntimeError('Five-second grip hold or final OPEN hold changed')
 if np.max(abs(np.diff(np.vstack([start,q]),axis=0)))>2:raise RuntimeError('Natural grasp exceeds 2-tick limit')
 return z
class NaturalCradleGraspPlan(SingleForwardPlan):
 PLAN_VERSION='natural_cradle_grasp_hold_open_no_rotation_v1_20261030'
 TITLE='자연 홈 파지: EMPTY 준비 -> 자연 PLACE -> 새 접근/5초 고정 -> 자동 OPEN -> REMOVE/OFF. 회전 없음.'
 FOLDER_PREFIX='natural_cradle_grasp';STATUS='natural_cradle_grasp_observation_complete'
 COMPLETE_MESSAGE='자연 홈 파지/고정/열기 관찰 완료. 큐브 제거와 토크 OFF 및 설정 복구를 기록했습니다. 회전은 실행하지 않았습니다.'
 CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE=ALLOWED[0];CONTACT_RELEASE_BEFORE_PHASE=None;EMPTY_RELEASE_ONLY=False
 proof_hashes=staticmethod(hashes)
 def __init__(self,request):
  RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
 def plan(self,raw):
  a=self.validate_start(raw);z=validate_candidate();rows=prefix_rows(self,a)+[(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])];v=validate_path(a,rows,ENTRY)
  if np.max(abs(v[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise RuntimeError('Original thumb travel/time limit exceeded')
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
 OUT.mkdir(exist_ok=True);r=json.loads(SOURCE.read_bytes());assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes'];assert not REQUEST.exists() and not LOADED.exists()
 shutil.copyfile(RESEARCH/'retargeted_grasp_native/grasp_hold_release.npz',LOADED);REQUEST.write_text(json.dumps(dict(plan_version=NaturalCradleGraspPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=sha(SOURCE),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8');p=NaturalCradleGraspPlan(REQUEST);rows=p.plan(p.source_position());print(json.dumps(dict(whole_goals=len(rows),nominal_motion_s=len(rows)*.075,thumb_travel_bound=p.MAX_THUMB_TRAVEL,rotation_goals=0,devices_opened=False)))
if __name__=='__main__':prepare()
