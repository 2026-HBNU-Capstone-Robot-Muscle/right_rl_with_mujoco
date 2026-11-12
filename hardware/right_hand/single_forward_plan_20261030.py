"""The exact completed forward reference with current OFF recovery."""
import json,hashlib
from pathlib import Path
import numpy as np
from four_reply_yaw_plan_20261029 import FourReplyYawPlan as Base,hashes as base_hashes
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as RawBase,LOADED_PATH as REFERENCE
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import validate_path
from new_pair_grasp_plan_20261027 import ENTRY
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261030/single_forward';REQUEST=OUT/'request.json';PROOF=OUT/'review.json';LOADED=OUT/'candidate.npz';HAND_PATH=OUT/'whole_hand_path.npz'
MODEL_SOURCES=[OUT/f'full_loaded_seated_front_{f}/review.json' for f in (-15,-10,-5)]
FILES=('single_forward_plan_20261030.py','single_forward_gate_20261030.py','single_forward_capture_20261030.py','leap_single_forward_trial_20261030.py','leap_single_forward_trial.cmd','screen_single_forward_20261030.py','screen_single_forward_hand_20261030.py','screen_yaw_roundtrip_model_20261030.py')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes():
 paths=[*(ROOT/n for n in FILES),REQUEST,LOADED,HAND_PATH,OUT/'proposal.json',OUT/'loaded_hand_dynamics/review.json',*MODEL_SOURCES]
 return {**base_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in paths}}
def validate_candidate():
 z=np.load(LOADED);s=np.load(REFERENCE);mapping={'grasp':'stage_5_loaded_grasp','grasp_hold':'stage_6_grasp_hold','yaw':'stage_7_horizontal_yaw','yaw_hold':'stage_8_horizontal_hold','release':'stage_9_loaded_open','remove_hold':'stage_10_remove_hold'}
 expected=np.array([mapping[str(p)] for p in s['phases']])
 if len(z['goals'])!=2032 or not np.array_equal(z['start'],s['start']) or not np.array_equal(z['goals'],s['goals']) or not np.array_equal(z['phases'],expected):raise RuntimeError('Single forward differs from completed physical reference, or inverse inserted')
 h=np.load(HAND_PATH)
 if not np.array_equal(h['start'],z['start']) or not np.array_equal(h['goals'],z['goals']):raise RuntimeError('Empty dynamics path differs from single forward')
 return z
class SingleForwardPlan(Base):
 PLAN_VERSION='completed_single_forward_with_strict_seating_v1_20261030'
 TITLE='SINGLE FORWARD: EMPTY -> PLACE GREEN up/ORANGE wristward -> same638 yaw -> automatic OPEN -> REMOVE/OFF.'
 FOLDER_PREFIX='single_forward_trial';STATUS='single_forward_observation_complete'
 COMPLETE_MESSAGE='정회전 한 동작 완료. 큐브 제거와 토크OFF 및 설정 복구를 기록했습니다. 실제 각도는 영상에서 확인합니다.'
 proof_hashes=staticmethod(hashes)
 CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE='stage_5_loaded_grasp';CONTACT_RELEASE_BEFORE_PHASE=None;EMPTY_RELEASE_ONLY=False
 def __init__(self,request):
  RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
  if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
   center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],int)
   if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
 def plan(self,raw):
  start=self.validate_start(raw);z=validate_candidate();rows=prefix_rows(self,start)+[(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])];values=validate_path(start,rows,ENTRY)
  if np.max(abs(values[:,:4]-start[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise RuntimeError('Unchanged travel/time bound exceeded')
  return rows
def prepare():
 src=ROOT/'fast_off_readonly_20261030_044532_258003/result.json';r=json.loads(src.read_bytes())
 assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes']
 if REQUEST.exists():raise RuntimeError('Frozen single-forward request already exists')
 REQUEST.write_text(json.dumps(dict(plan_version=SingleForwardPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=src.relative_to(ROOT).as_posix(),source_sha256=sha(src),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
 p=SingleForwardPlan(REQUEST);rows=p.plan(p.source_position());print(json.dumps(dict(whole_goals=len(rows),nominal_motion_s=len(rows)*.075,new_goal_vectors=0,devices_opened=False)))
if __name__=='__main__':prepare()
