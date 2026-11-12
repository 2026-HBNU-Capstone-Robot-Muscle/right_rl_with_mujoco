"""Only the reviewed EMPTY recovery to OPEN. No grasp or rotation targets."""
import json,hashlib
from pathlib import Path
import numpy as np
from single_forward_plan_20261030 import SingleForwardPlan,RawBase
from four_reply_yaw_plan_20261029 import hashes as transport_hashes
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from calibrated_quarter_plan_20261028 import PREFIX_PHASES
from new_pair_grasp_plan_20261027 import OPEN_PATH,ENTRY
from loaded_quarter_reentry_20261025 import validate_path
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261030/placement_alignment_capture';REQUEST=OUT/'request.json';PROOF=OUT/'review.json'
REFERENCE_RESULT=ROOT/'single_forward_trial_20261030_052636_068402/result.json'
REFERENCE_IMAGE=REFERENCE_RESULT.with_name('cube_before_grasp.jpg')
FILES=('placement_alignment_plan_20261030.py','placement_alignment_gate_20261030.py','placement_alignment_capture_20261030.py','placement_alignment_camera_20261030.py','screen_placement_alignment_20261030.py','leap_placement_alignment_20261030.py','leap_placement_alignment.cmd')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes():return {**transport_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in [*(ROOT/n for n in FILES),REQUEST,REFERENCE_RESULT,REFERENCE_IMAGE]}}
def validate_scope(plan,raw):
 rows=plan.plan(raw);opened=np.load(OPEN_PATH)['open_goal']
 if not rows or any(phase not in PREFIX_PHASES for phase,q in rows) or not np.array_equal(rows[-1][1],opened):raise RuntimeError('Placement-only scope includes an unknown motion or misses OPEN')
 return rows
class PlacementAlignmentPlan(SingleForwardPlan):
 PLAN_VERSION='placement_alignment_photo_only_v1_20261030'
 TITLE='배치 비교 촬영: 빈손 준비 -> PLACE/사진 -> 큐브 제거 -> OFF. 잡기와 회전 지령 없음.'
 FOLDER_PREFIX='placement_alignment_capture';STATUS='placement_alignment_photo_complete'
 COMPLETE_MESSAGE='배치 비교 촬영과 큐브 제거 확인 완료. 토크OFF와 설정 복구를 기록했습니다.'
 CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE=None;CONTACT_RELEASE_BEFORE_PHASE=None;EMPTY_RELEASE_ONLY=False
 proof_hashes=staticmethod(hashes)
 def __init__(self,request):
  RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
 def plan(self,raw):
  a=self.validate_start(raw);rows=prefix_rows(self,a);v=validate_path(a,rows,ENTRY)
  if np.max(abs(v[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise RuntimeError('Original travel/time bound exceeded')
  return rows
def prepare(source):
 src=Path(source).resolve();src.relative_to(ROOT);r=json.loads(src.read_bytes());assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes'];OUT.mkdir(exist_ok=True)
 if REQUEST.exists():raise RuntimeError('Refuse to overwrite the fixed alignment request')
 REQUEST.write_text(json.dumps(dict(plan_version=PlacementAlignmentPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=src.relative_to(ROOT).as_posix(),source_sha256=sha(src),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
 p=PlacementAlignmentPlan(REQUEST);rows=validate_scope(p,p.source_position());reference=json.loads(REFERENCE_RESULT.read_bytes());assert reference['all_torque_off_verified'] and reference['settings_restored'] and reference['placement_goal_ticks']==rows[-1][1].tolist();print(json.dumps(dict(goals=len(rows),nominal_empty_motion_s=len(rows)*p.SAMPLE_PERIOD,reference=str(REFERENCE_IMAGE),no_grasp_or_turn_targets=True,devices_opened=False)))
if __name__=='__main__':
 import argparse
 a=argparse.ArgumentParser();a.add_argument('--source',required=True);prepare(a.parse_args().source)
