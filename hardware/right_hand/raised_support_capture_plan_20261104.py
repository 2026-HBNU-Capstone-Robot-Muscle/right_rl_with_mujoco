"""EMPTY raised support followed only by static placement observation."""
import json
import numpy as np
from raised_cradle_plan_20261104 import RaisedCradlePlan,ROOT,OUT as TURN_OUT,validate_candidate,sha
from calibrated_quarter_plan_20261028 import PREFIX_PHASES
from four_reply_yaw_plan_20261029 import hashes as transport_hashes
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from loaded_quarter_reentry_20261025 import validate_path
from new_pair_grasp_plan_20261027 import ENTRY
OUT=ROOT/'right_hand_rotation_20261104/raised_support_capture';REQUEST=OUT/'request.json';PROOF=OUT/'review.json';FILES=('raised_support_capture_plan_20261104.py','raised_support_capture_gate_20261104.py','raised_support_capture_protocol_20261104.py','leap_raised_support_capture_20261104.py','leap_raised_support_capture.cmd','screen_raised_support_capture_current_20261104.py','raised_cradle_plan_20261104.py')
def hashes():
 paths=[*(ROOT/n for n in FILES),REQUEST,TURN_OUT/'candidate.npz',TURN_OUT/'empty_recovery_screen/review.json',TURN_OUT/'empty_recovery_screen/prefix_path.npz'];return {**transport_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in paths}}
class RaisedSupportCapturePlan(RaisedCradlePlan):
 PLAN_VERSION='raised_support_static_placement_capture_only_v1_20261104';TITLE='빈손 받침 준비 -> 자연 PLACE/2초 촬영 -> REMOVE/SPACE OFF. 회전과 잡기 없음.';FOLDER_PREFIX='raised_support_capture';STATUS='raised_support_static_observation_complete';COMPLETE_MESSAGE='새 받침의 정지 배치를 저장했습니다. 제거 확인과 토크OFF/설정복구를 기록했습니다. 회전은 실행하지 않았습니다.';proof_hashes=staticmethod(hashes)
 def plan(self,raw):
  prefix=[(p,q) for p,q in super().plan(raw) if p in PREFIX_PHASES];goal=validate_candidate()['start'];rows=prefix+[('stage_5_loaded_grasp',goal.copy()) for _ in range(27)];validate_path(np.asarray(raw),rows,ENTRY)
  if not np.all(np.asarray([q for p,q in rows if p not in PREFIX_PHASES])==goal):raise RuntimeError('Static capture attempted changing loaded goal')
  return rows
 def frozen_for_phase(self,phase):return np.arange(16) if phase=='stage_5_loaded_grasp' else super().frozen_for_phase(phase)
 def progress_for_phase(self,phase):return np.array([],int) if phase=='stage_5_loaded_grasp' else super().progress_for_phase(phase)
def prepare():
 OUT.mkdir(exist_ok=True);source=ROOT/'resume_off_readonly_20261104_180154_900670/result.json';r=json.loads(source.read_bytes());assert len(r['snapshots'])==3 and r['maximum_drift_ticks']==0 and r['all_torque_off_read'] and not r.get('pose_health_validation_errors') and not r['servo_writes'] and not r['torque_changes']
 if REQUEST.exists():raise RuntimeError('Do not overwrite a reviewed request')
 REQUEST.write_text(json.dumps(dict(plan_version=RaisedSupportCapturePlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8');p=RaisedSupportCapturePlan(REQUEST);rows=p.plan(p.source_position());print(json.dumps(dict(rows=len(rows),empty_goals=len(rows)-27,static_observation_goals=27,seconds=len(rows)*.075,motor_writes=0)))
if __name__=='__main__':prepare()
