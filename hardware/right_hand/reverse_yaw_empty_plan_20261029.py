"""Reverse the reached joint cycle, EMPTY only. No inverse cube claim."""
import hashlib,json
from pathlib import Path
import numpy as np
from yaw_load_empty_plan_20261029 import YawLoadEmptyPlan as Base
from four_reply_yaw_plan_20261029 import hashes as base_hashes
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as RawBase,LOADED_PATH
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import validate_path
from new_pair_grasp_plan_20261027 import ENTRY,OPEN_PATH
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from pitch_handoff_plan_20261028 import TRANSFER_SEED as MODEL_GRAVITY_SEED
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/reverse_yaw_empty'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json';HAND_PATH=OUT/'reverse_cycle.npz'
FORWARD_RESULT=ROOT/'adaptive_yaw_loaded_20261029_160451_416563/result.json'
FILES=('reverse_yaw_empty_plan_20261029.py','reverse_yaw_empty_gate_20261029.py','leap_reverse_yaw_empty_20261029.py',
    'leap_reverse_yaw_empty.cmd','screen_reverse_yaw_empty_20261029.py','screen_reverse_yaw_hand_20261029.py',
    'screen_pitch_handoff_loaded_20261028.py','screen_pitch_yaw_join_20261028.py','slow_thumb_seam_resolution_20261026.py')
def hashes():return {**base_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [*(ROOT/f for f in FILES),REQUEST,HAND_PATH,FORWARD_RESULT,MODEL_GRAVITY_SEED]}}
def reverse_rows():
    z=np.load(HAND_PATH);return [(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])]
class ReverseYawEmptyPlan(Base):
    PLAN_VERSION='EMPTY_reverse_original638_joint_cycle_v1_20261029'
    TITLE='EMPTY REVERSE PATH: preparation -> reverse grasp/yaw/open joint cycle -> OFF. CUBE stays OUT.'
    FOLDER_PREFIX='reverse_yaw_empty';STATUS='empty_reverse_yaw_path_complete'
    COMPLETE_MESSAGE='반대방향 빈손 경로 완료. OFF와 원래 설정 복구를 저장했습니다. 큐브 역회전은 아직 검증하지 않았습니다.'
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
    def plan(self,raw):
        start=self.validate_start(raw);rows=prefix_rows(self,start)+reverse_rows();v=validate_path(start,rows,ENTRY)
        if np.max(abs(v[:,:4]-start[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise RuntimeError('Reverse path exceeds original travel/time bounds')
        return rows
def prepare():
    source=ROOT/'fast_off_readonly_20261029_161735_921599/result.json';r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes']
    OUT.mkdir(parents=True,exist_ok=True)
    if REQUEST.exists() or HAND_PATH.exists():raise RuntimeError('Refuse to overwrite reverse request/path')
    REQUEST.write_text(json.dumps(dict(plan_version=ReverseYawEmptyPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
        source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    original=np.load(LOADED_PATH);opened=np.load(OPEN_PATH)['open_goal']
    phases={'remove_hold':'stage_5_loaded_grasp','release':'stage_5_loaded_grasp','yaw_hold':'stage_6_grasp_hold',
        'yaw':'stage_7_horizontal_yaw','grasp_hold':'stage_8_horizontal_hold','grasp':'stage_9_loaded_open'}
    goals=original['goals'][::-1].copy();names=[phases[str(p)] for p in original['phases'][::-1]]
    distance=int(max(abs(opened-goals[-1])));count=max(1,int(np.ceil(distance/2)))
    join=np.rint([goals[-1]+(opened-goals[-1])*i/count for i in range(1,count+1)]).astype(int)
    goals=np.vstack([goals,join,np.repeat(opened[None,:],20,axis=0)])
    names+=['stage_9_loaded_open']*len(join)+['stage_10_remove_hold']*20
    np.savez_compressed(HAND_PATH,start=opened,goals=goals,phases=np.asarray(names),original_cycle_goals=len(original['goals']),additional_open_join_goals=len(join))
    p=ReverseYawEmptyPlan(REQUEST);print(json.dumps(dict(goals=len(p.plan(p.source_position())),reverse_yaw_goals=sum(s=='stage_7_horizontal_yaw' for s,q in reverse_rows()),devices_opened=False)))
if __name__=='__main__':prepare()
