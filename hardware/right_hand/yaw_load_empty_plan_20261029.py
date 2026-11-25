"""Same reached grasp and first320 yaw goals, EMPTY throughout, then OFF."""
import hashlib,json
from pathlib import Path
import numpy as np
from four_reply_yaw_plan_20261029 import FourReplyYawPlan as Base,hashes as base_hashes
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as RawBase,loaded_rows
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import validate_path
from new_pair_grasp_plan_20261027 import ENTRY
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/yaw_load_empty'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json';HAND_PATH=OUT/'hand_prefix.npz';YAW_GOALS=320
FILES=('yaw_load_empty_plan_20261029.py','yaw_load_empty_gate_20261029.py','leap_yaw_load_empty_20261029.py',
    'leap_yaw_load_empty.cmd','screen_yaw_load_empty_20261029.py','screen_yaw_load_hand_20261029.py')
def motion_prefix():
    rows=[];count=0
    for phase,q in loaded_rows():
        if phase=='stage_7_horizontal_yaw':
            if count==YAW_GOALS:break
            count+=1
        rows.append((phase,q.copy()))
    if count!=YAW_GOALS:raise RuntimeError('Original yaw prefix is incomplete')
    return rows
def hashes():return {**base_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [*(ROOT/n for n in FILES),REQUEST,HAND_PATH]}}
class YawLoadEmptyPlan(Base):
    PLAN_VERSION='EMPTY_same_grasp_yaw320_load_comparison_v1_20261029'
    TITLE='EMPTY LOAD COMPARISON: preparation -> same grasp/yaw prefix320 -> fixed1.5s -> OFF. NO CUBE or PLACE.'
    FOLDER_PREFIX='yaw_load_empty';STATUS='empty_yaw_load_comparison_complete';COMPLETE_MESSAGE='빈손 부하 비교 완료. OFF와 원래 설정 복구를 저장했습니다.'
    CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE=None;CONTACT_RELEASE_BEFORE_PHASE=None;EMPTY_RELEASE_ONLY=True
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
    def plan(self,raw):
        start=self.validate_start(raw);rows=prefix_rows(self,start)+motion_prefix();values=validate_path(start,rows,ENTRY)
        if np.max(abs(values[:,:4]-start[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise RuntimeError('Original travel/time bound exceeded')
        return rows
def prepare():
    source=ROOT/'fast_off_readonly_20261029_053634_849768/result.json';r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes']
    OUT.mkdir(parents=True,exist_ok=True)
    REQUEST.write_text(json.dumps(dict(plan_version=YawLoadEmptyPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
        source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    p=YawLoadEmptyPlan(REQUEST);rows=motion_prefix()
    from new_pair_grasp_plan_20261027 import OPEN_PATH
    np.savez_compressed(HAND_PATH,start=np.load(OPEN_PATH)['open_goal'],goals=np.asarray([q for phase,q in rows]),phases=np.asarray([phase for phase,q in rows]))
    print(json.dumps(dict(goals=len(p.plan(p.source_position())),yaw_goals=YAW_GOALS,nominal_motion_seconds=len(p.plan(p.source_position()))*.075,devices_opened=False)))
if __name__=='__main__':prepare()
