"""Empty preparation and45s fixed-goal communication observation; no cube path."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as Base,hashes as base_hashes
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import validate_path
from new_pair_grasp_plan_20261027 import ENTRY
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/empty_hold_comm'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json';HOLD_SECONDS=45.
FILES=('empty_hold_comm_plan_20261029.py','empty_hold_comm_gate_20261029.py','leap_empty_hold_comm_20261029.py',
    'screen_empty_hold_comm_20261029.py','check_empty_hold_comm_20261029.py','empty_hold_motion_runner_20261029.py','leap_empty_hold_comm.cmd',
    'leap_retained_slow_yaw_20261029.py','retained_slow_yaw_gate_20261029.py','retained_slow_yaw_plan_20261029.py',
    'fast_health_read_20261029.py','fast_health_window_20261029.py','combined_quiet_reserve_20261029.py')
def hashes():return {**base_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [*(ROOT/n for n in FILES),REQUEST]}}
class EmptyHoldCommPlan(Base):
    PLAN_VERSION='empty_open_hold45s_75ms_comm_v1_20261029'
    TITLE='EMPTY COMM CHECK: preparation -> fixed open pose45s -> OFF. No PLACE or cube movement.'
    FOLDER_PREFIX='empty_hold_comm';STATUS='empty_hold_comm_observation_complete'
    COMPLETE_MESSAGE='빈손 유지 검사 완료. OFF와 원래 설정 복구 결과를 저장했습니다.'
    CAPTURE_BEFORE_PHASE=None;CONTACT_RELEASE_BEFORE_PHASE=None;CAPTURE_PLACEMENT=True;EMPTY_RELEASE_ONLY=True
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        Base.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
    def plan(self,raw):
        a=self.validate_start(raw);rows=prefix_rows(self,a);values=validate_path(a,rows,ENTRY)
        if np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL:raise ValueError('Existing thumb displacement bound exceeded')
        if (len(rows)+20)*self.SAMPLE_PERIOD+HOLD_SECONDS+20>self.ON_TIMEOUT:raise ValueError('Existing300s bound exceeded')
        return rows
def prepare():
    parser=argparse.ArgumentParser();parser.add_argument('--source',default='fast_off_readonly_20261029_041356_739556/result.json');a=parser.parse_args()
    source=(ROOT/a.source).resolve();source.relative_to(ROOT);r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and r['servo_writes']==0 and r['torque_changes']==0
    OUT.mkdir(parents=True,exist_ok=True)
    REQUEST.write_text(json.dumps(dict(plan_version=EmptyHoldCommPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
        off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    p=EmptyHoldCommPlan(REQUEST);rows=p.plan(p.source_position());print(json.dumps(dict(prefix_goals=len(rows),nominal_motion_and_hold_s=len(rows)*.075+45,devices_opened=False)))
if __name__=='__main__':prepare()
