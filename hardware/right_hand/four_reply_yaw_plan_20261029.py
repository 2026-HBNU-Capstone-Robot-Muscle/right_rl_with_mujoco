"""Same completed-speed trajectory; four-device health reply groups only."""
import hashlib,json
from pathlib import Path
import numpy as np
from reference_speed_yaw_plan_20261029 import ReferenceSpeedYawPlan as Base,hashes as base_hashes
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as RawBase
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/four_reply_yaw'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json'
BENCH=ROOT/'fast4_compare_readonly_20261029_051534_614138/result.json'
FILES=('four_reply_yaw_plan_20261029.py','four_reply_yaw_gate_20261029.py','leap_four_reply_yaw_20261029.py',
    'leap_four_reply_yaw.cmd','screen_four_reply_yaw_20261029.py','fast_health_read4_20261029.py','compare_fast4_readonly_20261029.py')
def hashes():return {**base_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [*(ROOT/n for n in FILES),REQUEST,BENCH]}}
class FourReplyYawPlan(Base):
    PLAN_VERSION='completed_speed_yaw_four_reply_groups_v1_20261029'
    TITLE='REFERENCE-SPEED YAW:4-device health replies; EMPTY -> PLACE GREEN up/ORANGE wristward -> yaw -> OPEN -> REMOVE -> OFF.'
    FOLDER_PREFIX='four_reply_yaw';STATUS='four_reply_yaw_observation_complete';proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
def prepare():
    source=ROOT/'fast_off_readonly_20261029_051855_895498/result.json';r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes']
    OUT.mkdir(parents=True,exist_ok=True)
    REQUEST.write_text(json.dumps(dict(plan_version=FourReplyYawPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
        source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    p=FourReplyYawPlan(REQUEST);rows=p.plan(p.source_position());print(json.dumps(dict(goals=len(rows),nominal_motion_seconds=len(rows)*.075,devices_opened=False)))
if __name__=='__main__':prepare()
