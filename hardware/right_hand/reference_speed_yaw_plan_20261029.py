"""Reproduce the actually completed75ms yaw route and GREEN-up start."""
import hashlib,json
from pathlib import Path
import numpy as np
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as Base,hashes as base_hashes,LOADED_PATH,loaded_rows
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/reference_speed_yaw'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json'
REFERENCE=ROOT/'calibrated_quarter_trial_20261028_174723_390729/result.json'
FILES=('reference_speed_yaw_plan_20261029.py','reference_speed_yaw_gate_20261029.py',
    'leap_reference_speed_yaw_20261029.py','leap_reference_speed_yaw.cmd','screen_reference_speed_yaw_20261029.py',
    'leap_paced_retained_yaw_20261029.py','paced_retained_yaw_plan_20261029.py','paced_retained_yaw_gate_20261029.py',
    'leap_empty_hold_comm_20261029.py','leap_retained_slow_yaw_20261029.py','fast_health_read_20261029.py',
    'fast_health_window_20261029.py','combined_quiet_reserve_20261029.py','retained_slow_yaw_gate_20261029.py',
    'retained_slow_yaw_plan_20261029.py','review_empty_hold_comm_20261029.py')
def hashes():
    return {**base_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [*(ROOT/n for n in FILES),REQUEST,REFERENCE]}}
class ReferenceSpeedYawPlan(Base):
    PLAN_VERSION='completed_physical_yaw75ms_reproduction_v1_20261029'
    TITLE='REFERENCE-SPEED YAW: EMPTY -> PLACE GREEN up/ORANGE wristward -> original75ms yaw -> OPEN -> REMOVE -> OFF.'
    FOLDER_PREFIX='reference_speed_yaw';STATUS='reference_speed_yaw_observation_complete'
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        Base.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
def prepare():
    source=ROOT/'fast_off_readonly_20261029_045411_027663/result.json';r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes']
    OUT.mkdir(parents=True,exist_ok=True)
    REQUEST.write_text(json.dumps(dict(plan_version=ReferenceSpeedYawPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
        off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    p=ReferenceSpeedYawPlan(REQUEST);rows=p.plan(p.source_position())
    print(json.dumps(dict(goals=len(rows),loaded_goals=len(loaded_rows()),nominal_motion_s=len(rows)*.075,devices_opened=False)))
if __name__=='__main__':prepare()
