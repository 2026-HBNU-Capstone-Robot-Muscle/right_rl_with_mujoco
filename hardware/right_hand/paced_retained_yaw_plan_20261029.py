"""Unchanged reviewed slow-yaw targets, with paced passive health observations."""
import hashlib,json
import numpy as np
from retained_slow_yaw_plan_20261029 import RetainedSlowYawPlan as Base,hashes as retained_hashes,loaded_rows
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as BasePlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/paced_retained_yaw'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json'
FILES=('paced_retained_yaw_plan_20261029.py','paced_retained_yaw_gate_20261029.py',
    'leap_paced_retained_yaw_20261029.py','leap_paced_retained_yaw.cmd',
    'screen_paced_retained_yaw_empty_20261029.py','screen_paced_retained_yaw_review_20261029.py',
    'leap_empty_hold_comm_20261029.py','review_empty_hold_comm_20261029.py')
def hashes():
    return {**retained_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [*(ROOT/n for n in FILES),REQUEST]}}
class PacedRetainedYawPlan(Base):
    PLAN_VERSION='retained_slow_yaw75ms_passive_observer_v1_20261029'
    TITLE='RED-UP SLOW YAW:75ms passive health reads; EMPTY -> PLACE -> grasp -> slow yaw -> OPEN -> REMOVE -> OFF.'
    FOLDER_PREFIX='paced_retained_yaw';STATUS='paced_retained_yaw_observation_complete'
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        BasePlan.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
def prepare():
    source=ROOT/'fast_off_readonly_20261029_042529_399612/result.json';r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes']
    OUT.mkdir(parents=True,exist_ok=True)
    REQUEST.write_text(json.dumps(dict(plan_version=PacedRetainedYawPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
        off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    p=PacedRetainedYawPlan(REQUEST);rows=p.plan(p.source_position())
    print(json.dumps(dict(goals=len(rows),loaded_goals=len(loaded_rows()),nominal_motion_s=len(rows)*.075,devices_opened=False)))
if __name__=='__main__':prepare()
