"""Same EMPTY joint targets; only bounded catch-up timing is added."""
import hashlib,json
from pathlib import Path
import numpy as np
from yaw_load_empty_plan_20261029 import YawLoadEmptyPlan as Base,hashes as base_hashes,motion_prefix
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as RawBase
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/adaptive_yaw_empty'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json';STRESS_PATH=OUT/'hold_stress_path.npz'
FILES=('adaptive_yaw_empty_plan_20261029.py','adaptive_yaw_empty_gate_20261029.py','leap_adaptive_yaw_empty_20261029.py',
    'leap_adaptive_yaw_empty.cmd','screen_adaptive_yaw_empty_20261029.py','screen_adaptive_yaw_hand_20261029.py',
    'root_lag_pause_20261029.py','root_lag_motion_runner_20261029.py')
def hashes():return {**base_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [*(ROOT/n for n in FILES),REQUEST,STRESS_PATH]}}
class AdaptiveYawEmptyPlan(Base):
    PLAN_VERSION='EMPTY_yaw320_bounded_middle_root_wait_v1_20261029'
    TITLE='EMPTY CATCH-UP COMPARISON: same goals, ID12 wait if lagged -> OFF. CUBE stays OUT throughout.'
    FOLDER_PREFIX='adaptive_yaw_empty';STATUS='adaptive_empty_yaw_observation_complete';proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
def prepare():
    source=ROOT/'fast_off_readonly_20261029_060806_538777/result.json';r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes']
    OUT.mkdir(parents=True,exist_ok=True)
    REQUEST.write_text(json.dumps(dict(plan_version=AdaptiveYawEmptyPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
        source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    from new_pair_grasp_plan_20261027 import OPEN_PATH
    goals=[];phases=[];count=0;holds=0
    for phase,q in motion_prefix():
        goals.append(q);phases.append(phase)
        if phase=='stage_7_horizontal_yaw':
            count+=1
            if count%20==0:
                for _ in range(16):goals.append(q.copy());phases.append('fixed_hold_stress')
                holds+=1
    np.savez_compressed(STRESS_PATH,start=np.load(OPEN_PATH)['open_goal'],goals=np.asarray(goals),phases=np.asarray(phases),
        stress_hold_count=holds,stress_added_seconds=holds*1.2)
    p=AdaptiveYawEmptyPlan(REQUEST);print(json.dumps(dict(goals=len(p.plan(p.source_position())),nominal_motion_seconds=len(p.plan(p.source_position()))*.075,stress_holds=holds,devices_opened=False)))
if __name__=='__main__':prepare()
