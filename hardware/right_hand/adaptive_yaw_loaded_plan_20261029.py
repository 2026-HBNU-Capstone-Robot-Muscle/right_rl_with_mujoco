"""The original638-goal loaded route; only a bounded catch-up hook is added."""
import hashlib,json
from pathlib import Path
import numpy as np
from four_reply_yaw_plan_20261029 import FourReplyYawPlan as Base,hashes as base_hashes
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as RawBase,LOADED_PATH
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/adaptive_yaw_loaded'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json';STRESS_PATH=OUT/'hold_stress_candidate.npz'
EMPTY_EVIDENCE=ROOT/'adaptive_yaw_empty_20261029_062809_665980/offline_review/review.json'
FILES=('adaptive_yaw_loaded_plan_20261029.py','adaptive_yaw_loaded_gate_20261029.py','leap_adaptive_yaw_loaded_20261029.py',
    'leap_adaptive_yaw_loaded.cmd','screen_adaptive_yaw_loaded_20261029.py','screen_adaptive_yaw_loaded_stress_20261029.py',
    'root_lag_pause_20261029.py','root_lag_motion_runner_20261029.py','adaptive_yaw_empty_gate_20261029.py','review_adaptive_yaw_empty_20261029.py',
    'screen_adaptive_loaded_recovery_only_20261029.py','screen_loaded_yaw_convergence_20261029.py',
    'build_loaded_yaw_convergence_20261029.py','run_loaded_pause_convergence_20261029.py',
    'screen_adaptive_yaw_loaded_refined_20261029.py','aggregate_loaded_pause_refined_20261029.py')
def hashes():return {**base_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [*(ROOT/n for n in FILES),REQUEST,STRESS_PATH,EMPTY_EVIDENCE]}}
class AdaptiveYawLoadedPlan(Base):
    PLAN_VERSION='original_loaded_yaw638_bounded_ID12_wait_v1_20261029'
    TITLE='CUBE YAW: EMPTY -> PLACE GREEN up/ORANGE wristward -> grasp/638-goal yaw -> OPEN -> REMOVE -> OFF.'
    FOLDER_PREFIX='adaptive_yaw_loaded';STATUS='adaptive_loaded_yaw_observation_complete';proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
def prepare():
    source=ROOT/'fast_off_readonly_20261029_063206_154385/result.json';r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes']
    OUT.mkdir(parents=True,exist_ok=True)
    if REQUEST.exists() or STRESS_PATH.exists():raise RuntimeError('Refuse to overwrite prepared request/stress path')
    REQUEST.write_text(json.dumps(dict(plan_version=AdaptiveYawLoadedPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
        source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    z=np.load(LOADED_PATH);yaw=z['goals'][z['phases']=='yaw'];points=set(np.rint(np.linspace(20,len(yaw),16)).astype(int).tolist())
    goals=[];fixed=[]
    for index,q in enumerate(yaw,1):
        goals.append(q.copy());fixed.append(False)
        if index in points:
            for _ in range(16):goals.append(q.copy());fixed.append(True)
    np.savez_compressed(STRESS_PATH,start=z['grasp_goal'],goals=np.asarray(goals),fixed_hold_rows=np.asarray(fixed),
        hold_after_original_targets=np.asarray(sorted(points)),added_hold_seconds=19.2,each_hold_seconds=1.2)
    p=AdaptiveYawLoadedPlan(REQUEST);print(json.dumps(dict(goals=len(p.plan(p.source_position())),yaw_goals=len(yaw),stress_holds=len(points),devices_opened=False)))
if __name__=='__main__':prepare()
