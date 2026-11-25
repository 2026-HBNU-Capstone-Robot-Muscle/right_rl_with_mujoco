"""Separate operator proposal: place RED up/GREEN wristward, then slow yaw."""
import json,hashlib
from pathlib import Path
import numpy as np
from retained_red_yaw_plan_20261029 import RetainedRedYawPlan as Candidate,loaded_rows,PATH as LOADED_PATH
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as BasePlan,hashes as base_hashes
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/retained_slow_yaw_v1'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json'
FILES=('retained_slow_yaw_plan_20261029.py','retained_slow_yaw_gate_20261029.py',
    'leap_retained_slow_yaw_20261029.py','leap_retained_slow_yaw.cmd',
    'screen_retained_slow_yaw_empty_20261029.py','screen_retained_slow_yaw_hand_20261029.py',
    'screen_retained_slow_yaw_review_20261029.py','retained_red_yaw_plan_20261029.py',
    'combined_quiet_reserve_20261029.py','pitch_slow_yaw_gate_20261029.py','fast_health_read_20261029.py','fast_health_window_20261029.py',
    'fast_health_readonly_20261029_015443_298323/result.json','firmware_transport_readonly_20261029_014828_559357/result.json',
    '.runtime/python/Lib/site-packages/dynamixel_sdk/protocol2_packet_handler.py')
def hashes():
    paths=[ROOT/f for f in FILES]+[REQUEST,LOADED_PATH,OUT/'loaded_hand_dynamics/review.json']
    return {**base_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}
class RetainedSlowYawPlan(Candidate):
    PLAN_VERSION='supervised_red_up_yaw150ms_v1_20261029'
    TITLE='RED-UP SLOW YAW: EMPTY preparation -> PLACE RED up/GREEN wristward -> grasp -> slow yaw -> OPEN -> REMOVE -> OFF.'
    FOLDER_PREFIX='retained_slow_yaw';STATUS='retained_slow_yaw_observation_complete'
    COMPLETE_MESSAGE='Red-up slow yaw observation ended; cube removed; OFF/settings restored. Review the saved video for actual rotation.'
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        BasePlan.__init__(self,request)
        self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
def prepare():
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--source',default='prefix638_off_readonly_20261029_005950_947910/result.json');args=parser.parse_args()
    source=(ROOT/args.source).resolve();source.relative_to(ROOT);r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and r['servo_writes']==0 and r['torque_changes']==0
    OUT.mkdir(parents=True,exist_ok=True)
    request=dict(plan_version=RetainedSlowYawPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
        off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False)
    REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    p=RetainedSlowYawPlan(REQUEST);rows=p.plan(p.source_position())
    print(json.dumps(dict(status='proposal_awaiting_review',total_goals=len(rows),loaded_goals=len(loaded_rows()),
        nominal_motion_s=len(rows)*.075,current_ma=450,active_limit_s=p.ON_TIMEOUT,start=p.source_position().tolist(),devices_opened=False)))
if __name__=='__main__':prepare()
