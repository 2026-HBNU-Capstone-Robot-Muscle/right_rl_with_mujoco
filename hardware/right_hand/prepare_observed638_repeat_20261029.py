"""Fresh read-only pose for one reproducibility check of the same demo."""
import hashlib,json
from pathlib import Path
from adaptive_yaw_loaded_plan_20261029 import AdaptiveYawLoadedPlan,ROOT
from hand_motor_mapping import SIM_TO_MOTOR_IDS
source=ROOT/'fast_off_readonly_20261029_161735_921599/result.json';r=json.loads(source.read_bytes())
assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes']
folder=ROOT/'right_hand_rotation_20261029/observed_loaded_160451/next_repeat_recovery';folder.mkdir(parents=True,exist_ok=True)
path=folder/'request.json'
if path.exists():raise RuntimeError('Refuse to overwrite reviewed request')
path.write_text(json.dumps(dict(plan_version=AdaptiveYawLoadedPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
    source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
p=AdaptiveYawLoadedPlan(path);print(json.dumps(dict(request=path.relative_to(ROOT).as_posix(),goals=len(p.plan(p.source_position())),devices_opened=False)))
