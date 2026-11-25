import hashlib,json
import numpy as np
from reverse_yaw_empty_plan_20261029 import ROOT,OUT,REQUEST,ReverseYawEmptyPlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS
source=ROOT/'fast_off_readonly_20261029_162936_457804/result.json';r=json.loads(source.read_bytes())
assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and not r['servo_writes'] and not r['torque_changes']
folder=OUT/'live_recovery_162936';folder.mkdir(exist_ok=True);path=folder/'request.json'
if path.exists():raise RuntimeError('Refuse to overwrite current request')
path.write_text(json.dumps(dict(plan_version=ReverseYawEmptyPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
    source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
p=ReverseYawEmptyPlan(path);old=ReverseYawEmptyPlan(REQUEST).source_position();delta=np.asarray(r['snapshots'][-1]['position'])-old
print(json.dumps(dict(delta_ticks=delta.tolist(),maximum_delta_ticks=int(max(abs(delta))),goals=len(p.plan(p.source_position())),devices_opened=False)))
