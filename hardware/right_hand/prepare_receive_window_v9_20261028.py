"""Bind the receive-window policy and new post-repair OFF pose, no devices."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
path=ROOT/'horizontal_quarter_gate_20261027.py';data=path.read_bytes()
needle=b'read_resynchronization_policy=dict(RECEIVE_POLICY),'
assert data.count(needle)==2
path.write_bytes(data.replace(needle,needle+b'sync_receive_window_policy=dict(WINDOW_POLICY),'))
import horizontal_quarter_plan_20261027 as p
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from observed_book_model_policy import healthy_off
source=ROOT/'repeat_off_readonly_20261028_010146_387518/result.json';off=json.loads(source.read_bytes())
assert off['status']=='off_snapshots_saved' and off['maximum_drift_ticks']<=2
assert off['servo_writes']==0 and off['torque_changes']==0 and len(off['snapshots'])==3
for s in off['snapshots']:healthy_off(s)
p.OUT.mkdir(exist_ok=False)
request=dict(plan_version=p.HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
    origin='after_user_reseated_ID11_host_receive_window_only',source=source.relative_to(ROOT).as_posix(),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),off_state=off['snapshots'][-1],hardware_execution_allowed=False)
p.REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
plan=p.HorizontalQuarterPlan(p.REQUEST)
print(json.dumps(dict(status='v9_request_ready_model_pending',goals=len(plan.plan(plan.source_position())),motor_writes=0)))
