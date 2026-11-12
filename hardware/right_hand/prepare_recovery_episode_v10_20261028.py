"""Bind policy and a fresh OFF reference; never access devices."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
path=ROOT/'horizontal_quarter_gate_20261027.py'
data=path.read_bytes()
old=b'sync_receive_window_policy=dict(WINDOW_POLICY),'
new=old+b'read_recovery_episode_policy=dict(EPISODE_POLICY),'
assert data.count(old)==2 and b'read_recovery_episode_policy' not in data
path.write_bytes(data.replace(old,new))
from horizontal_quarter_plan_20261027 import OUT,HorizontalQuarterPlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from observed_book_model_policy import healthy_off
source=ROOT/'repeat_off_readonly_20261028_020314_646667/result.json';r=json.loads(source.read_bytes())
assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']==0 and r['servo_writes']==0
for state in r['snapshots']:healthy_off(state)
OUT.mkdir(exist_ok=True)
request=dict(plan_version=HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
    origin='post_stopped_empty_open_020027_bounded_confirmation_recovery',source=source.relative_to(ROOT).as_posix(),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),off_state=r['snapshots'][-1],hardware_execution_allowed=False)
(OUT/'request.json').write_bytes((json.dumps(request,indent=2)+'\n').encode())
plan=HorizontalQuarterPlan(OUT/'request.json')
print(json.dumps(dict(status='v10_source_ready_model_pending',goals=len(plan.plan(plan.source_position())),motor_writes=0)))
