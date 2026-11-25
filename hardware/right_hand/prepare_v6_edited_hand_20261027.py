"""Bind the newly read, cube-free posture to the unchanged large-turn data."""
import hashlib,json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,HorizontalQuarterPlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from observed_book_model_policy import healthy_off
source=ROOT/'repeat_off_readonly_20261027_204724_205068'/'result.json'
data=json.loads(source.read_text(encoding='utf-8'))
assert data['status']=='off_snapshots_saved' and data['servo_writes']==0 and data['maximum_drift_ticks']<=2
for state in data['snapshots']:healthy_off(state)
OUT.mkdir(exist_ok=True)
request=dict(plan_version=HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
    origin='user_changed_hand_cube_removed_fresh_readonly_three_snapshots',
    source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    off_state=data['snapshots'][-1],hardware_execution_allowed=False)
REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
plan=HorizontalQuarterPlan(REQUEST);rows=plan.plan(plan.source_position())
print(json.dumps(dict(request=str(REQUEST),goals=len(rows),motor_writes=0),ensure_ascii=False))
