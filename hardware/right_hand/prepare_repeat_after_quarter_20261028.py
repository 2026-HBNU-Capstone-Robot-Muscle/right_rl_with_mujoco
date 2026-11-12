"""Prepare a repeat source from fresh confirmed OFF reads, without devices."""
import json,hashlib
from horizontal_quarter_plan_20261027 import ROOT,HorizontalQuarterPlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from observed_book_model_policy import healthy_off
source=ROOT/'repeat_off_readonly_20261028_014813_732889/result.json';r=json.loads(source.read_bytes())
assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and r['servo_writes']==0
for state in r['snapshots']:healthy_off(state)
out=ROOT/'right_hand_rotation_20261028/repeat_after_quarter_012305';out.mkdir(exist_ok=False)
request=dict(plan_version=HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
    origin='repeatability_after_first_observed_horizontal_quarter',source=source.relative_to(ROOT).as_posix(),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),off_state=r['snapshots'][-1],hardware_execution_allowed=False)
path=out/'request.json';path.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
plan=HorizontalQuarterPlan(path)
print(json.dumps(dict(status='repeat_source_ready_model_pending',goals=len(plan.plan(plan.source_position())),motor_writes=0)))
