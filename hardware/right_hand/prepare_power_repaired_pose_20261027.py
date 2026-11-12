"""Create an OFF-only model request after power wiring reconnection."""
import hashlib,json
from horizontal_quarter_plan_20261027 import ROOT,HorizontalQuarterPlan
from observed_book_model_policy import healthy_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS
source=ROOT/'repeat_off_readonly_20261027_214344_043175'/'result.json'
data=json.loads(source.read_text(encoding='utf-8'))
assert data['status']=='off_snapshots_saved' and data['servo_writes']==0 and data['maximum_drift_ticks']<=2
for state in data['snapshots']:healthy_off(state)
out=ROOT/'right_hand_rotation_20261027'/'power_repaired_214344';out.mkdir(exist_ok=True)
request=dict(plan_version=HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
    origin='reported_power_reconnected_three_fresh_off_reads_no_motion',
    source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    off_state=data['snapshots'][-1],hardware_execution_allowed=False)
path=out/'request.json';path.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
plan=HorizontalQuarterPlan(path);rows=plan.plan(plan.source_position())
print(json.dumps(dict(request=str(path),goals=len(rows),assistant_motor_writes=0),ensure_ascii=False))
