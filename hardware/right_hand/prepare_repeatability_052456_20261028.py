"""Bind a fresh READ-only OFF state for a same-route repeatability trial."""
import hashlib,json
from calibrated_quarter_plan_20261028 import ROOT,HorizontalQuarterPlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS
source=ROOT/'prefix638_off_readonly_20261028_053405_769389/result.json'
r=json.loads(source.read_bytes());assert r['status']=='off_snapshots_saved' and r['servo_writes']==r['torque_changes']==0
assert r['maximum_drift_ticks']<=2 and len(r['snapshots'])==3
for s in r['snapshots']:HorizontalQuarterPlan.validate_off_state(s)
out=ROOT/'right_hand_rotation_20261028/repeatability_after_052456';out.mkdir(exist_ok=True)
request=dict(plan_version=HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
    source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False)
(out/'request.json').write_bytes((json.dumps(request,indent=2)+'\n').encode())
p=HorizontalQuarterPlan(out/'request.json');rows=p.plan(p.source_position())
print(json.dumps(dict(request=(out/'request.json').relative_to(ROOT).as_posix(),goals=len(rows),source_read_drift_ticks=r['maximum_drift_ticks'],new_motion_goals=False,motor_writes=0)))
