"""Current OFF request from the precise READ-only observation; no devices."""
import hashlib,json
import numpy as np
import calibrated_quarter_plan_20261028 as p
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from calibrated_precise_clock_20261028 import review
evidence=review();source=p.ROOT/evidence['source'];r=json.loads(source.read_bytes());snapshots=r['snapshots']
assert len(snapshots)==3
for s in snapshots:p.HorizontalQuarterPlan.validate_off_state(s)
drift=int(np.max(np.ptp([s['position'] for s in snapshots],axis=0)));assert drift<=2
p.OUT.mkdir(exist_ok=True)
request=dict(plan_version=p.HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=snapshots[-1],source=source.relative_to(p.ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False)
p.REQUEST.write_bytes((json.dumps(request,indent=2)+'\n').encode())
plan=p.HorizontalQuarterPlan(p.REQUEST);rows=plan.plan(plan.source_position())
print(json.dumps(dict(request=p.REQUEST.relative_to(p.ROOT).as_posix(),goals=len(rows),nominal_seconds=len(rows)*.075,current_off_drift_ticks=drift,devices_opened=False)))
