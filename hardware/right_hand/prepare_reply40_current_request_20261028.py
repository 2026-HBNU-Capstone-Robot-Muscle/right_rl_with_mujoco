"""Use the final three current READ-only snapshots; no device access."""
import hashlib,json
import numpy as np
import calibrated_quarter_plan_20261028 as p
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from single_reply40_evidence_20261028 import SOURCE,review
evidence=review();raw=json.loads(SOURCE.read_bytes());snapshots=raw['snapshots']
assert len(snapshots)==3
for state in snapshots:p.HorizontalQuarterPlan.validate_off_state(state)
drift=int(np.max(np.ptp([s['position'] for s in snapshots],axis=0)));assert drift<=2
p.OUT.mkdir(exist_ok=True)
request=dict(plan_version=p.HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=snapshots[-1],
    source=SOURCE.relative_to(p.ROOT).as_posix(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),hardware_execution_allowed=False)
p.REQUEST.write_bytes((json.dumps(request,indent=2)+'\n').encode())
plan=p.HorizontalQuarterPlan(p.REQUEST);rows=plan.plan(plan.source_position())
print(json.dumps(dict(request=p.REQUEST.relative_to(p.ROOT).as_posix(),goals=len(rows),nominal_motion_s=len(rows)*.075,off_drift_ticks=drift,loaded_goals_unchanged=True,devices_opened=False)))
