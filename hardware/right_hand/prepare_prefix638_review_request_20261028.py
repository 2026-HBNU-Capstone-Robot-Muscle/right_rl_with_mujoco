"""Bind the fresh READ-only snapshot to the new offline recovery check."""
import json,hashlib
from pathlib import Path
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan,OUT,REQUEST
ROOT=Path(__file__).resolve().parent
source=ROOT/'prefix638_off_readonly_20261028_051557_812726/result.json'
r=json.loads(source.read_bytes())
assert r['status']=='off_snapshots_saved' and r['servo_writes']==r['torque_changes']==0
assert r['maximum_drift_ticks']<=2 and len(r['snapshots'])==3
for state in r['snapshots']:HorizontalQuarterPlan.validate_off_state(state)
OUT.mkdir(exist_ok=True)
request=dict(plan_version=HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
    off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False)
REQUEST.write_bytes((json.dumps(request,indent=2)+'\n').encode())
plan=HorizontalQuarterPlan(REQUEST);rows=plan.plan(plan.source_position())
print(json.dumps(dict(request=REQUEST.relative_to(ROOT).as_posix(),start=plan.source_position().tolist(),goals=len(rows),nominal_motion_s=len(rows)*plan.SAMPLE_PERIOD,all_proof_sources_present=len(plan.proof_hashes()),no_devices_opened=True)))
