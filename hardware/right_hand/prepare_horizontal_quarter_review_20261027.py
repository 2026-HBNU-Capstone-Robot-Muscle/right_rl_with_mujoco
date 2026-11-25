"""Prepare a source-bound request from completed READ-only snapshots."""
import json,hashlib
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan,ROOT,OUT,REQUEST
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from observed_book_model_policy import healthy_off

def main():
    source=ROOT/'repeat_off_readonly_20261027_183434_733420/result.json';r=json.loads(source.read_bytes())
    if r['status']!='off_snapshots_saved' or r['servo_writes']!=0 or r['torque_changes']!=0 or r['maximum_drift_ticks']>2 or len(r['snapshots'])!=3:raise ValueError('Need three healthy stable READ-only snapshots')
    for state in r['snapshots']:healthy_off(state)
    request=dict(plan_version=HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),origin='three_assistant_ping_read_only_snapshots_user_power_confirmed',source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),off_state=r['snapshots'][-1],hardware_execution_allowed=False)
    OUT.mkdir(exist_ok=True)
    if REQUEST.exists() and json.loads(REQUEST.read_bytes())!=request:raise ValueError('Refuse to replace canonical request')
    REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8');plan=HorizontalQuarterPlan(REQUEST);rows=plan.plan(plan.source_position())
    print(json.dumps(dict(source=str(REQUEST),goals=len(rows),nominal_active_seconds=len(rows)*plan.SAMPLE_PERIOD,active_limit_seconds=plan.ON_TIMEOUT,minimum_voltage_v=min(v/10 for s in r['snapshots'] for v in s['voltage']),maximum_temperature_c=max(v for s in r['snapshots'] for v in s['temperature']),devices_opened=False,motor_writes=0)))
if __name__=='__main__':main()
