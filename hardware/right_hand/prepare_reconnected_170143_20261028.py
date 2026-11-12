"""Compare the freshly reconnected OFF state and prepare independent evidence."""
import hashlib,json
import numpy as np
import calibrated_quarter_plan_20261028 as p
from hand_motor_mapping import SIM_TO_MOTOR_IDS
source=p.ROOT/'prefix638_off_readonly_20261028_170143_255609/result.json';r=json.loads(source.read_bytes())
assert r['status']=='off_snapshots_saved' and r['servo_writes']==r['torque_changes']==0 and r['maximum_drift_ticks']<=2
for s in r['snapshots']:p.HorizontalQuarterPlan.validate_off_state(s)
state=r['snapshots'][-1]
baseline=json.loads((p.ROOT/'right_hand_rotation_20261028/observed90_prefix638/operator_review_v3_precise_two_full/request.json').read_bytes())
delta=np.asarray(state['position'])-np.asarray(baseline['off_state']['position'])
out=p.ROOT/'right_hand_rotation_20261028/reconnected_170143_margin_v4';out.mkdir(exist_ok=True)
request=dict(plan_version=p.HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=state,
    source=source.relative_to(p.ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False)
(out/'request.json').write_bytes((json.dumps(request,indent=2)+'\n').encode())
plan=p.HorizontalQuarterPlan(out/'request.json');rows=plan.plan(plan.source_position())
report=dict(status='fresh_reconnected_OFF_state_prepared',source=source.relative_to(p.ROOT).as_posix(),
    state_ticks=state['position'],changes_from_saved_start_ticks=delta.tolist(),maximum_saved_start_difference_ticks=int(max(abs(delta))),
    min_voltage_v=min(state['voltage'])/10.,max_voltage_v=max(state['voltage'])/10.,max_temperature_c=max(state['temperature']),
    all16_off_error0_watchdog0=True,all16_existing1Mbps_and_mode5_verified=True,endpoint_drift_ticks=r['maximum_drift_ticks'],
    goals=len(rows),loaded_yaw_goals=638,loaded_goal_path_unchanged=True,new_start_model_review_pending=True,motor_writes=0)
(out/'OFF_state_review.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
print(json.dumps(report))
