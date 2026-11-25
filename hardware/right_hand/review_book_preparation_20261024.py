"""Prepare an offline replay request from the already saved photo/encoders."""
import json
from pathlib import Path
import numpy as np
from book_empty_preparation_plan import BookPreparationPlan,REFERENCE,ROOT,LIFT
from hand_motor_mapping import SIM_TO_MOTOR_IDS

out=ROOT/'right_hand_rotation_20261024/book20_current_review'/('reference_fixture_'+BookPreparationPlan.PLAN_VERSION)
out.mkdir(exist_ok=True)
r=json.loads(REFERENCE.read_bytes());motors=sorted(r['snapshots'][0]['motors'],key=lambda v:v['id'])
fields=dict(position='position_ticks_signed',torque='torque_enabled',error='hardware_error',
            watchdog='bus_watchdog_raw',voltage='voltage_raw',temperature='temperature_c')
state={key:[v[field] for v in motors] for key,field in fields.items()}
request=out/'request.json'
request.write_text(json.dumps(dict(plan_version=BookPreparationPlan.PLAN_VERSION,
    motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=state,
    origin='offline_replay_of_saved_stationary_photo_encoder_pair',hardware_execution_allowed=False),indent=2),encoding='utf-8')
plan=BookPreparationPlan(request);raw=plan.source_position();rows=plan.plan(raw)
print(json.dumps(dict(request=str(request),goals=len(rows),roots_goals=sum(p==LIFT for p,_ in rows),
    nominal_seconds=len(rows)*.05,delayed_cycle_budget_seconds=(len(rows)+20)*.06+2,
    maximum_goal_step_ticks=int(np.max(abs(np.diff(np.vstack([raw]+[q for _,q in rows]),axis=0))))),indent=2))
