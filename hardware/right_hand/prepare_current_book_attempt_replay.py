"""Create a clearly offline replay request from the user's last saved reading."""
import json
import hashlib
from pathlib import Path
from book_empty_preparation_plan import BookPreparationPlan,ROOT,LIFT
from hand_motor_mapping import SIM_TO_MOTOR_IDS

source=ROOT/'book_prepare_empty_20261024_162129_845384/result.json'
data=source.read_bytes();r=json.loads(data);state=r['preflight_state']
assert r['error']=='Hand differs from the photographed book posture; no writes'
assert r['all_torque_off_verified'] and not r.get('initial') and not r.get('samples')
folder=ROOT/'right_hand_rotation_20261024/book20_current_review'/('current_attempt_fixture_'+BookPreparationPlan.PLAN_VERSION)
folder.mkdir(exist_ok=True)
request=folder/'request.json'
request.write_text(json.dumps(dict(plan_version=BookPreparationPlan.PLAN_VERSION,
    motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=state,
    origin='offline_replay_of_user_attempt_preflight_not_stable_off_measurement',
    observation_source=str(source),observation_source_sha256=hashlib.sha256(data).hexdigest(),
    hardware_execution_allowed=False),indent=2),encoding='utf-8')
plan=BookPreparationPlan(request);rows=plan.plan(plan.source_position())
print(json.dumps(dict(request=str(request),goals=len(rows),lift_goals=sum(p==LIFT for p,_ in rows),
    movement_and_hold_s=(len(rows)+20)*.05,delayed_budget_s=(len(rows)+20)*.06+2),indent=2))
