"""Build a labelled offline replay request from saved hardware telemetry only."""
import hashlib
import json
from collections import Counter
from pathlib import Path
from book_grasp_trial_plan import BookGraspPlan,ROOT,LOADED_PHASE

source=ROOT/'book_prepare_empty_20261024_165621_852486/result.json'
r=json.loads(source.read_bytes())
if (r['status']!='book_empty_preparation_complete' or not r['all_torque_off_verified']
        or not r['settings_restored']):raise ValueError('Preparation was not completed')
folder=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v2'
folder.mkdir(exist_ok=True)
path=folder/'request.json'
request=dict(plan_version=BookGraspPlan.PLAN_VERSION,
    motor_ids_in_sim_order=r['joint_mapping']['motor_ids_in_sim_order'],
    origin='offline_replay_of_saved_final_OFF_snapshot_NOT_live_stable_readings',
    original_result=str(source),original_result_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    original_OFF_stationarity_verified=r['final_off_reference_valid'],
    off_state=r['final_off_snapshots'][-1])
if path.exists() and json.loads(path.read_bytes())!=request:raise ValueError('Refuse to replace offline source')
path.write_text(json.dumps(request,indent=2),encoding='utf-8')
plan=BookGraspPlan(path);rows=plan.plan(plan.source_position())
review=dict(request=str(path),phase_counts=dict(Counter(p for p,_ in rows)),
    command_count=len(rows),nominal_motion_and_settle_seconds=(len(rows)+40)*.05,
    delayed_motion_budget_seconds=(len(rows)+40)*.06+2,
    placement_goal=next(q.tolist() for i,(p,q) in enumerate(rows) if i+1<len(rows) and rows[i+1][0]==LOADED_PHASE and p!=LOADED_PHASE),
    final_goal=rows[-1][1].tolist(),hardware_commands_sent=0,
    note='Historical OFF replay only. Runtime acquires three fresh stable OFF reads and replays the full path again.')
(folder/'plan_review.json').write_text(json.dumps(review,indent=2),encoding='utf-8')
print(json.dumps(review))
