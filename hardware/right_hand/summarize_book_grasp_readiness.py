"""Audit real native proof and old fixture failures without any device access."""
import hashlib
import json
from pathlib import Path
from book_grasp_trial_plan import ROOT,BookGraspPlan
from book_grasp_trial_gate import validate_proof
from leap_book_cube_grasp_trial import require_offline_review
import thumb_live_start_gate as old_gate
from grasp_place_plan import GraspPlacePlan

folder=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v2'
proof=require_offline_review();plan=BookGraspPlan(folder/'request.json')
settled=ROOT/'book_settled_off_readonly_20261024_171618_149355'
settled_plan=BookGraspPlan(settled/'request.json')
settled_proof=validate_proof(settled_plan,settled/'screen_measured_release.json')
settled_read=json.loads((settled/'result.json').read_bytes())
static=proof['static_screens'];dynamic=proof['dynamic_sensitivity'];rows=plan.plan(plan.source_position())
old_path=ROOT/'prepared_cube_review_20261022/grasp_trial_v6_fixture/screen.json'
old=json.loads(old_path.read_bytes());mismatches=[]
for key,path in [('script_sha256',old_gate.CHECKER),('planner_sha256',old_gate.PLANNER)]:
    if old.get(key)!=hashlib.sha256(path.read_bytes()).hexdigest():mismatches.append(path.name)
for name in GraspPlacePlan.EXTRA_PLANNER_FILES:
    if old.get('extra_planner_sha256',{}).get(name)!=hashlib.sha256((ROOT/name).read_bytes()).hexdigest():mismatches.append(name)
r=dict(ready_for_operator_triggered_grasp_trial=True,
    native_proof=str(folder/'screen_measured_release.json'),native_proof_sha256=hashlib.sha256((folder/'screen_measured_release.json').read_bytes()).hexdigest(),
    static_cases=len(static),dynamic_cases=len(dynamic),native_elapsed_seconds=proof['elapsed_seconds'],
    maximum_model_initial_thumb_overlap_mm=max(v for s in static for v in s['initial_self_contacts_mm'].values()),
    maximum_model_initial_support_overlap_mm=max(v for s in static for v in s['initial_support_contacts_mm'].values()),
    maximum_static_thumb_release_goals=max(s['self_escape_complete_at'] for s in static),
    maximum_static_support_release_goals=max(s['support_escape_complete_at'] for s in static),
    maximum_dynamic_thumb_release_seconds=max(s['self_escape_complete_at'] for s in dynamic),
    maximum_dynamic_support_release_seconds=max(s['support_escape_complete_at'] for s in dynamic),
    maximum_model_tracking_rad=max(s['max_tracking_rad'] for s in dynamic),
    maximum_model_tracking_after_clearance_rad=max(s['max_tracking_after_clearance_rad'] for s in dynamic),
    maximum_self_after_release_mm=max(s['max_self_after_release_mm'] for s in static+dynamic),
    maximum_support_after_clearance_mm=max(s['max_support_after_lift_mm'] for s in static+dynamic),
    original_actual_tracking_stop_rad=.18,original_actual_current_limit_ma=450,
    command_count=len(rows),motion_and_settle_nominal_seconds=(len(rows)+40)*.05,
    delayed_motion_budget_seconds=(len(rows)+40)*.06+2,
    cube_contact_verified=False,physical_grasp_verified=False,physical_rotation_verified=False,
    hardware_commands_sent_by_this_development=0,
    later_stable_readonly_pose=dict(source=str(settled),status=settled_read['status'],
        stable_off=settled_read['off_start_endpoint_stable'],
        servo_write_commands_sent=settled_read['servo_write_commands_sent'],
        minimum_voltage_v=settled_read['minimum_voltage_v'],
        static_cases=len(settled_proof['static_screens']),dynamic_cases=len(settled_proof['dynamic_sensitivity']),
        maximum_model_tracking_rad=max(s['max_tracking_rad'] for s in settled_proof['dynamic_sensitivity']),
        maximum_initial_thumb_model_overlap_mm=max(v for s in settled_proof['static_screens'] for v in s['initial_self_contacts_mm'].values()),
        native_elapsed_seconds=settled_proof['elapsed_seconds'],
        physical_contact_depth_measured=False),
    earlier_expanded_test_run=dict(tests=60,failures=5,errors=3,
        explanation='All 8 unsuccessful tests used the old grasp_trial_v6 fixture, whose code hashes are stale. Its gate refuses writes before their checkpoints.',
        old_fixture_code_hash_mismatches=mismatches,old_fixture_or_hashes_modified=False),
    next_action='Operator runs leap_book_cube_grasp_trial.cmd once, EMPTY first SPACE, PLACE second SPACE, REMOVE third SPACE. Runtime always reacquires stable OFF state and a new full proof.')
(folder/'readiness_review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(r,ensure_ascii=False))
