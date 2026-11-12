"""Compact evidence summary after native validation and device-free tests."""
import json
import argparse
from pathlib import Path
from book_empty_preparation_plan import BookPreparationPlan,ROOT
from book_empty_preparation_gate import validate_proof

parser=argparse.ArgumentParser();parser.add_argument('--fixture',default='structural_scope_fix_fixture')
parser.add_argument('--proof',default='screen.json');args=parser.parse_args()
if Path(args.fixture).name!=args.fixture or Path(args.proof).name!=args.proof:
    raise ValueError('Fixture and proof must be local basenames')
folder=ROOT/'right_hand_rotation_20261024/book20_current_review'/args.fixture
plan=BookPreparationPlan(folder/'request.json');r=validate_proof(plan,folder/args.proof)
raw=plan.source_position();rows=plan.plan(raw)
result=dict(status='ready_for_operator_triggered_empty_preparation',plan_version=plan.PLAN_VERSION,
    source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(raw),
    native_static_cases=len(r['static_screens']),native_dynamic_cases=len(r['dynamic_sensitivity']),
    native_worker_elapsed_seconds=r.get('elapsed_seconds'),
    max_model_tracking_rad=max(s['max_tracking_rad'] for s in r['dynamic_sensitivity']),
    max_static_self_overlap_mm=max(s['max_self_overlap_mm'] for s in r['static_screens']),
    max_dynamic_self_overlap_mm=max(s['max_self_overlap_mm'] for s in r['dynamic_sensitivity']),
    max_support_after_lift_mm=max(s['max_support_after_lift_mm'] for s in r['static_screens']+r['dynamic_sensitivity']),
    initial_support_model_overlap_max_mm=max(max(s['initial_support_contacts_mm'].values(),default=0.)
        for s in r['static_screens']),
    maximum_support_separation_goal=max(s['support_escape_complete_at'] for s in r['static_screens']),
    maximum_dynamic_support_separation_s=max(s['support_escape_complete_at'] for s in r['dynamic_sensitivity']),
    goal_samples=len(rows),nominal_movement_and_hold_seconds=(len(rows)+20)*.05,
    delayed_cycle_budget_seconds=(len(rows)+20)*.06+2,code_hashes=r['code_hashes'],
    launcher='leap_book_prepare_empty.cmd',physical_calibration_verified=False,
    free_space_certificate=False,physical_preparation_verified=False,physical_cube_rotation_verified=False,
    real_devices_opened_by_native_screener=False,motor_commands_sent_during_development=0,
    old_rotation_launcher_ready_for_current_book_setup=False,
    limitations=['Book front sensitivity cases are photo-based assumptions, not measured uncertainty bounds.',
        'Raw and native numerical admission is not approval; each exact current EMPTY route must pass the full new screen.',
        'Existing modeled support overlaps are permitted only to separate monotonically; new/deeper contacts are rejected.',
        'Live execution requires SPACE, fresh stable OFF telemetry and a new complete native screen before writes.'])
(folder/'ready_review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k!='code_hashes'},indent=2))
