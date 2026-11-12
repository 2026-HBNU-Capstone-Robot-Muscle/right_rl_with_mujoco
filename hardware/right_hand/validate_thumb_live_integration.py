"""Exercise Windows -> WSL -> proof validation, with stored OFF fixture only.

No camera, motor port or SDK is opened. This is not a current hardware pose.
"""
import datetime as dt
import argparse
import json
from pathlib import Path
from thumb_live_start_plan import ROOT,CurrentPlan
from thumb_live_start_gate import run_screen,validate_proof
from hand_preparation_plan import PreparationPlan
from prepared_placement_plan import PlacementPlan
from grasp_approach_plan import GraspApproachPlan
from grasp_place_plan import GraspPlacePlan

def check(kind,relative):
    fixture=ROOT/relative;request=fixture/'request.json'
    stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output=fixture/f'windows_wsl_screen_{stamp}.json'
    print('Checking fixture:',relative,flush=True)
    run_screen(request,output,cancel=lambda:None)
    proof=validate_proof(kind(request),output)
    # Refresh tests from the real subprocess, never a fake hardware pass.
    (fixture/'screen.json').write_bytes(output.read_bytes())
    summary=dict(actual_windows_wsl_subprocess_passed=True,accepted=proof['accepted'],
        static_cases=len(proof['static_screens']),native_ranges_unchanged=proof['native_ranges_unchanged'],
        max_static_self_overlap_mm=max(r['max_self_overlap_mm'] for r in proof['static_screens']),
        max_native_range_violation_rad=max(r['native_range_violation_rad'] for r in proof['static_screens']),
        dynamic=proof['dynamic_sensitivity'],source_fixture_only=True,
        physical_start_stability_verified=False,hardware_commands_sent=0,
        proof_file=str(output.relative_to(ROOT)))
    (fixture/'windows_integration.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();group=parser.add_mutually_exclusive_group()
    group.add_argument('--fingers',action='store_true');group.add_argument('--placement',action='store_true')
    group.add_argument('--grasp',action='store_true')
    group.add_argument('--loaded-approach',action='store_true')
    group.add_argument('--loaded-recovery',action='store_true')
    group.add_argument('--release-v6',action='store_true')
    group.add_argument('--all',action='store_true');args=parser.parse_args()
    if args.release_v6:
        from concurrent.futures import ThreadPoolExecutor
        suffixes=('fixture','prior_fixture','after_stop_fixture','first_stop_fixture',
                  'loaded_stop_fixture','125440_fixture','125613_fixture')
        with ThreadPoolExecutor(max_workers=2) as executor:
            list(executor.map(lambda suffix:check(GraspPlacePlan,
                'prepared_cube_review_20261022/grasp_trial_v6_'+suffix),suffixes))
        raise SystemExit(0)
    jobs=[(PlacementPlan,'right_hand_preparation_20261022/placement_fixture'),
          (PreparationPlan,'right_hand_preparation_20261022/finger_fixture'),
          (CurrentPlan,'right_hand_preparation_20261021/fresh_start_fixture'),
          (GraspApproachPlan,'prepared_cube_review_20261022/empty_approach_v2_fixture'),
          (GraspPlacePlan,'prepared_cube_review_20261022/grasp_trial_v6_fixture'),
          (GraspPlacePlan,'prepared_cube_review_20261022/grasp_trial_v6_prior_fixture'),
          (GraspPlacePlan,'prepared_cube_review_20261022/grasp_trial_v6_after_stop_fixture'),
          (GraspPlacePlan,'prepared_cube_review_20261022/grasp_trial_v6_first_stop_fixture'),
          (GraspPlacePlan,'prepared_cube_review_20261022/grasp_trial_v6_loaded_stop_fixture')]
    if args.all:
        for kind,relative in jobs:check(kind,relative)
    else:check(*jobs[8 if args.loaded_recovery else 4 if args.loaded_approach else 3 if args.grasp else 0 if args.placement else 1 if args.fingers else 2])
