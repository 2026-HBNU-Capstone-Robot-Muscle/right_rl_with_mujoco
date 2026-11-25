"""Stored fixtures only: actual Windows->WSL screens, never serial/camera."""
from concurrent.futures import ThreadPoolExecutor
import datetime as dt
import json
from pathlib import Path
import argparse
from thumb_live_start_plan import ROOT, CurrentPlan
from hand_preparation_plan import PreparationPlan
from prepared_placement_plan import PlacementPlan
from grasp_approach_plan import GraspApproachPlan
from grasp_place_plan import GraspPlacePlan
from cube_small_rotation_plan import SmallRotationPlan
from supported_yaw_empty_plan import SupportedYawEmptyPlan
from supported_cube_rotation_plan import SupportedCubeRotationPlan
from slow_grasp_demo_plan import SlowGraspDemoPlan
from grasped_yaw_plan import GraspedYawPlan
from thumb_live_start_gate import write_request, run_screen, validate_proof
REUSE_VALID=False


def check(job):
    kind,folder=job
    request=folder/'request.json'
    reused=False
    if REUSE_VALID and (folder/'screen.json').exists():
        try:
            proof=validate_proof(kind(request),folder/'screen.json')
            reused=True
        except (RuntimeError,ValueError,KeyError,TypeError):pass
    stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output=folder/f'windows_wsl_screen_{stamp}.json'
    print('Offline fixture:',folder.name,'validated existing proof' if reused else 'native replay',flush=True)
    if not reused:
        run_screen(request,output,cancel=lambda:None)
        proof=validate_proof(kind(request),output)
        (folder/'screen.json').write_bytes(output.read_bytes())
    return dict(fixture=str(folder.relative_to(ROOT)),accepted=proof['accepted'],
                static_cases=len(proof['static_screens']),dynamic_cases=len(proof['dynamic_sensitivity']),
                maximum_tracking_rad=max(r['max_tracking_rad'] for r in proof['dynamic_sensitivity']),
                hardware_commands_sent=0,unchanged_validated_proof_reused=reused)


def main():
    global REUSE_VALID
    parser=argparse.ArgumentParser();group=parser.add_mutually_exclusive_group()
    group.add_argument('--rotation-only',action='store_true');group.add_argument('--supported-only',action='store_true')
    group.add_argument('--loaded-only',action='store_true')
    parser.add_argument('--resume',action='store_true',help='Reuse only complete proofs matching the exact request, path and current code hashes.')
    parser.add_argument('--include-grasped-yaw',action='store_true',help='Also verify three new yaw starts and the unchanged slow-demo fixture.')
    args=parser.parse_args();REUSE_VALID=args.resume
    out=ROOT/'right_hand_rotation_20261022'
    jobs=[]
    for name,relative in (
        ('fixture_v6_start','prepared_cube_review_20261022/grasp_trial_v6_fixture/request.json'),
        ('fixture_loaded_stop','prepared_cube_review_20261022/grasp_trial_v6_loaded_stop_fixture/request.json')):
        state=json.loads((ROOT/relative).read_bytes())['off_state']
        folder=out/name;folder.mkdir(exist_ok=True)
        path=write_request(folder,state,SmallRotationPlan.PLAN_VERSION,SmallRotationPlan.validate_off_state)
        (folder/'request.json').write_bytes(path.read_bytes())
        jobs.append((SmallRotationPlan,folder))
    source=ROOT/'cube_grasp_trial_20261022_141534_666300/result.json'
    state=json.loads(source.read_bytes())['final_off_snapshots'][-1]
    folder=out/'fixture_completed_off';folder.mkdir(exist_ok=True)
    path=write_request(folder,state,SmallRotationPlan.PLAN_VERSION,SmallRotationPlan.validate_off_state)
    (folder/'request.json').write_bytes(path.read_bytes())
    jobs.append((SmallRotationPlan,folder))
    if args.supported_only or args.loaded_only:jobs=[]
    if not args.rotation_only and not args.loaded_only:
        for label,source_kind in (('fixture_supported_current','current'),
                                  ('fixture_supported_prior','prior'),
                                  ('fixture_supported_loaded_stop','loaded_stop'),
                                  ('fixture_supported_failed_sag','failed_sag')):
            if source_kind=='current':
                latest=ROOT/'cube_small_rotation_trial_20261022_151241_962114/result.json'
                state=json.loads(latest.read_bytes())['final_off_snapshots'][-1]
            elif source_kind=='failed_sag':
                source=ROOT/'supported_yaw_empty_20261022_155659_749370/fresh_off_request.json'
                state=json.loads(source.read_bytes())['off_state']
            else:
                suffix='fixture' if source_kind=='prior' else 'loaded_stop_fixture'
                state=json.loads((ROOT/f'prepared_cube_review_20261022/grasp_trial_v6_{suffix}/request.json').read_bytes())['off_state']
            folder=out/label;folder.mkdir(exist_ok=True)
            path=write_request(folder,state,SupportedYawEmptyPlan.PLAN_VERSION,SupportedYawEmptyPlan.validate_off_state)
            (folder/'request.json').write_bytes(path.read_bytes())
            jobs.append((SupportedYawEmptyPlan,folder))
        old=[(CurrentPlan,'right_hand_preparation_20261021/fresh_start_fixture'),
             (PreparationPlan,'right_hand_preparation_20261022/finger_fixture'),
             (PlacementPlan,'right_hand_preparation_20261022/placement_fixture'),
             (GraspApproachPlan,'prepared_cube_review_20261022/empty_approach_v2_fixture')]
        suffixes=('fixture','prior_fixture','after_stop_fixture','first_stop_fixture',
                  'loaded_stop_fixture','125440_fixture','125613_fixture')
        old.extend((GraspPlacePlan,'prepared_cube_review_20261022/grasp_trial_v6_'+s) for s in suffixes)
        if not args.supported_only:jobs.extend((kind,ROOT/relative) for kind,relative in old)
    if args.loaded_only or not (args.rotation_only or args.supported_only):
        for label,relative,key in (
            ('fixture_loaded_supported_latest_off','supported_yaw_empty_20261023_030552_675209/result.json','final_off_snapshots'),
            ('fixture_loaded_supported_prior','supported_yaw_empty_20261022_155659_749370/fresh_off_request.json','off_state'),
            ('fixture_loaded_supported_watchdog_stop','cube_supported_rotation_trial_20261023_033108_297056/fresh_off_request.json','off_state')):
            record=json.loads((ROOT/relative).read_bytes())
            state=record[key][-1] if key=='final_off_snapshots' else record[key]
            folder=ROOT/'right_hand_rotation_20261023'/label;folder.mkdir(exist_ok=True)
            path=write_request(folder,state,SupportedCubeRotationPlan.PLAN_VERSION,SupportedCubeRotationPlan.validate_off_state)
            (folder/'request.json').write_bytes(path.read_bytes())
            jobs.append((SupportedCubeRotationPlan,folder))
    new_jobs=[]
    if args.include_grasped_yaw:
        for label in ('grasped_yaw_fixture','grasped_yaw_prior_fixture','grasped_yaw_open_fixture','slow_grasp_demo_fixture'):
            folder=ROOT/'right_hand_rotation_20261023'/label
            (folder/'request.json').write_bytes((folder/'fresh_off_request.json').read_bytes())
            new_jobs.append((SlowGraspDemoPlan if label=='slow_grasp_demo_fixture' else GraspedYawPlan,folder))
        jobs=new_jobs+jobs
    with ThreadPoolExecutor(max_workers=2) as executor:
        results=list(executor.map(check,jobs))
    if new_jobs:
        fresh=results[:len(new_jobs)];results=results[len(new_jobs):]
        for kind,folder in new_jobs:
            proof=validate_proof(kind(folder/'fresh_off_request.json'),folder/'screen.json')
            canonical=folder/'native_screen.json'
            if canonical.exists():
                import hashlib
                old=canonical.read_bytes();archive=folder/('before_grasped_yaw_'+hashlib.sha256(old).hexdigest()[:16]+'.json')
                if not archive.exists():archive.write_bytes(old)
            canonical.write_bytes((folder/'screen.json').read_bytes())
        (ROOT/'right_hand_rotation_20261023/grasped_yaw_integration_validation.json').write_text(
            json.dumps(dict(passed=True,fixtures=fresh,hardware_commands_sent=0,real_devices_opened=False),indent=2),encoding='utf-8')
    report=dict(passed=True,
                actual_windows_wsl_subprocess=any(not row['unchanged_validated_proof_reused'] for row in results),
                all_proofs_generated_by_windows_wsl=True,fixtures=results,
                hardware_commands_sent=0,real_camera_or_serial_opened=False)
    name=('supported_loaded_integration_validation.json' if args.loaded_only else
          'supported_empty_integration_validation.json' if args.supported_only else
          'rotation_integration_validation.json' if args.rotation_only else 'integration_validation.json')
    (out/name).write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__': main()
