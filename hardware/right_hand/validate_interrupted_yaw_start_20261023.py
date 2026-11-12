"""Native replay of the saved interrupted OFF endpoint, never a real device."""
import hashlib
import json
import argparse
from pathlib import Path
from grasped_yaw_plan import GraspedYawPlan
from thumb_live_start_gate import write_request,validate_proof
from validate_small_rotation_integration_20261022 import check

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'grasped_yaw_trial_20261023_215230_227769/result.json'
FOLDER=ROOT/'right_hand_rotation_20261023/grasped_yaw_after_stop_fixture'


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--goal-stop',action='store_true');args=parser.parse_args()
    source=ROOT/'grasped_yaw_trial_20261023_222816_628099/result.json' if args.goal_stop else SOURCE
    folder=ROOT/'right_hand_rotation_20261023/grasped_yaw_goal_stop_fixture' if args.goal_stop else FOLDER
    blob=source.read_bytes();r=json.loads(blob)
    assert r['all_torque_off_verified'] and r['settings_restored']
    assert r['failed_phase']==('stage_3_loaded_approach' if args.goal_stop else 'stage_5_grasped_small_yaw')
    state=r['final_off_snapshots'][-1]
    folder.mkdir(exist_ok=True)
    request=write_request(folder,state,GraspedYawPlan.PLAN_VERSION,GraspedYawPlan.validate_off_state)
    (folder/'request.json').write_bytes(request.read_bytes())
    summary=check((GraspedYawPlan,folder))
    validate_proof(GraspedYawPlan(request),folder/'screen.json')
    canonical=folder/'native_screen.json'
    if canonical.exists():
        old=canonical.read_bytes();archive=folder/('before_read_confirmation_'+hashlib.sha256(old).hexdigest()[:16]+'.json')
        if not archive.exists():archive.write_bytes(old)
    canonical.write_bytes((folder/'screen.json').read_bytes())
    summary.update(source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(blob).hexdigest(),
        source_OFF_endpoints_stationary=r.get('final_off_reference_valid'),
        saved_endpoint_is_not_live_start_approval=True,
        next_execution_still_requires_stable_fresh_OFF_reads=True,
        motor_commands_sent=0,real_devices_opened=False)
    (folder/'replay_review.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
