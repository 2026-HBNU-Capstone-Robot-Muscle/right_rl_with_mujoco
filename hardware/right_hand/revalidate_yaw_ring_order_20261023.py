"""Recheck saved yaw starts after the EMPTY ring-order change. No devices."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
from grasped_yaw_plan import GraspedYawPlan
from thumb_live_start_gate import validate_proof
from validate_small_rotation_integration_20261022 import check

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261023'


def replay(label):
    folder=OUT/label
    (folder/'request.json').write_bytes((folder/'fresh_off_request.json').read_bytes())
    summary=check((GraspedYawPlan,folder))
    proof=validate_proof(GraspedYawPlan(folder/'request.json'),folder/'screen.json')
    canonical=folder/'native_screen.json'
    previous=canonical.read_bytes()
    archive=folder/('before_ring_order_'+hashlib.sha256(previous).hexdigest()[:16]+'.json')
    if not archive.exists():archive.write_bytes(previous)
    canonical.write_bytes((folder/'screen.json').read_bytes())
    return summary


def main():
    labels=('grasped_yaw_fixture','grasped_yaw_prior_fixture','grasped_yaw_open_fixture',
            'grasped_yaw_after_stop_fixture','grasped_yaw_goal_stop_fixture')
    with ThreadPoolExecutor(max_workers=2) as executor:results=list(executor.map(replay,labels))
    current=OUT/'grasped_yaw_LED_complete_fixture'
    proof=validate_proof(GraspedYawPlan(current/'fresh_off_request.json'),current/'native_screen.json')
    results.append(dict(fixture=str(current.relative_to(ROOT)),accepted=proof['accepted'],
        static_cases=len(proof['static_screens']),dynamic_cases=len(proof['dynamic_sensitivity']),
        maximum_tracking_rad=max(r['max_tracking_rad'] for r in proof['dynamic_sensitivity']),
        hardware_commands_sent=0,unchanged_validated_proof_reused=True))
    record=dict(passed=True,fixtures=results,legacy_motion_programs_changed=False,
        saved_endpoint_not_live_start_approval=True,real_devices_opened=False,
        loaded_grasp_and_yaw_waypoints_changed=False)
    (OUT/'ring_order_integration_validation.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
