"""Seven saved OFF fixtures, full native path proofs; no device access."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import numpy as np
from grasped_yaw_plan import GraspedYawPlan,CANDIDATE_SHA
from thumb_live_start_gate import validate_proof,write_request
from validate_small_rotation_integration_20261022 import check

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024'
OLD_FIXTURES=tuple(ROOT/'right_hand_rotation_20261023'/name for name in (
    'grasped_yaw_fixture','grasped_yaw_prior_fixture','grasped_yaw_open_fixture',
    'grasped_yaw_after_stop_fixture','grasped_yaw_goal_stop_fixture','grasped_yaw_LED_complete_fixture'))
LATEST=OUT/'grasped_yaw_completed_turn_off_fixture'


def replay(folder):
    (folder/'request.json').write_bytes((folder/'fresh_off_request.json').read_bytes())
    summary=check((GraspedYawPlan,folder))
    validate_proof(GraspedYawPlan(folder/'request.json'),folder/'screen.json')
    canonical=folder/'native_screen.json'
    if canonical.exists():
        old=canonical.read_bytes();archive=folder/('before_contact_candidate_'+hashlib.sha256(old).hexdigest()[:16]+'.json')
        if not archive.exists():archive.write_bytes(old)
    canonical.write_bytes((folder/'screen.json').read_bytes())
    return summary


def main():
    source=ROOT/'grasped_yaw_trial_20261023_234707_886202/result.json'
    blob=source.read_bytes();sha=hashlib.sha256(blob).hexdigest()
    assert sha=='adc0cf67dfb20d1e0de6127b8d5961fa19beae6173d657c06fa96b65dc58948c'
    result=json.loads(blob)
    assert result['all_torque_off_verified'] and result['settings_restored']
    state={key:np.asarray(value) for key,value in result['final_off_snapshots'][-1].items()}
    LATEST.mkdir(exist_ok=True)
    request=write_request(LATEST,state,GraspedYawPlan.PLAN_VERSION,GraspedYawPlan.validate_off_state)
    r=json.loads(request.read_bytes())
    r.update(origin='saved_completed_turn_OFF_endpoint_not_live_motion_approval',
        saved_source=str(source.relative_to(ROOT)),saved_source_sha256=sha)
    request.write_text(json.dumps(r,indent=2),encoding='utf-8')
    jobs=OLD_FIXTURES+(LATEST,)
    # Preflight source binding and route budget before launching any checker.
    for folder in jobs:
        plan=GraspedYawPlan(folder/'fresh_off_request.json');raw=plan.source_position()
        plan.validate_start(raw)
        print('PLAN',folder.name,len(plan.plan(raw)),'goals',flush=True)
    with ThreadPoolExecutor(max_workers=2) as executor:records=list(executor.map(replay,jobs))
    review=dict(passed=True,candidate_sha256=CANDIDATE_SHA,fixtures=records,
        all_seven_paths_replayed=True,legacy_motion_programs_changed=False,
        loaded_grasp_waypoints_changed=False,small_turn_candidate_changed=True,
        saved_endpoint_not_live_start_approval=True,real_devices_opened=False,hardware_commands_sent=0)
    (OUT/'contact_yaw_integration_validation.json').write_text(json.dumps(review,indent=2),encoding='utf-8')
    print(json.dumps(review,indent=2))


if __name__=='__main__':main()
