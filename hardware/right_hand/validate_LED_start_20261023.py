"""Saved OFF LED-test pose only. Native path replay; no real device access."""
import hashlib
import json
from pathlib import Path
from grasped_yaw_plan import GraspedYawPlan
from thumb_live_start_gate import write_request,validate_proof
from validate_small_rotation_integration_20261022 import check

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261023/grasped_yaw_LED_complete_fixture'
SOURCE=ROOT/'bus_echo_off_20261023_233131_294853/result.json'


def main():
    data=SOURCE.read_bytes();r=json.loads(data);rows=r['reads']
    assert r['status']=='OFF_LED_echo_complete' and r['all_torque_off_verified']
    assert r['led_restoration_verified'] and r['goal_writes']==0 and r['torque_writes']==0
    state={k:rows[-1][k] for k in ('torque','led','error','watchdog','goal','position','voltage','temperature')}
    GraspedYawPlan.validate_off_state(state)
    OUT.mkdir(exist_ok=True)
    request=write_request(OUT,state,GraspedYawPlan.PLAN_VERSION,GraspedYawPlan.validate_off_state)
    payload=json.loads(request.read_bytes())
    payload['origin']='saved_OFF_LED_test_endpoint_for_offline_diagnosis_not_live_start_approval'
    request.write_text(json.dumps(payload,indent=2),encoding='utf-8')
    (OUT/'request.json').write_bytes(request.read_bytes())
    summary=check((GraspedYawPlan,OUT))
    proof=validate_proof(GraspedYawPlan(request),OUT/'screen.json')
    (OUT/'native_screen.json').write_bytes((OUT/'screen.json').read_bytes())
    summary.update(source=str(SOURCE.relative_to(ROOT)),source_sha256=hashlib.sha256(data).hexdigest(),
        saved_endpoint_is_not_live_start_approval=True,
        next_execution_still_requires_stable_fresh_OFF_reads=True,
        physical_cube_rotation_verified=False,real_devices_opened=False)
    (OUT/'replay_review.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
