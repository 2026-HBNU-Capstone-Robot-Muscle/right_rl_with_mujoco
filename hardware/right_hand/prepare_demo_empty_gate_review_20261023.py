"""Keep historical proof and prepare latest OFF fixture, without devices."""
import hashlib
import json
from pathlib import Path
from slow_grasp_demo_plan import SlowGraspDemoPlan
from thumb_live_start_gate import write_request

ROOT=Path(__file__).resolve().parent


def main():
    fixture=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_fixture'
    old=fixture/'native_screen.json';blob=old.read_bytes()
    archive=fixture/('before_empty_start_gate_'+hashlib.sha256(blob).hexdigest()[:16]+'.json')
    if not archive.exists():archive.write_bytes(blob)
    validation=fixture.parent/'slow_grasp_demo_validation.json'
    validation_blob=validation.read_bytes()
    validation_archive=fixture/('validation_before_empty_start_gate_'+hashlib.sha256(validation_blob).hexdigest()[:16]+'.json')
    if not validation_archive.exists():validation_archive.write_bytes(validation_blob)
    source=ROOT/'slow_grasp_demo_20261023_175232_779550/result.json'
    blob=source.read_bytes();record=json.loads(blob)
    assert record['all_torque_off_verified'] and record['settings_restored']
    state=record['final_off_snapshots'][-1]
    latest=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_after_open_fixture'
    latest.mkdir(exist_ok=True)
    request=write_request(latest,state,SlowGraspDemoPlan.PLAN_VERSION,SlowGraspDemoPlan.validate_off_state)
    request_record=json.loads(request.read_bytes())
    request_record.update(origin='offline_latest_post_OFF_fixture_not_live_motion_authorization',
        source_sha256=hashlib.sha256(blob).hexdigest(),
        required_operator_precondition='Cube removed before next run. Fresh readings and a new online proof still required.')
    request.write_text(json.dumps(request_record,indent=2),encoding='utf-8')
    plan=SlowGraspDemoPlan(request);rows=plan.plan(plan.source_position())
    print('Latest OFF fixture:',latest.name,len(rows),'targets')
    print('Historical proof preserved:',archive.name)


if __name__=='__main__':main()
