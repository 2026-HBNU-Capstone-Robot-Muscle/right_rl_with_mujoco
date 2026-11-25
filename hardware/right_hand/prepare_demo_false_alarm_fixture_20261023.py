"""Saved stable OFF readings from the false-stop run, no live devices."""
import hashlib
import json
from pathlib import Path
from slow_grasp_demo_plan import SlowGraspDemoPlan
from thumb_live_start_gate import write_request

ROOT=Path(__file__).resolve().parent


def main():
    original=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_fixture'
    for name in ('native_screen.json','../slow_grasp_demo_validation.json'):
        source=(original/name).resolve();blob=source.read_bytes()
        destination=original/('before_temporal_gate_'+source.stem+'_'+hashlib.sha256(blob).hexdigest()[:16]+'.json')
        if not destination.exists():destination.write_bytes(blob)
    source=ROOT/'slow_grasp_demo_20261023_180831_999579/fresh_off_request.json'
    blob=source.read_bytes();request_record=json.loads(blob)
    destination=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_empty_verified_fixture'
    destination.mkdir(exist_ok=True)
    request=write_request(destination,request_record['off_state'],SlowGraspDemoPlan.PLAN_VERSION,SlowGraspDemoPlan.validate_off_state)
    record=json.loads(request.read_bytes());record.update(origin='offline_false_stop_stable_OFF_fixture_not_live_motion_authorization',
        source_request_sha256=hashlib.sha256(blob).hexdigest(),
        required_operator_precondition='Keep cube removed until PLACE CUBE; actual execution requires fresh reads and new proof.')
    request.write_text(json.dumps(record,indent=2),encoding='utf-8')
    plan=SlowGraspDemoPlan(request);print('Latest OFF targets:',len(plan.plan(plan.source_position())))


if __name__=='__main__':main()
