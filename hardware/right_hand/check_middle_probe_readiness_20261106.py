"""Check the frozen review and prove that the default entry opens no devices."""
import json,sys
from unittest.mock import patch
from middle_probe_plan_20261106 import O,REQUEST,PROOF,MiddleProbePlan,sha,hashes
from middle_probe_gate_20261106 import validate_review
def main():
    plan=MiddleProbePlan(REQUEST);proof=validate_review(plan,PROOF);rows=plan.plan(plan.source_position())
    import leap_middle_clearance_probe_20261106 as entry
    old=sys.argv
    with patch('dynamixel_sdk.PortHandler',side_effect=AssertionError('Unexpected motor access')) as port,patch.object(entry.NaturalCamera,'__init__',side_effect=AssertionError('Unexpected camera access')) as camera,patch.object(entry,'run_screen',side_effect=AssertionError('Unexpected simulator process')) as screen:
        try:sys.argv=['leap_middle_clearance_probe_20261106.py'];code=entry.main()
        finally:sys.argv=old
        assert code==0 and port.call_count==camera.call_count==screen.call_count==0
    result=dict(status='middle_probe_frozen_review_and_no_device_dry_run_passed',proof_sha256=sha(PROOF),code_hashes=hashes(),whole_goals=len(rows),nominal_motion_seconds=len(rows)*.075,empty_static_conditions=len(proof['empty_recovery']['static_cases']),empty_dynamic_conditions=len(proof['empty_recovery']['dynamic_cases']),cube_safety_conditions=proof['middle_probe_models']['cube_safety_cases'],hand_safety_conditions=proof['middle_probe_models']['hand_safety_cases'],operator_execute_and_SPACE_required=True,dry_run_result=code,motor_port_constructions=port.call_count,camera_constructions=camera.call_count,simulation_processes=screen.call_count,physical_trial_executed=False,new_common_grip_verified=False)
    (O/'ready.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in result.items() if k!='code_hashes'}),flush=True)
if __name__=='__main__':main()
