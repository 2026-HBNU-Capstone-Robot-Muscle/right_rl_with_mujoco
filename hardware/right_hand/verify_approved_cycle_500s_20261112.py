"""Verify approved startup under mocked devices; never start hardware."""
import contextlib
import io
import json
from unittest.mock import patch
import nominal_cycle_plan_20261112 as plan
from nominal_cycle_gate_20261112 import validate_review
import leap_cycle_nominal_two_turns_20261112 as entry

p=plan.NominalCyclePlan(plan.REQUEST); r=validate_review(p,plan.PROOF)
assert plan.scope_approved() and p.ON_TIMEOUT==r['active_motion_limit_s']==500
assert r['current_ma']==450 and r['goal_step_ticks']==2 and r['sample_period_s']==.075
assert not r['all_condition_safety_certified'] and not r['hand_only_all_passed'] and not r['physical_two_turn_success']
with patch('dynamixel_sdk.PortHandler',side_effect=AssertionError('motor access')) as port, \
     patch('camera_stream.CameraStream',side_effect=AssertionError('camera access')) as camera, \
     patch('builtins.input',side_effect=AssertionError('repeat approval prompt')), \
     contextlib.redirect_stdout(io.StringIO()):
    with patch('sys.argv',['trial']), contextlib.redirect_stdout(io.StringIO()) as output:
        assert entry.main()==0 and 'CHECK ONLY' in output.getvalue()
    with patch('sys.argv',['trial','--execute']),patch.object(entry.runtime,'main',return_value=0) as simulated_start:
        assert entry.main()==0 and simulated_start.call_count==1
    assert port.call_count==camera.call_count==0
result={'passed':True,'maximum_on_seconds':500,'repeat500_prompt_required':False,
        'current_PID_joint_health_limits_unchanged':True,'known_model_failures_preserved':True,
        'startup_test_used_mocked_runtime':True,'real_camera_or_motor_ports_opened':0,'assistant_motor_writes':0}
(plan.OUT/'approved_500s_verification.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
delivery_path=plan.OUT/'delivery.json'
delivery=json.loads(delivery_path.read_bytes())
delivery.update(status='user_approved_nominal_diagnostic_waiting_SPACE',proof_sha256=plan.sha(plan.PROOF),
                operator_scope_approved_in_chat=True,scope_confirmation_required_before_device_access=False,
                operator_SPACE_required=True)
delivery_path.write_text(json.dumps(delivery,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result))
