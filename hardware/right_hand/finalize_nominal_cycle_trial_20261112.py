"""Rebind reviewed code without granting or changing operator approval."""
import json
from nominal_cycle_plan_20261112 import OUT,PROOF,REQUEST,NominalCyclePlan,hashes,scope_approved
from nominal_cycle_gate_20261112 import make_review,validate_review

old=json.loads(PROOF.read_bytes()); p=NominalCyclePlan(REQUEST)
report=make_review(p,old['empty_recovery'],hashes())
PROOF.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
validate_review(p,PROOF)
status={'status':'scope_approved_waiting_operator_SPACE' if scope_approved() else 'prepared_for_operator_scope_confirmation','operator_scope_approved':scope_approved(),
        'assistant_started_motion':False,'nominal_loaded_cycle_model_passed':True,
        'empty_static_cases':len(report['empty_recovery']['static_cases']),
        'empty_dynamic_cases':len(report['empty_recovery']['dynamic_cases']),
        'all_condition_safety_certified':False,'hand_only_all_passed':report['hand_only_all_passed'],
        'proposed_maximum_on_seconds':500,'hardware_execution_requires_operator_typing500_and_SPACE':not scope_approved(),
        'operator_SPACE_required':True,
        'physical_two_turn_success':False,'new_live_SAC':False}
(OUT/'READY.json').write_text(json.dumps(status,indent=2)+'\n',encoding='utf-8')
print(json.dumps(status))
