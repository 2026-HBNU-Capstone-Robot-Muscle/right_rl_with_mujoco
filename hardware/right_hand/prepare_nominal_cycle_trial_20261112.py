"""Prepare files and offline evidence; never grant operator scope approval."""
import json
from nominal_cycle_plan_20261112 import ROOT,OUT,SOURCE,SOURCE_REVIEW,REQUEST,MANIFEST,APPROVAL,NominalCyclePlan,candidate,sha

OUT.mkdir(exist_ok=True)
if REQUEST.exists() or MANIFEST.exists() or APPROVAL.exists(): raise RuntimeError('Preserve prepared proposal')
MANIFEST.write_text(json.dumps({'source_goals_sha256':sha(SOURCE),'source_review_sha256':sha(SOURCE_REVIEW),
    'loaded_goals':5089,'control_seconds':.075,'goal_step_ticks':2,'new_SAC_training':False,
    'nominal_model_success_not_physical_or_robustness_success':True},indent=2)+'\n',encoding='utf-8')
seed=ROOT/'right_hand_rotation_20261112/stage2_work_grip/request.json'
old=json.loads(seed.read_bytes())
request={'plan_version':NominalCyclePlan.PLAN_VERSION,'motor_ids_in_sim_order':old['motor_ids_in_sim_order'],
         'off_state':old['off_state'],'source':seed.relative_to(ROOT).as_posix(),'source_sha256':sha(seed),
         'seed_is_saved_OFF_not_current_device_measurement':True,'runtime_must_remeasure_actual_OFF_before_any_write':True,
         'hardware_execution_allowed':False}
REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
APPROVAL.write_text(json.dumps({'user_explicitly_approved':False,'maximum_on_seconds':500,
    'source_goals_sha256':sha(SOURCE),'acknowledges_nominal_only_and_hand_only_failures':False},indent=2)+'\n',encoding='utf-8')
p=NominalCyclePlan(REQUEST); rows=p.plan(p.source_position()); z=candidate()
report={'status':'complete_cycle_diagnostic_proposal_not_authorized','loaded_goals':len(z['goals']),
        'whole_goals_from_saved_seed':len(rows),'command_seconds_from_saved_seed':len(rows)*.075,
        'conservative_budget_seconds_from_saved_seed':(len(rows)+40)*.075+20,
        'maximum_on_seconds_proposed':500,'previous_on_seconds':300,'operator_scope_approved':False,
        'holds_seconds_each':54*.075,'same_first1417_not_claimed':True,'devices_opened':0}
(OUT/'proposal.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report))
