"""Publish source changes and a fresh request, without opening any devices."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def replace(name,old,new,count=1):
    path=ROOT/name;text=path.read_text(encoding='utf-8')
    if text.count(old)!=count:raise RuntimeError('Unexpected source: '+name)
    path.write_text(text.replace(old,new),encoding='utf-8')

replace('horizontal_quarter_motion_runner_20261027.py',
    'from horizontal_preload_confirmation_20261027 import confirm_off_preload',
    'from horizontal_preload_confirmation_20261027 import confirm_off_preload\nfrom horizontal_placement_settle_20261027 import confirm as confirm_placement')
replace('horizontal_quarter_motion_runner_20261027.py',
    "                for _ in range(getattr(p,'CHECKPOINT_SETTLE_SAMPLES',1)):\n                    state,_,cycle_start=observe('placement_checkpoint');finish_cycle(cycle_start)\n                if np.any(abs(state['position']-command)>getattr(p,'CHECKPOINT_MAX_ERROR_TICKS',4096)):\n                    raise RuntimeError('Open preparation has not settled within its placement tolerance; no loaded target advance')",
    "                confirmation=[]\n                for _ in range(p.CHECKPOINT_SETTLE_SAMPLES):\n                    state,_,cycle_start=observe('placement_checkpoint')\n                    confirmation.append(dict(position=state['position'].copy(),goal=state['goal'].copy(),\n                        sample_index=len(report.get('samples',[]))-1))\n                    finish_cycle(cycle_start)\n                confirm_placement(confirmation,command,report)")
replace('horizontal_quarter_plan_20261027.py',"horizontal_quarter_trial_v6_current'","horizontal_quarter_trial_v7_stable_placement'")
replace('horizontal_quarter_plan_20261027.py','reviewed_horizontal_quarter_experiment_v6_current_book_numeric_20261027','reviewed_horizontal_quarter_experiment_v7_stable_placement_20261027')
replace('horizontal_quarter_plan_20261027.py','HORIZONTAL QUARTER-TURN v6:','HORIZONTAL QUARTER-TURN v7: stable OPEN check;')
replace('horizontal_quarter_plan_20261027.py',
    "EXTRA_FILES+=('rewired_initial_pair_scope_20261027.py','horizontal_thumb_book_numeric_20261027.py')",
    "EXTRA_FILES+=('rewired_initial_pair_scope_20261027.py','horizontal_thumb_book_numeric_20261027.py')\nfrom horizontal_placement_evidence_20261027 import FILES as PLACEMENT_FILES\nEXTRA_FILES+=('horizontal_placement_settle_20261027.py','horizontal_placement_evidence_20261027.py',\n    'verify_placement_bound_geometry_20261027.py','screen_weaker_roots_loaded_20261027.py')+PLACEMENT_FILES")
replace('horizontal_quarter_gate_20261027.py',
    'from horizontal_thumb_book_numeric_20261027 import review as thumb_book_review',
    'from horizontal_thumb_book_numeric_20261027 import review as thumb_book_review\nfrom horizontal_placement_evidence_20261027 import review as placement_review')
replace('horizontal_quarter_gate_20261027.py','empty_root_motion_policy=dict(ROOT_POLICY),',
    'empty_root_motion_policy=dict(ROOT_POLICY),stable_placement_review=placement_review(),',count=2)

import horizontal_quarter_plan_20261027 as p
from horizontal_placement_evidence_20261027 import review
from observed_book_model_policy import healthy_off
evidence=review();source=ROOT/'repeat_off_readonly_20261027_224731_974099/result.json';data=json.loads(source.read_bytes())
assert data['status']=='off_snapshots_saved' and data['maximum_drift_ticks']<=2
assert data['servo_writes']==0 and data['torque_changes']==0 and len(data['snapshots'])==3
for s in data['snapshots']:healthy_off(s)
p.OUT.mkdir(exist_ok=False)
request=dict(plan_version=p.HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(p.SIM_TO_MOTOR_IDS) if hasattr(p,'SIM_TO_MOTOR_IDS') else [8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1],
    origin='stable_measured_open_checkpoint_fresh_off_no_device_writes',source=source.relative_to(ROOT).as_posix(),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),off_state=data['snapshots'][-1],hardware_execution_allowed=False)
p.REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
plan=p.HorizontalQuarterPlan(p.REQUEST);rows=plan.plan(plan.source_position())
print(json.dumps(dict(status='fresh_v7_request_prepared_model_proof_pending',goals=len(rows),
    geometry_cases=evidence['geometry']['cases'],weak_loaded_cases=evidence['weaker_response']['required_cases'],
    weak_yaw_range=evidence['weaker_response']['yaw_range_deg'],motor_writes=0)))
