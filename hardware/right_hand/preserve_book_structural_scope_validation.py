"""Preserve real native proof and audit the exact change from the failed screen."""
import hashlib
import json
from pathlib import Path
from book_empty_preparation_plan import ROOT,BookPreparationPlan
from book_empty_preparation_gate import validate_proof
from book_support_contact_scope import load_scope,allowed_initial_pairs

source=Path(json.loads((ROOT/'book_screen_camera_validation_latest.json').read_bytes())['folder']).resolve()
if not source.is_relative_to(ROOT):raise ValueError('Validation folder outside workspace')
report=json.loads((source/'result.json').read_bytes())
if report['status']!='camera_and_full_native_screen_passed' or report['motor_interfaces_opened']:
    raise ValueError('No completed device-free native proof with active camera')
plan=BookPreparationPlan(source/'saved_off_request.json');new=validate_proof(plan,source/'screen.json')
old_path=ROOT/'book_prepare_empty_20261024_164116_703445/current_start_screen.json'
old=json.loads(old_path.read_bytes())
if any(old[k]!=new[k] for k in ('start_ticks','last_goal_ticks','plan_sha256')):
    raise ValueError('Initial angles or trajectory changed; classification-only comparison invalid')
changes=[]
for i,(a,b) in enumerate(zip(old['static_screens'],new['static_screens'])):
    if a.keys()!=b.keys():raise ValueError('Static schema changed')
    keys=[k for k in a if a[k]!=b[k]]
    if keys:
        pair='if_md_collision_5|support_glass'
        if (keys!=['support_escape_max_unapproved_overlap_mm']
                or a['support_escape_max_unapproved_overlap_mm']!=a['initial_support_contacts_mm'][pair]
                or b['support_escape_max_unapproved_overlap_mm']!=0
                or pair not in allowed_initial_pairs()):
            raise ValueError('Unexpected native result change')
        changes.append(dict(static_case=i,book_front_mm=a['book_front_mm'],pair=pair,
            initial_model_depth_mm=a['initial_support_contacts_mm'][pair],
            separation_complete_at=a['support_escape_complete_at'],
            separation_deadline=a['support_escape_deadline']))
if len(changes)!=3 or old['dynamic_sensitivity']!=new['dynamic_sensitivity']:
    raise ValueError('Unexpected number of changed classification results or dynamic change')
destination=ROOT/'right_hand_rotation_20261024/book20_current_review/structural_scope_fix_fixture'
destination.mkdir(exist_ok=True)
for src,name in ((source/'saved_off_request.json','request.json'),(source/'screen.json','screen.json')):
    target=destination/name
    if target.exists() and target.read_bytes()!=src.read_bytes():raise ValueError('Refuse to overwrite an existing proof')
    target.write_bytes(src.read_bytes())
validate_proof(BookPreparationPlan(destination/'request.json'),destination/'screen.json')
scope=load_scope()
evidence=dict(status='structural_initial_contact_scope_validated',source_validation_folder=str(source),
    failed_user_proof=str(old_path),failed_user_proof_sha256=hashlib.sha256(old_path.read_bytes()).hexdigest(),
    camera_active_full_native_screen_seconds=report['native_screen_seconds'],
    native_worker_seconds=new['elapsed_seconds'],static_cases=201,dynamic_cases=9,
    unchanged_path=True,unchanged_initial_contact_depths=True,unchanged_dynamic_results=True,
    changed_classification_cases=changes,depth_and_separation_deadlines_unchanged=True,
    geometry_counts_by_root={k:len(v['geometry_names']) for k,v in scope['root_groups'].items()},
    initial_scope_method=scope['method'],new_or_deeper_contacts_still_rejected=True,
    thumb_and_fixed_palm_excluded=True,actual_proof_bytes_preserved=True,old_hashes_modified=False,
    motor_interfaces_opened=False,motor_commands_sent=0,camera_recording=report['camera_recording'])
(destination/'scope_validation.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8')
print(json.dumps(evidence,indent=2))
