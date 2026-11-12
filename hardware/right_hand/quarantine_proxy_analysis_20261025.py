"""Invalidate affected derived analyses without editing raw operator recordings."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261024'
rows=[]
for folder,names in (
    ('original_sequence_clearance',[
        'adapted_reference_path.json','adapted_reference_path_v2.json',
        'horizontal_reference_primitive.json','horizontal_reference_primitive_to6.json',
        'horizontal_cube_dynamics.json','horizontal_cube_dynamics_to6.json','whole_ninety_finish_search.json',
        'regrasp_configuration_search.json','regrasp_configuration_search_v2.json','regrasp_configuration_search_v3.json',
        'free_cube_regrasp_review.json','free_cube_regrasp_review_v3.json','regrasp_timing_control_review.json',
        'fingertip_collision_visual_inventory.json']),
    ('middle_ring_side_review',['paired_native_geometry.json','progress_20261025.json']),
    ('original_sequence_review',['source_and_path_audit.json','review_summary.json'])):
    for name in names:
        p=BASE/folder/name
        if p.exists():rows.append(dict(file=p.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
report=dict(reason='Visual mesh clones lost their class without explicit type=mesh; some housing proxies compiled as spheres.',
    affected_derived_analyses=rows,discard_path_approval_claims_from_these_hashes=True,
    operator_raw_recordings_affected=False,source_recorded_sequence_affected=False,
    native_model_and_actual_control_code_affected=False,hardware_commands_sent_from_affected_plans=0,
    corrected_model_revision='explicit_mesh_type_v1_20261025',
    requirement='Recheck exact compiled mesh types, transforms and a known recorded middle/ring overlap before reusing proposals.')
p=BASE/'proxy_analysis_superseded_20261025.json'
if p.exists():
    old=json.loads(p.read_bytes())
    known={(r['file'],r['sha256']) for r in old['affected_derived_analyses']}
    report['affected_derived_analyses']=old['affected_derived_analyses']+[r for r in rows if (r['file'],r['sha256']) not in known]
p.write_text(json.dumps(report,indent=2),encoding='utf-8')
print('Superseded derived hashes',len(report['affected_derived_analyses']),'raw operator results unchanged')
