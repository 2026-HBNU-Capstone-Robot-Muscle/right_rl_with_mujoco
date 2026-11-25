"""Extract one fully labeled offline target; no motor access."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'prepared_cube_review_20261022'
source=OUT/'grasp_approach_screen.json';doc=json.loads(source.read_bytes())
case=next(r for r in doc['cases'] if r['relief_ticks']==32 and r['order']=='simultaneous')
assert case['static_self_mm']==case['static_support_mm']==0 and not case['outside_native_range']
assert len(case['trials'])==7 and all(t['max_self_mm']==t['max_support_mm']==0 for t in case['trials'])
target=case['goals_id_order'][-1]
out=dict(format='offline_tilted_cube_grasp_target_v1',target_ticks=target,
    source_file=source.name,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    mapping_version='all_finger_orders_user_confirmed_20261018_v2',
    fixed_ring_ids=[13,15,16],ring_relief_id=14,ring_relief_ticks=32,
    static_self_overlap_mm=0,static_support_overlap_mm=0,
    simulated_cube_start_candidates=7,cube_registration_verified=False,
    physical_grasp_verified=False,physical_cube_rotation_verified=False,
    hardware_execution_allowed=False,role='Source target for a separately screened EMPTY approach check.',
    limitation='Some synthesized cube starts move up to 14.5mm and do not end in a three-finger grip; no loaded rollout authorized.')
p=OUT/'empty_grasp_target.json';p.write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print(hashlib.sha256(p.read_bytes()).hexdigest());print(target)
