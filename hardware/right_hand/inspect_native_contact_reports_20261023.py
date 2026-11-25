import json
from pathlib import Path
root=Path(__file__).resolve().parent
folder=root/'slow_grasp_demo_20261023_182105_573825'
p=folder/'on_native_geometry_review.json'
r=json.loads(p.read_bytes())
print('KEYS',list(r))
print('BRANCH_SUMMARIES',json.dumps(r.get('branch_summaries'),indent=2))
for row in r.get('rows',[]):
    if row['source'].endswith('grid_diagnostic') and row['palm_branch']==0:
        print('GRID_CONTACTS',row);break
for row in r.get('representatives',[])[:2]:
    print('REP',{k:v for k,v in row.items() if k not in ('tip_boxes','cube_vertices_world_m')})
    for box in row.get('tip_boxes',[]):print('TIP',{k:v for k,v in box.items() if k!='vertices_world_m'})
print('GEOMETRY_SCRIPTS',[p.name for p in root.glob('*.py') if 'grasp' in p.name and ('geometry' in p.name or 'native' in p.name)])
print('PIVOT',json.loads((root/'right_hand_rotation_20261023/ring_pivot_candidate_search.json').read_bytes()))
