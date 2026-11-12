import json
from pathlib import Path
p=Path('right_hand_rotation_20261024/original_sequence_clearance/CAD_tip_contact_comparison_v2.json')
d=json.loads(p.read_bytes())
for r in d['free_cube_dynamics']:
    last=r['stages'][-1]
    print('MODEL',r.get('model',r.get('contact_model')),'keys',list(r)[:7], 'friction',r['friction'],
        'retained',r['final_retained'],'center',last['cube_center_m'],'quat',last['cube_quat_wxyz'],
        'error',last['target_errors_deg'])
