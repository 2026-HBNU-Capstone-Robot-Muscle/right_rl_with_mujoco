import json,numpy as np
from pathlib import Path
p=Path('right_hand_rotation_20261024/original_sequence_clearance/reverse_original_turn_evaluation.json')
d=json.loads(p.read_bytes());good=[r for r in d['runs'] if r.get('valid_start') and r['mode']=='native_motor']
for r in good:
    print('NATIVE',r['offset_mm'],r['friction'],'four_second_goal',r['final_four_second_hold_reached'],
        'error',r['final']['target_error_deg'],'gap_s',r['maximum_no_hand_contact_seconds'],
        'self_mm',r['maximum_hand_self_model_overlap_mm'],'initial',r['initial_cube_qpos'],
        'shift',r['final']['shift_mm'],'contacts',r['final']['contacts'])
