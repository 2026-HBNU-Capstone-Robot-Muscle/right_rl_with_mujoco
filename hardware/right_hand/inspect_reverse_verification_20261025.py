import json
from pathlib import Path
d=json.loads(Path('right_hand_rotation_20261024/original_sequence_clearance/reverse_original_native_verification.json').read_bytes())
motion=[r for r in d['runs'] if r.get('valid_start') and r.get('mode')=='moving_path'];control=[r for r in d['runs'] if r.get('mode')=='matched_stationary_control']
print('moving_valid',len(motion),'passing',sum(r['simulated_orientation_and_retention_pass'] for r in motion),'controls',len(control),'control_pass',sum(r['simulated_orientation_and_retention_pass'] for r in control))
r=next(r for r in motion if r['gain_scale']==1 and r['friction']==.8 and r['offset_from_nominal_mm']==[0,0])
print({k:v for k,v in r.items() if k not in ('frames','initial_state')})
