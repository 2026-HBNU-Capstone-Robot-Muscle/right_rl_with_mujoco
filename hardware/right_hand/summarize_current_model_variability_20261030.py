import json
from pathlib import Path
R=Path(__file__).resolve().parent
for f in (-15,-10,-5):
 p=R/f'right_hand_rotation_20261030/single_forward/full_loaded_seated_front_{f}/review.json';r=json.loads(p.read_bytes());print('FRONT',f)
 for c in r['cases']:print({k:c.get(k) for k in ('gain_scale','friction','passed','forward_from_placement_deg','yaw_before_release_deg','center_translation_mm')})
for f in ('.7','1'):
 p=R/f'right_hand_rotation_20261030/hold_material_sensitivity/tip_{"0"+f if f.startswith(".") else f}/hold_sensitivity_review.json'
 if p.exists():
  r=json.loads(p.read_bytes());print('MATERIAL',r['tip_sliding_coefficient'],r['modeled_forward_from_placement_deg'],r['all_hold_probes_passed'])
