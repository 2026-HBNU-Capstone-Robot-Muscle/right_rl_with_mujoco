import json
from pathlib import Path
for name in ('right_hand_rotation_20261024/support_photo_review_221703/initial_review.json','right_hand_rotation_20261024/support_photo_review_221703/provisional_cad_alignment_yaw.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
    p=Path(name);r=json.loads(p.read_bytes());print(name)
    if 'initial_review' in name or 'alignment_yaw' in name:print(json.dumps(r,ensure_ascii=False)[:8500])
    else:print(json.dumps({k:v for k,v in r.items() if not isinstance(v,(list,dict))},ensure_ascii=False)[:8500]);print('COMPOSITE_KEYS',[k for k,v in r.items() if isinstance(v,(list,dict))])
