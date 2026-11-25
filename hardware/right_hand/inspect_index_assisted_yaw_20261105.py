"""Compact read-only progress/review for the index-assisted offline search."""
import json
from pathlib import Path
O=Path(__file__).resolve().parent/'right_hand_rotation_20261105/canonical_body_grip/index_assisted_yaw'
for p in sorted(O.glob('*/review.json')):
    try:d=json.loads(p.read_bytes())
    except json.JSONDecodeError:continue
    best=d['cases'][d['best_index']]
    print(json.dumps(dict(path=str(p.relative_to(O)),cases=len(d['cases']),safe=sum(c['dynamic']['safety_passed'] for c in d['cases']),best_yaw=best['whole_yaw_deg'],best_true90_orientation_error=best['orientation_error_to_true90_deg'],best_tilt=best['up_tilt_deg'],best_target90=best['target90_direction_filter_passed'],best_delta=best['deltas_by_motor'],best_contacts=best['dynamic']['final_contact_N'])))
