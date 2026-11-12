"""Compact hypothetical-reaction results; completed run and task pass differ."""
import json
from pathlib import Path
O=Path(__file__).resolve().parent/'right_hand_rotation_20261105/canonical_body_grip'
for p in sorted((O/'layer_reaction').glob('selected_v3_thumb12*/review.json')):
    d=json.loads(p.read_bytes())
    print(json.dumps(dict(gain=d['gain_scale'],friction=d['friction'],cases=[dict(torque=c['hypothetical_reaction_torque_nm'],safe=c['safety_passed'],passed=c['load_filter_passed'],yaw=c['max_yaw_change_deg'],shift=c['max_center_shift_mm'],reasons=c['reasons']) for c in d['cases']]),ensure_ascii=False))
