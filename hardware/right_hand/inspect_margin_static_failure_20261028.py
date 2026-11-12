import json
from pathlib import Path
p=Path(__file__).resolve().parent/'right_hand_rotation_20261028/reconnected_170143_margin_v4/empty_recovery_screen/review.json'
r=json.loads(p.read_bytes())
print(json.dumps(r['static_cases'][0]))
