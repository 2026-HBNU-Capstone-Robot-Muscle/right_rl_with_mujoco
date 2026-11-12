"""Summarize saved pinch diagnostics without devices."""
import json
from pathlib import Path
root=Path(__file__).resolve().parent/'right_hand_rotation_20261026/yaw_after_flip_v1'
data=json.loads((root/'pinch_approach_replay.json').read_bytes())
rows=[r for r in data['cases'] if r.get('stopped') is None and r.get('static_rejected_at') is None]
for r in sorted(rows,key=lambda v:v['translation_mm'])[:8]:print(json.dumps(r))
