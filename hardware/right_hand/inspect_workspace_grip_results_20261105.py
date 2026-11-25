"""Compact static endpoint summary; no controller or device imports."""
import json
from pathlib import Path
O=Path(__file__).resolve().parent/'right_hand_rotation_20261105/canonical_body_grip'
for p in sorted((O/'sliding_workspace_grip').glob('*/review.json')):
    d=json.loads(p.read_bytes())
    if d['proposal_accepted']:print(json.dumps(dict(path=str(p.relative_to(O)),goal=d['goal'],translation=d['translation_world_mm'],finger_goals=d.get('grasp_finger_goal_ticks',d.get('middle_goal_ticks')),preload=d.get('model_only_preload_proposals',[]),contacts=d['contacts']),ensure_ascii=False))
