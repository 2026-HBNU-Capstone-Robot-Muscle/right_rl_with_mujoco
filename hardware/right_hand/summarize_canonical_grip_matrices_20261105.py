"""Print bounded current matrix results, including incomplete or failed work."""
import json
from pathlib import Path
from canonical_grip_task_review_20261105 import task_reasons
O=Path(__file__).resolve().parent/'right_hand_rotation_20261105/canonical_body_grip'
for tag in ('full_placement_v2','full_placement_v3_thumb12','empty_path','empty_path_v3_thumb12','placement_offsets','placement_offsets_v3_thumb12'):
    for p in sorted((O/tag).glob('*/review.json')):
        d=json.loads(p.read_bytes());cases=d['cases'];summary=[]
        for c in cases:
            if 'continuous' in c:why=task_reasons(c['continuous']);passed=not why and c['initial_placement']['passed']
            else:why=c.get('reasons',[]);passed=c.get('task_passed',c.get('passed',False))
            summary.append(dict(parameters={k:c[k] for k in ('gain_scale','friction','forward_offset_mm','side_offset_mm','initial_yaw_offset_deg') if k in c},passed=passed,reasons=why))
        print(json.dumps(dict(file=str(p.relative_to(O)),count=len(cases),passed=sum(c['passed'] for c in summary),failed=[c for c in summary if not c['passed']]),ensure_ascii=False))
