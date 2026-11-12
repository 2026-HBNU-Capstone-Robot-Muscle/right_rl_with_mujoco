"""Compact saved-result inspection; no device access or simulation."""
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent / 'right_hand_rotation_20261105/canonical_body_grip'

for path in sorted((ROOT / 'placement_offsets').glob('*/review.json')):
    report = json.loads(path.read_bytes())
    print(json.dumps({
        'file': str(path.relative_to(ROOT)),
        'completed': len(report['cases']),
        'passed': sum(c['task_passed'] for c in report['cases']),
        'cases': [{
            'offsets': [c['forward_offset_mm'], c['side_offset_mm'], c['initial_yaw_offset_deg']],
            'passed': c['task_passed'], 'reasons': c['reasons'],
            'yaw_change_deg': None if not c['continuous'] else c['continuous']['regrasp_yaw_change_deg'],
            'center_m': None if not c['continuous'] else c['continuous']['final_center_world_m'],
        } for c in report['cases']],
    }, ensure_ascii=False))

result = ROOT.parents[1] / 'forward_hold_observe_20261105_040030_646453/result.json'
if result.exists():
    data = json.loads(result.read_bytes())
    rows = data.get('samples', [])
    print(json.dumps({'physical_result_keys': list(data), 'sample_count': len(rows),
                      'phases': dict(Counter(c.get('phase') for c in rows)),
                      'first_sample': rows[0] if rows else None,
                      'last_sample': rows[-1] if rows else None}, ensure_ascii=False))
