"""Print compact status of latest anchor capture and its vision folder."""

import json
from pathlib import Path

root = Path(__file__).resolve().parent
for path in sorted(root.glob('sequence_vision_20261009_23*')):
    result_path = path / 'result.json'
    if not result_path.exists():
        continue
    result = json.loads(result_path.read_text(encoding='utf-8'))
    print(path.name, {'files': [p.name for p in path.glob('*.png')],
          'summary': result.get('summary'), 'by_phase': result.get('by_phase'),
          'error': result.get('error')})
result = json.loads((root / 'leap_anchor_two_face_result.json').read_text(encoding='utf-8'))
print('latest_motor', {key: result.get(key) for key in (
    'timestamp', 'status', 'phase', 'error', 'failed_phase',
    'all_torque_off_verified', 'settings_restored', 'vision_by_phase')})
