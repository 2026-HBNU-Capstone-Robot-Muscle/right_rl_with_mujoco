"""Print bounded task/safety metrics from the current device-free scouts."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
KEYS = ('passed', 'stop', 'last_phase', 'last_sample', 'placed_to_turn_deg',
        'yaw_before_release_deg', 'final_yaw_from_placement_deg',
        'max_self_mm', 'max_cube_mm', 'max_tracking_rad',
        'final_cube_supported_by_hand', 'center_translation_mm')
for name in ('RF14_-64', 'RF14_-128'):
    path = ROOT / 'right_hand_rotation_20261029/counter_ring_clearance' / name / 'cube_nominal/review.json'
    if path.is_file():
        data = json.loads(path.read_bytes())
        print(json.dumps(dict(name=name, all_passed=data.get('all_passed'),
            cases=[{k:r.get(k) for k in KEYS} for r in data.get('cases', [])]), ensure_ascii=False))
    else:
        print(name, 'not completed')
