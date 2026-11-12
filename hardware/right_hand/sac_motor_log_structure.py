"""Inspect saved motor telemetry structure without touching COM hardware."""

import json
from pathlib import Path

root = Path(__file__).resolve().parent
for name in ('leap_sequence_flip_result.json', 'leap_support_capture_result.json',
             'leap_anchor_capture_result.json'):
    path = root / name
    if not path.exists():
        continue
    data = json.loads(path.read_text(encoding='utf-8'))
    print(name, 'status=', data.get('status'), 'top_keys=', list(data)[:25])
    for key in ('samples', 'telemetry', 'motion_samples'):
        rows = data.get(key)
        if isinstance(rows, list):
            print(' ', key, 'count=', len(rows), 'first_keys=', list(rows[0])[:25] if rows else [])
