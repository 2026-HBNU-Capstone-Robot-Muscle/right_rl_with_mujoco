"""Small readout of saved hold-current timestamps and placement metadata."""

import json
from pathlib import Path

root = Path(__file__).resolve().parent
for filename in ('leap_support_capture_result.json', 'leap_anchor_capture_result.json'):
    data = json.loads((root / filename).read_text(encoding='utf-8'))
    rows = [r for r in data['samples'] if r['phase'] == 'place_cube']
    print(filename, 'enabled_at=', data.get('enabled_at'),
          'placement=', data.get('cube_placement_camera_snapshot'),
          'sample_t_first_last=', (rows[0]['t'], rows[-1]['t']))
    for r in (rows[0], rows[20], rows[len(rows) // 2], rows[-1]):
        print('  t=', r['t'], 'ID2/8/12/16 current=',
              [r['current_ma'][i-1] for i in (2, 8, 12, 16)])
