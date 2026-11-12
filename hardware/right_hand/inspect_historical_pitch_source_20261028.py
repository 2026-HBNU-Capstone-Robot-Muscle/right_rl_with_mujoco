"""Bounded saved-data inspection; no device imports or access."""
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / 'loaded_thumb_rotation_20261026_135418_442474'

def main():
    r = json.loads((SOURCE / 'result.json').read_bytes())
    print('KEYS', list(r))
    print('SUMMARY', json.dumps({k: r[k] for k in (
        'status', 'plan_version', 'request', 'current_start_screen', 'source',
        'cube_placement_confirmed', 'cube_removal_confirmed',
        'all_torque_off_verified', 'settings_restored', 'sample_period_s'
    ) if k in r}, ensure_ascii=False))
    for key in ('write_events', 'samples', 'planned_goals'):
        values = r.get(key)
        print(key, type(values).__name__, len(values) if hasattr(values, '__len__') else values)
        if isinstance(values, list) and values:
            print('FIRST', json.dumps(values[0], ensure_ascii=False)[:1900])
            print('LAST', json.dumps(values[-1], ensure_ascii=False)[:1900])
            print('PHASES', dict(Counter(v.get('phase', '') for v in values if isinstance(v, dict))))
    for p in (SOURCE / 'current_start_request.json', SOURCE / 'request.json'):
        if p.is_file():
            data = json.loads(p.read_bytes())
            print('REQUEST', p.name, json.dumps(data, ensure_ascii=False)[:2600])

if __name__ == '__main__':
    main()
