"""Read saved empty rehearsal runs without opening any devices."""
import json
from pathlib import Path
from collections import Counter

ROOT = Path(__file__).resolve().parent
for folder in sorted(ROOT.glob('supported_yaw_empty_20261022_*')):
    print('\nRUN', folder.name)
    files = list(folder.glob('*.json'))
    print('FILES', [p.name for p in files])
    for path in files:
        if path.name in ('request.json', 'current_start_screen.json'):
            continue
        r = json.loads(path.read_bytes())
        if not isinstance(r, dict):
            continue
        if not any(k in r for k in ('status', 'error', 'samples')):
            continue
        print('RESULT', path.name, 'KEYS', list(r))
        print(json.dumps({k: v for k, v in r.items()
                          if not isinstance(v, (list, dict))}, ensure_ascii=False))
        for key in ('camera_recording', 'current_start_screen', 'initial_state', 'final_off_state'):
            value = r.get(key)
            if value is not None:
                print(key, json.dumps(value, ensure_ascii=False)[:3500])
        rows = r.get('samples', [])
        print('PHASES', dict(Counter(s.get('phase') for s in rows)))
        for row in rows[-3:]:
            print('LAST_SAMPLE', json.dumps(row, ensure_ascii=False)[:4200])
    log = folder / 'current_start_screen.log'
    if log.exists():
        print('SCREEN_LOG_TAIL', log.read_text(encoding='utf-8', errors='replace')[-4500:])
print('\nLATEST_ROOT')
r = json.loads((ROOT / 'leap_supported_yaw_empty_result.json').read_bytes())
print(json.dumps({k:v for k,v in r.items() if not isinstance(v,(list,dict))}, ensure_ascii=False))
