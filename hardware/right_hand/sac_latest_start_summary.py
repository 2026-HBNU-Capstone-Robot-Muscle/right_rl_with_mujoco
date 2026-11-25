"""Print only the latest empty-start outcome; never opens hardware."""
import json
from pathlib import Path
root = Path(__file__).resolve().parent
path = root / 'leap_sac_start_empty_result.json'
d = json.loads(path.read_text(encoding='utf-8'))
rows = d.get('samples', [])
summary = {k:d.get(k) for k in ('timestamp', 'status', 'error', 'all_torque_off_verified',
                              'settings_restored', 'voltage_policy', 'voltage_events')}
summary.update(samples=len(rows), min_voltage_v=min((min(r['voltage_v']) for r in rows), default=None),
               peak_current_ma=max((abs(v) for r in rows for v in r['current_ma']), default=None),
               max_tracking_rad=max((max(r['tracking_rad']) for r in rows), default=None),
               final_tracking_rad=max(rows[-1]['tracking_rad']) if rows else None)
print(json.dumps(summary, indent=2))
(root/'sac_latest_start_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
