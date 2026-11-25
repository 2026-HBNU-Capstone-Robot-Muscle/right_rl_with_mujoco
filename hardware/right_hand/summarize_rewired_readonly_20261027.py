import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
source=ROOT/'repeat_off_readonly_20261027_162601_077240/result.json'
r=json.loads(source.read_bytes());before=json.loads((ROOT/'horizontal_quarter_trial_20261027_155220_214230/result.json').read_bytes())['initial']
print(json.dumps(dict(source=source.relative_to(ROOT).as_posix(),status=r['status'],minimum_voltage_v=min(v/10 for s in r['snapshots'] for v in s['voltage']),maximum_voltage_v=max(v/10 for s in r['snapshots'] for v in s['voltage']),voltages_by_id_v=[v/10 for v in r['snapshots'][-1]['voltage']],before_off_voltage_by_id_v=[v/10 for v in before['voltage']],max_temperature_c=max(v for s in r['snapshots'] for v in s['temperature']),maximum_drift_ticks=r['maximum_drift_ticks'],motor_writes=r['servo_writes'],torque_changes=r['torque_changes'])))
