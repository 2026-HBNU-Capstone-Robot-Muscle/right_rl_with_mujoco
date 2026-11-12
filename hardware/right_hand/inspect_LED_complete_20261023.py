import json
from collections import Counter
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parent
for p in sorted(root.glob('bus_echo_off_20261023_*/result.json'))[-3:]:
    r=json.loads(p.read_bytes());rows=r.get('reads',[])
    print(p.parent.name)
    print({k:r.get(k) for k in ('status','error','diagnostic_revision','all_torque_off_verified',
        'led_restoration_verified','goal_writes','torque_writes','drive_setting_writes','eeprom_writes')})
    print('phases',r.get('phases',[]),'read_failures',r.get('read_failures',[]),
        'echo_mismatches',r.get('echo_mismatches',[]))
    if rows:
        print('samples',len(rows),'read_ms',np.percentile([x['read_ms'] for x in rows],[0,50,95,100]).tolist(),
            'min_V',min(min(x['voltage']) for x in rows)/10,
            'position_total_range',np.ptp([x['position'] for x in rows],axis=0).tolist(),
            'last_pos',rows[-1]['position'])
    if r.get('diagnostic_revision')==2:
        print('write_counts',dict(Counter((x['phase'],x['purpose']) for x in r.get('led_writes',[]))))
        print('keys',list(r))
