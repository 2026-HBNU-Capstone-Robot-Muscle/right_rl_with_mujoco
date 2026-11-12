from pathlib import Path
import json
for p in sorted(Path('.').glob('leap_roots_empty_20261019*.json'))[-4:]:
    r=json.loads(p.read_text(encoding='utf-8'))
    print(p.name,r.get('error'), 'OFF',r.get('all_torque_off_verified'),'restored',r.get('settings_restored'))
    for e in r.get('write_events',[]):
        if e['address']==116:
            print(' goal_write',e['requested'],'readback',e.get('readback'))
        if e['address']==64:print(' torque_write',e['requested'])
    print('keys',list(r))
