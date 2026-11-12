"""Compact saved-screen diagnosis, no devices."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for name in ('supported_yaw_empty_20261022_155339_886571','supported_yaw_empty_20261022_155659_749370'):
    f=ROOT/name
    r=json.loads((f/'result.json').read_bytes())
    p=json.loads((f/'current_start_screen.json').read_bytes())
    print(name,'writes',r.get('write_events'),'raw',p['start_ticks'])
    fields=('contact_clear_pip_travel_ticks','self_escape_complete_at',
            'self_escape_max_unapproved_overlap_mm','self_escape_max_step_increase_mm',
            'max_support_increase_mm','unapproved_native_range_violation_rad')
    for field in fields:
        valid=[s for s in p['static_screens'] if s.get(field) is not None]
        worst=max(valid,key=lambda s:s[field])
        print(field,'max',worst[field],'offset',worst['offset_ticks'])
    bad=[s for s in p['static_screens'] if s['self_escape_max_unapproved_overlap_mm']>1e-6 or not s['self_escape_complete']]
    print('STATIC_BAD',len(bad))
    print('NOMINAL',json.dumps(p['static_screens'][0]))
    print('DYNAMIC',json.dumps(p['dynamic_sensitivity']))
