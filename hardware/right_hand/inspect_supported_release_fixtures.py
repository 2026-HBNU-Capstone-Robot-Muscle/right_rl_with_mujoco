"""Inspect saved fixture outcomes only."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent/'right_hand_rotation_20261022'
for f in sorted(ROOT.glob('fixture_supported_*')):
    req=json.loads((f/'request.json').read_bytes())
    raw=req['off_state']['position']
    paths=sorted(f.glob('windows_wsl_screen_*.json'))
    print(f.name,'raw',raw)
    if not paths:continue
    p=json.loads(paths[-1].read_bytes())
    print('PROOF',paths[-1].name,p['accepted'])
    for row in p['dynamic_sensitivity']:
        print('DYNAMIC',row)
    bad=[s for s in p['static_screens'] if s.get('self_escape_max_unapproved_overlap_mm',0)>1e-6]
    print('STATIC_BAD',len(bad),bad[:1])
