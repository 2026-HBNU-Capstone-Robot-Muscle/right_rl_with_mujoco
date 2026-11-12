import json
from pathlib import Path
from collections import Counter
p=Path(__file__).resolve().parent/'right_hand_rotation_20261023/supported_loaded_prefix_256.json'
r=json.loads(p.read_bytes())
for mode in (False,True):
    rows=[s for s in r['trials'] if s['control']==mode]
    bad=[s for s in rows if s['fault']]
    print('CONTROL',mode,'FAULTS',dict(Counter(s['fault'] or 'none' for s in rows)))
    for s in bad[:16]:print({k:v for k,v in s.items() if k not in ('trace','final_contacts')})
    if bad:print('FIRST_FAULT_STEP',min(s['last_step'] for s in bad))
