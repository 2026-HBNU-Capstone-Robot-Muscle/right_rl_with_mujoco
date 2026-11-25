import json,sys
from pathlib import Path
p=Path(sys.argv[1] if len(sys.argv)>1 else 'right_hand_rotation_20261024/reverse_empty_entry/placement_fixture_v1/screen.json')
r=json.loads(p.read_bytes())
for key in ('static_screens','dynamic_sensitivity'):
    print(key)
    for row in (r[key][:3] if key=='static_screens' else r[key]):print(json.dumps(row,ensure_ascii=False))
