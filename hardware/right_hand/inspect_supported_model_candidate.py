import json
from pathlib import Path
p=Path(__file__).resolve().parent/'right_hand_rotation_20261022/supported_yaw_result.json'
r=json.loads(p.read_bytes())
print('KEYS',list(r))
print(json.dumps({k:v for k,v in r.items() if not isinstance(v,(list,dict))},indent=2))
for row in r.get('validation',[]):
    print(json.dumps({k:v for k,v in row.items() if k not in ('initial_qpos','trace','initial_ctrl')},indent=2)[:2000])
