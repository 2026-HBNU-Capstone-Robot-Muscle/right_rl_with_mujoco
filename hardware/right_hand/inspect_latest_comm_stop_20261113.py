import json
from pathlib import Path
import dynamixel_sdk.robotis_def as defs
ROOT=Path(__file__).resolve().parent
p=ROOT/'support_first_trial_20261113_041949_350339/result.json'
r=json.loads(p.read_bytes())
print('SDK_COMM_MINUS3002',[name for name in dir(defs) if name.startswith('COMM_') and getattr(defs,name)==-3002])
for key in r:
    if any(word in key for word in ['failure','recovery','resynchron','failed','last_issued','read_fail','primary_motion','torque_off','restored']):
        value=r[key]
        print(key,json.dumps(value,ensure_ascii=False)[:2200])
samples=r.get('samples',[])
print('SAMPLES',len(samples))
if samples:
    last=samples[-1];print('LAST_KEYS',list(last))
    print('LAST_SAMPLE',json.dumps(last,ensure_ascii=False)[:2000])
