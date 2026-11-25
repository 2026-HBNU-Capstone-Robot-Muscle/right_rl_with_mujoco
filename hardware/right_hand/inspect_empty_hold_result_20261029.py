import json
from pathlib import Path
root=Path(__file__).resolve().parent
p=root/'empty_hold_comm_20261029_042239_202853/result.json';r=json.loads(p.read_bytes())
print('KEYS',json.dumps({k:len(v) if isinstance(v,(dict,list)) else v for k,v in r.items() if k not in ('joint_mapping',)},ensure_ascii=False))
for key in ('samples','read_events','read_recovery_events','two_complete_health_confirmations','communication_recoveries','fast_OFF_setting_readbacks'):
    v=r.get(key);print(key, json.dumps((v[:2]+v[-1:]) if isinstance(v,list) else v,ensure_ascii=False)[:7000])
print('SAMPLESHAPE',json.dumps(r.get('samples',[{}])[0],ensure_ascii=False)[:4000])
