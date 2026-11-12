import json
from pathlib import Path
p=Path(__file__).resolve().parent/'horizontal_quarter_trial_20261028_045326_636152/result.json'
r=json.loads(p.read_bytes())
print(json.dumps({k:v for k,v in r.items() if k not in ('samples','settings_before','settings_after') and not isinstance(v,(list,dict))}))
print('SAMPLE',json.dumps(r['samples'][0])[:5500])
print('KEYS',list(r))
