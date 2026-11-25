import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for folder in sorted(ROOT.glob('horizontal_quarter_trial_20261027_*'))[-2:]:
 r=json.loads((folder/'result.json').read_bytes());chosen={k:v for k,v in r.items() if any(s in k for s in ('voltage','readback','read_failure','recovery')) and k not in ('samples','write_events')}
 print(json.dumps(dict(folder=folder.name,fields=chosen,last_writes=r.get('write_events',[])[-12:]),ensure_ascii=False))
