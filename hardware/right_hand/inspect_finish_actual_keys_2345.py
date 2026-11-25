import json
from pathlib import Path
r=json.loads(Path('quarter_finish_empty_20261025_234555_682505/result.json').read_bytes())
print({k:(len(v) if isinstance(v,(dict,list)) else v) for k,v in r.items() if k!='samples'})
