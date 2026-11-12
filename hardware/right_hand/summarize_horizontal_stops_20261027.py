import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for name in ('horizontal_quarter_trial_20261027_143018_624423','horizontal_quarter_trial_20261027_143147_188487'):
    r=json.loads((ROOT/name/'result.json').read_bytes())
    print(json.dumps(dict(folder=name,error=r.get('error'),sample_keys=list(r['samples'][-1]),voltage_events=r.get('voltage_events',[]),goal_events=r.get('goal_readback_events',[])),ensure_ascii=False))
