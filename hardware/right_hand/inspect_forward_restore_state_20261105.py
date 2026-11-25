"""Compare saved OFF settings, including interrupted camera cleanup."""
import json
from pathlib import Path
R=Path(__file__).resolve().parent
fields=('d_gain','i_gain','p_gain','current_cap','acceleration','velocity','watchdog','torque','error')
for name in ('forward_hold_observe_20261105_030930_516359','forward_hold_observe_20261105_035757_923198','forward_hold_observe_20261105_040030_646453'):
    d=json.loads((R/name/'result.json').read_bytes());out={}
    for key in ('preflight_state','initial'):
        if key in d:out[key]={f:d[key].get(f) for f in fields}
    if d.get('final_off_snapshots'):out['final_off']={f:d['final_off_snapshots'][-1].get(f) for f in fields}
    print(json.dumps(dict(folder=name,states=out,restoration_errors={k:v for k,v in d.items() if 'restor' in k or 'cleanup' in k},read_confirmation_events=d.get('read_recovery_confirmation_events',[])),ensure_ascii=False))
