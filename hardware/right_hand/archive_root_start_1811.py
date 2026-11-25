import hashlib,json
from pathlib import Path
import numpy as np
r=Path(__file__).resolve().parent
out=r/'right_hand_start_20261019_1811';out.mkdir(exist_ok=True)
b=(r/'leap_roots_empty_result.json').read_bytes();data=json.loads(b)
assert data['timestamp']=='2026-10-19T18:11:01.488208+09:00'
assert data['write_events']==[] and data['all_torque_off_verified']
s=data['preflight_state']
assert not any(s['torque']) and not any(s['error']) and not any(s['watchdog'])
(out/'source_preflight.json').write_bytes(b)
old=json.loads((r/'right_hand_start_20261019/source_readonly.json').read_text())
before=np.array([x['position_ticks_signed'] for x in old['motors']]);after=np.array(s['position'])
comparison=[dict(motor_id=i+1,old=int(before[i]),current=int(after[i]),
                delta_ticks=int(after[i]-before[i]),delta_deg=float((after[i]-before[i])*360/4096)) for i in range(16)]
(out/'source_comparison.json').write_text(json.dumps(comparison,indent=2),encoding='utf-8')
print('SHA256',hashlib.sha256(b).hexdigest())
print('Current:',after.tolist())
