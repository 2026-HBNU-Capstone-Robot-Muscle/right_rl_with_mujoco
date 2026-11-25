from pathlib import Path
import json,hashlib
r=Path(__file__).resolve().parent
b=(r/'leap_roots_empty_result.json').read_bytes();v=json.loads(b)
assert v['write_events']==[] and v['all_torque_off_verified']
s=v['preflight_state']
assert not any(s['torque']) and not any(s['error']) and not any(s['watchdog'])
assert s['position'][1]==2056
o=r/'right_hand_start_20261019_thumb_shift';o.mkdir(exist_ok=True)
(o/'source_preflight.json').write_bytes(b)
print(v['timestamp'],hashlib.sha256(b).hexdigest(),s['position'])
