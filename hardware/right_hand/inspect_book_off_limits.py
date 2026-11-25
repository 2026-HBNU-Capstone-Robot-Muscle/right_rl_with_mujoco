"""Report native-limit differences in saved read-only data, never correct them."""
import json
from pathlib import Path
root=Path(__file__).resolve().parent
r=json.loads((root/'book_settled_off_readonly_20261024_173647_632324/result.json').read_bytes())
s=r['preflight_state'];limits=json.loads((root/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
bad=[dict(id=i+1,actual=v,lower=limits['lower_ticks'][i],upper=limits['upper_ticks'][i])
    for i,v in enumerate(s['position']) if not limits['lower_ticks'][i]<=v<=limits['upper_ticks'][i]]
print(json.dumps(dict(position=s['position'],outside=bad,torque=s['torque'],watchdog=s['watchdog'])))
