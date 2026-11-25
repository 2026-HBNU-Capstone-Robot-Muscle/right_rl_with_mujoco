import json
from horizontal_quarter_plan_20261027 import OUT
r=json.loads((OUT/'empty_recovery_screen/review.json').read_bytes())
print(json.dumps(dict(first_static=r['static_cases'][0],worst_static=max(r['static_cases'],key=lambda v:v['max_self_mm']),start=r['start_ticks']),indent=2))
