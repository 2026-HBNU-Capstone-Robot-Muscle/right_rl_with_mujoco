import json
from pathlib import Path
p=Path(__file__).resolve().parent/'readonly_recovery_v2_20261029_013344_758169/result.json'
r=json.loads(p.read_bytes());a,b=[s['state'] for s in r['samples'][:2]]
print(json.dumps(dict(goal_delta=[y-x for x,y in zip(a['goal'],b['goal'])],position_delta=[y-x for x,y in zip(a['position'],b['position'])],
    first_goal_minus_position=[x-y for x,y in zip(a['goal'],a['position'])],second_goal_minus_position=[x-y for x,y in zip(b['goal'],b['position'])],
    torque_first=a['torque'],torque_second=b['torque'],instructions=r['instruction_counts'])))
