import json
from pathlib import Path
d=json.loads(Path('sac_id12_replan_probe_result.json').read_text())
for v in d['validation']:
    print('CANDIDATE',v['candidate']['id12_goal_cap_ticks'],v['candidate']['id11_offset_rad'])
    for r in v['checks']:
        if not r['passed']: print(r)
