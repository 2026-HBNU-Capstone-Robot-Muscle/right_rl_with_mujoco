import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'reply40_production_readonly_20261028_055358_224806/result.json').read_bytes())
rows=r.get('off_goal_changes',[])
print(json.dumps(dict(events=len(rows),max_goal_change_ticks=max((v['maximum_delta_ticks'] for v in rows),default=0),
    maximum_goal_position_difference_ticks=max((int(np.max(abs(np.array(v['after'])-v['position']))) for v in rows),default=0),last_position=r['snapshots'][-1]['position'])))
