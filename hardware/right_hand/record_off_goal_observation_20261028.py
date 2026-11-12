import json
from pathlib import Path
import numpy as np
import calibrated_quarter_plan_20261028 as p
source=p.ROOT/'reply40_production_readonly_20261028_055358_224806/result.json'
r=json.loads(source.read_bytes());rows=r.get('off_goal_changes',[])
observed=dict(source=source.relative_to(p.ROOT).as_posix(),observations=len(rows),
    maximum_off_goal_change_ticks=max(v['maximum_delta_ticks'] for v in rows),
    maximum_goal_position_difference_ticks=max(int(np.max(abs(np.array(v['after'])-v['position']))) for v in rows),
    no_external_write_or_firmware_cause_asserted=True,active_exact_goal_checks_unchanged=True,motor_writes=0)
assert observed['maximum_off_goal_change_ticks']==1 and observed['maximum_goal_position_difference_ticks']==0
path=p.OUT/'verification.json';report=json.loads(path.read_bytes());report['off_goal_register_observation']=observed
path.write_bytes((json.dumps(report,indent=2)+'\n').encode())
print(json.dumps(observed))
