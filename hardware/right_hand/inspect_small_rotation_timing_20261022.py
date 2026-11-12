import json
from pathlib import Path
import cube_small_rotation_plan as p
from test_cube_small_rotation_trial import Tests
for path in sorted((p.ROOT/'right_hand_rotation_20261022').glob('fixture*/request.json')):
    plan=p.SmallRotationPlan(path);rows=plan.plan(plan.source_position())
    phases={}
    for phase,q in rows:phases[phase]=phases.get(phase,0)+1
    print(path.parent.name,len(rows),phases)
bus,r,error,entered=Tests().execute()
print('Error',str(error),'goal attempts',bus.goal_attempts,'entered',entered[-2:])
print('last sample', {k:v for k,v in r.get('samples',[{}])[-1].items() if k in ('t','phase')})
print('enabled at',r.get('enabled_at'),'operator pause',r.get('operator_pause_seconds_total'))
