import json
from pathlib import Path
from collections import Counter
import numpy as np
import horizontal_quarter_plan_20261027 as p
root=Path(__file__).resolve().parent
for folder in sorted(root.glob('horizontal_quarter_trial_20261027_*'))[-2:]:
    r=json.loads((folder/'result.json').read_bytes());samples=r.get('samples',[])
    print(json.dumps(dict(folder=folder.name,error=r.get('error'),phases=dict(Counter(s['phase'] for s in samples)),all_off=r.get('all_torque_off_verified'),restored=r.get('settings_restored'),goal_events=r.get('goal_readback_events',[]),voltage_events=r.get('voltage_events',[]),camera=r.get('camera_recording',{}).get('folder'),root_curve=[dict(t=round(s['t'],3),goal=s['goal'][7],actual=s['actual'][7],current=s['current_ma'][7],min_v=min(s['voltage_v']),supply_end_v=s['voltage_v'][15],sum_ma=sum(abs(v) for v in s['current_ma'])) for s in samples if s['phase']==p.ROOTS][::10]),ensure_ascii=False))
    print('FILES',folder.name,[str(x.relative_to(folder)) for x in folder.iterdir()])
    print('CAMERA',json.dumps(r.get('camera_recording',{}),ensure_ascii=False)[:2400])
plan=p.HorizontalQuarterPlan(p.REQUEST);rows=plan.plan(plan.source_position())
for phase in dict(rows):
    end=[q for ph,q in rows if ph==phase][-1]
    print('ENDPOINT',phase,end.tolist())
