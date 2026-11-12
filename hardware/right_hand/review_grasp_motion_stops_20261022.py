"""Summarize the two failed recoveries without opening devices."""
import hashlib
import json
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for name in ('cube_grasp_trial_20261022_023904_356527','cube_grasp_trial_20261022_024003_945968'):
    folder=ROOT/name;data=(folder/'result.json').read_bytes();r=json.loads(data)
    samples=r.get('samples',[]);initial=r['initial']['position'];last=samples[-1]
    images=json.loads((folder/'camera/frames.json').read_bytes());phases={}
    for f in images['frames']:phases.setdefault(f['phase'],[]).append(f['image'])
    report=dict(folder=name,sha256=hashlib.sha256(data).hexdigest(),status=r['status'],error=r['error'],
        all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
        initial_ticks=initial,final_actual_ticks=last['actual'],final_goal_ticks=last['goal'],
        off_snapshots=[s['position'] for s in r.get('final_off_snapshots',[])],
        minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
        peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
        samples=len(samples),motion_goal_writes=sum(w['address']==116 for w in r['write_events']),
        images_by_phase={p:dict(count=len(v),first=v[0],last=v[-1]) for p,v in phases.items()},
        id12_trace=[{k:s[k][11] for k in ('goal','actual','current_ma','tracking_rad')}|{'t':s['t']}
            for i,s in enumerate(samples) if i%10==0 or i>=len(samples)-20])
    (folder/'stop_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
