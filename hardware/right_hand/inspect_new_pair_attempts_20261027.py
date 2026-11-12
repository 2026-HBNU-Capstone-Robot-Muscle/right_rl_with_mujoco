import json,collections
from pathlib import Path
import numpy as np
from new_pair_grasp_plan_20261027 import ROOT,REQUEST,OPEN_PATH,NewPairGraspPlan
reference=NewPairGraspPlan(REQUEST).source_position();opened=np.load(OPEN_PATH)['open_goal']
for folder in sorted(ROOT.glob('new_pair_grasp_20261027_*')):
    if not folder.is_dir():continue
    result=json.loads((folder/'result.json').read_bytes());request=folder/'fresh_off_request.json';r=json.loads(request.read_bytes()) if request.exists() else {};state=r.get('off_state',{});raw=np.array(state.get('position',[]));samples=result.get('samples',[])
    print(json.dumps(dict(folder=folder.name,status=result.get('status'),error=result.get('error'),phase=result.get('failed_phase'),writes=len(result.get('write_events',[])),off=result.get('all_torque_off_verified'),restored=result.get('settings_restored'),phases=dict(collections.Counter(s['phase'] for s in samples)),start=raw.tolist(),from_previous=[dict(id=i+1,ticks=int(v)) for i,v in enumerate(raw-reference) if abs(v)>4] if len(raw)==16 else [],from_open=(raw-opened).tolist() if len(raw)==16 else [],last_command=result.get('final_command'),off_end=result.get('final_off_snapshots',[])[-1].get('position') if result.get('final_off_snapshots') else None),ensure_ascii=False))
