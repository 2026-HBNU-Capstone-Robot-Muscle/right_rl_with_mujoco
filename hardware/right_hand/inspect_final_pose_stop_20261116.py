"""Print only bounded diagnostics from final-pose logs; no device access."""
import json
from pathlib import Path
import numpy as np
from observed_book_model_policy import bounds
from loaded_quarter_reentry_20261025 import limits
ROOT=Path(__file__).resolve().parent
lo,hi=bounds();goal_lo,goal_hi=limits()
for folder in sorted(ROOT.glob('final_grip_pose_20*'),reverse=True)[:3]:
    path=folder/'result.json'
    if not path.exists():continue
    r=json.loads(path.read_bytes())
    print('RUN',folder.name)
    print('STATUS',json.dumps({k:r.get(k) for k in ('status','error','failed_phase','all_torque_off_verified','settings_restored','hardware_authorized')},ensure_ascii=False))
    print('KEYS',list(r))
    def visit(value,key='root'):
        if isinstance(value,dict):
            if 'position' in value and isinstance(value['position'],list) and len(value['position'])==16:
                a=np.asarray(value['position'])
                bad=[dict(id=i+1,ticks=float(a[i]),start_lower=float(lo[i]),start_upper=float(hi[i]),
                          goal_lower=float(goal_lo[i]),goal_upper=float(goal_hi[i])) for i in range(16) if not lo[i]<=a[i]<=hi[i]]
                print('STATE',key,'OUTSIDE',json.dumps(bad),'TORQUE',value.get('torque'),'POSITIONS',a.tolist())
                return
            for k,v in value.items():
                if k not in ('samples','camera_recording'):visit(v,key+'.'+k)
        elif isinstance(value,list):
            for i,v in enumerate(value[:4]):visit(v,key+'.'+str(i))
    visit(r)
