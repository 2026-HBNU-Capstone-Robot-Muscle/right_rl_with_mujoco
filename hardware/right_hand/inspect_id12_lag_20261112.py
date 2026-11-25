"""Read only the recorded ID12 stop window and saved frame references."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'nominal_two_turn_trial_20261112_060746_992877'
r=json.loads((folder/'result.json').read_bytes());ss=r['samples'];end=ss[-1]['t']
tail=[s for s in ss if end-10<=s['t']<=end]
out={'last10s_ID12':[{'t':s['t'],'goal':s['goal'][11],'actual':s['actual'][11],
                       'current_ma':s['current_ma'][11],'voltage_v':s['voltage_v'][11],
                       'torque':s['torque'][11],'error':s['error'][11]} for s in tail[::12]+[tail[-1]]],
     'stop':r['error']}
frames=json.loads((folder/'camera/frames.json').read_bytes())['frames'];enabled=r['enabled_at']
out['images']=[]
for t in (end-8,end-3,end-.15):
    f=min(frames,key=lambda v:abs(v['camera_read_monotonic']-enabled-t))
    out['images'].append({'image':(folder/'camera'/f['image']).relative_to(ROOT).as_posix(),
                          't':f['camera_read_monotonic']-enabled})
print(json.dumps(out,ensure_ascii=False))
