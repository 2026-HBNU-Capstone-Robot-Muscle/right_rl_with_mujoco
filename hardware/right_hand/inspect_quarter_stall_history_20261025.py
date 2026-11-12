"""Evidence around the ID8 stop; only saved data are read."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
folder=ROOT/'reverse_quarter_path_empty_20261025_082749_355962'
r=json.loads((folder/'result.json').read_bytes());s=r['samples']
for second in range(8,34):
    a=min(s,key=lambda v:abs(v['t']-second))
    if a['phase']!='stage_6_empty_quarter_reference_path':continue
    print('ID8',round(a['t'],3),'goal',a['goal'][7],'actual',a['actual'][7],'mA',a['current_ma'][7])
for seconds in (1.0,2.0,4.0,8.0):
    rows=[a for a in s if a['t']>=s[-1]['t']-seconds]
    print('WINDOW',seconds,'samples',len(rows),'goal_span',max(a['goal'][7] for a in rows)-min(a['goal'][7] for a in rows),
        'actual_span',max(a['actual'][7] for a in rows)-min(a['actual'][7] for a in rows),
        'current_range',[min(a['current_ma'][7] for a in rows),max(a['current_ma'][7] for a in rows)])
frames=json.loads((folder/'camera/frames.json').read_bytes())
print('FRAME_SCHEMA',type(frames).__name__,str(frames)[:350])
if isinstance(frames,dict):frames=frames['frames']
for second in (20.,28.,32.4):
    f=min(frames,key=lambda a:abs(a['camera_read_monotonic']-(r['enabled_at']+second)))
    print('SELECTED_FRAME',second,f)
latest=json.loads((ROOT/'reverse_quarter_path_empty_20261025_082933_354374/fresh_off_request.json').read_bytes())
print('LATEST_OFF',latest['off_state']['position'])
