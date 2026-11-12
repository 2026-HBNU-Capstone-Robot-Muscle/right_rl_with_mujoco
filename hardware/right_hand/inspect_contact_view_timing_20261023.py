"""Inspect saved camera/sample clocks, with no devices."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_grasp_trial_20261023_041546_404551'
r=json.loads((FOLDER/'result.json').read_bytes())
s=r['samples'];frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
group=[x for x in s if x['phase']=='remove_cube']
diff=np.diff([x['t'] for x in group])
peaks=np.argsort(diff)[-8:][::-1]
print('ERROR',r.get('error'))
print('LAST_SAMPLE',json.dumps({k:v for k,v in group[-1].items() if k not in ('actual','goal','current_ma','voltage_v','tracking_rad')})[:1600])
print('CLOCK_GAPS',[(round(float(diff[i]),4),group[i]['t'],group[i+1]['t']) for i in peaks])
print('LAST_SAMPLES',[(x['t'],x.get('watchdog'),x.get('read_seconds')) for x in group[-8:]])
print('TIMING_FIELDS',{k:v for k,v in r.items() if ('time' in k or 'elapsed' in k or 'enabled' in k) and not isinstance(v,(list,dict))})
print('LAST_CAMERA',[(x['image'],x['camera_read_monotonic'],x['enabled_at']) for x in frames[-12:]])
blob=(ROOT/'empty_trial_camera.py').read_text(encoding='utf-8').splitlines()
for i,line in enumerate(blob):
    if any(k in line for k in ('def check','imshow','waitKey','def _record','def finish')):print('GUI_SOURCE',i+1,line.strip())
