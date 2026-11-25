"""Print small saved geometry summaries, not full telemetry."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
g=json.loads((ROOT/'prepared_cube_review_20261022/measured_geometry.json').read_bytes())
o=np.array(g['palm_world_position']);R=np.array(g['palm_world_rotation'])
for b in g['boxes_world_m']:
    if b['body']=='palm':
        p=(np.array(b['vertices'])-o)@R*1000
        print(b['name'],np.round(p.min(0),1).tolist(),np.round(p.max(0),1).tolist())
for name in ('sac_cube55_resets.json','sac_support_cad_reference_result.json'):
    d=json.loads((ROOT/name).read_bytes())
    if 'candidates' in d:
        for r in d['candidates']:
            q=r['qpos'];print('synthetic_cube',q[-7:], 'local_center_mm',np.round((np.array(q[-7:-4])-o)@R*1000,1).tolist())
    else:
        print('CAD:',{k:{x:v[x] for x in ('min_mm','max_mm','area_planes_mm2')} for k,v in d['geoms'].items()})
