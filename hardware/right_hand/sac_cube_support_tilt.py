"""Compare observed green-face normals with the provisional model support plane."""
import json
from pathlib import Path

import numpy as np

from sac_support_geometry_audit import quat_rotation

ROOT=Path(__file__).resolve().parent
normal=np.array([.3395997574287447,0.,.9405700424499699])
rows=[]
for label,name in (('old','sac_registration_anchor_result.json'),
                   ('latest','sac_registration_support_photo_diagnostic.json')):
    entry=json.loads((ROOT/name).read_text(encoding='utf-8'))
    R=quat_rotation(entry['cube_quat_world_candidate'])
    green=R@np.array([0.,-1.,0.])
    angle=float(np.degrees(np.arccos(np.clip(normal@green,-1,1))))
    rows.append({'pose':label,'green_face_normal_world':green.tolist(),
                 'angle_to_support_normal_deg':angle})
result={'rows':rows,'registration_verified':False,'hardware_authorized':False}
(ROOT/'sac_cube_support_tilt_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2))
