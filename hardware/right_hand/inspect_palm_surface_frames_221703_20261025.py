import json,numpy as np
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/support_photo_review_221703'
r=json.loads((OUT/'cad_geometry.json').read_bytes());up=-np.array(r['palm_world_rotation'])[:,2]
for g in r['geoms']:
    if g['body']!='palm' or g['group']!=3:continue
    R=np.array(g['rotation']);n=R[:,2]
    print(g['name'],'center',np.round(g['center'],4).tolist(),'size',g['size'],'normal',np.round(n,4).tolist(),'normal_tilt_global',round(float(np.degrees(np.arccos(np.clip(abs(n[2]),0,1)))),3),'normal_tilt_book',round(float(np.degrees(np.arccos(np.clip(abs(n@up),0,1)))),3))
