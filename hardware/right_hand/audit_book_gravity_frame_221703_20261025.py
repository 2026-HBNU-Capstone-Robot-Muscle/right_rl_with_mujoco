import json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/support_photo_review_221703'
cad=json.loads((OUT/'cad_geometry.json').read_bytes());R=np.array(cad['palm_world_rotation']);up=-R[:,2]
if up[2]<0:up=-up
def tilt(v):return float(np.degrees(np.arccos(np.clip(np.dot(v,up)/np.linalg.norm(v),-1,1))))
rows=[]
for g in cad['geoms']:
    if g['name'] in ('if_px_collision','mf_px_collision'):
        n=-np.array(g['rotation'])[:,2]
        rows.append(dict(geom=g['name'],normal_world=n.tolist(),native_world_up_tilt_deg=float(np.degrees(np.arccos(np.clip(n[2],-1,1)))),tilt_relative_to_book_plane_deg=tilt(n)))
result=dict(native_book_plane_tilt_from_gravity_horizontal_deg=float(np.degrees(np.arccos(np.clip(up[2],-1,1)))),book_up_normal_native_world=up.tolist(),support_surface_normals=rows,
    physical_book_level_assumption_not_new_measurement=True,model_frame_hypothesis_only=True,hardware_commands_sent=0)
(OUT/'book_gravity_frame_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))
