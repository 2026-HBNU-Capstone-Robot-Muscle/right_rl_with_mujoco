"""Plot fixed CAD side-plane vertices to check photo landmark correspondence."""
import json
from pathlib import Path
import numpy as np
import cv2 as cv
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024'/'book_support_sensitivity';OUT.mkdir(parents=True,exist_ok=True)
r=json.loads((ROOT/'pose_pair_readonly_20261024_045449_430810/native_geometry.json').read_bytes())
v=np.array(next(g['vertices_world_m'] for g in r['meshes'] if g['geom']=='palm_visual'))
p=(v-np.array(r['palm_position_world_m']))@np.array(r['palm_rotation_world'])*1000
canvas=np.full((700,1100,3),255,np.uint8);records=[]
for row,(side,y) in enumerate((('outer_side_plane',-97.),('opposite_extreme',float(p[:,1].max())))):
    chosen=p[abs(p[:,1]-y)<.002]
    cv.putText(canvas,f'CAD palm side Y={y:.3f} mm; +X toward fingers (left)',(25,row*350+32),cv.FONT_HERSHEY_SIMPLEX,.7,(30,30,30),2)
    for index,point in enumerate(chosen):
        x=350-int(point[0]*5.2);z=row*350+250+int(point[2]*4.8)
        cv.circle(canvas,(x,z),6,(40,60,200),-1)
        cv.putText(canvas,f'{index}: X{point[0]:.1f} Z{point[2]:.1f}',(x+9,z-4),cv.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),1)
    records.append(dict(side=side,y_mm=y,points_palm_mm=chosen.tolist()))
ok,encoded=cv.imencode('.png',canvas);assert ok
(OUT/'cad_palm_side_vertices.png').write_bytes(encoded.tobytes())
(OUT/'cad_palm_side_vertices.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
print(json.dumps(records,indent=2))
