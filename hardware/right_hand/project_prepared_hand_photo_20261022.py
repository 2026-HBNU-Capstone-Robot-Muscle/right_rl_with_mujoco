"""Conditional historical-transform overlays on SAVED photographs, not calibration."""
import itertools
import json
from pathlib import Path
import cv2 as cv
import numpy as np
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'prepared_cube_review_20261022'
r=json.loads((OUT/'placement_review.json').read_text(encoding='utf-8'))
g=json.loads((OUT/'measured_geometry.json').read_text())
im=cv.imdecode(np.frombuffer((ROOT/r['source']/'prepared_empty.jpg').read_bytes(),np.uint8),cv.IMREAD_COLOR)
with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
signs=np.array(list(itertools.product((-1,1),repeat=3)))
edges=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(signs[i]!=signs[j])==1]
colors={'if_':(255,130,50),'mf_':(30,220,30),'rf_':(30,170,245),'th_':(50,50,240),'palm':(180,180,180)}
for h in sorted(r['historical_bridge_diagnostic']['hypotheses'],key=lambda h:h['green_up_error_relative_measured_gravity_deg'])[:3]:
    R=np.array(h['world_to_camera_rotation']);tv=np.array(h['world_to_camera_translation']);rv=cv.Rodrigues(R)[0]
    def project(points):return cv.projectPoints(np.array(points,np.float64),rv,tv,K,dist)[0].reshape(-1,2)
    show=im.copy()
    for box in g['boxes_world_m']:
        pts=project(box['vertices'])
        if not np.isfinite(pts).all() or np.max(abs(pts))>6000:continue
        col=next((v for k,v in colors.items() if box['body'].startswith(k)),(200,200,200))
        for i,j in edges:cv.line(show,tuple(np.rint(pts[i]).astype(int)),tuple(np.rint(pts[j]).astype(int)),col,1,cv.LINE_AA)
    for name,p in g['joint_anchors_world_m'].items():
        xy=np.rint(project([p])[0]).astype(int)
        if max(abs(xy))>6000:continue
        col=next((v for k,v in colors.items() if name.startswith(k)),(255,255,255))
        cv.circle(show,tuple(xy),4,col,-1)
        cv.putText(show,name,tuple(xy+[5,-5]),cv.FONT_HERSHEY_SIMPLEX,.38,col,1,cv.LINE_AA)
    tag=''.join(map(str,h['ippe_indices']))
    cv.rectangle(show,(0,0),(1280,55),(0,0,0),-1)
    cv.putText(show,f'Conditional old wrist/palm bridge {tag}; NOT calibrated',(8,22),cv.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
    cv.putText(show,'Blue index / green middle / gold ring / red thumb',(8,43),cv.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
    cv.imencode('.jpg',show)[1].tofile(str(OUT/f'conditional_hand_overlay_{tag}.jpg'))
    print(tag,{k:np.round(project([v])[0],1).tolist() for k,v in g['joint_anchors_world_m'].items() if k.endswith('mcp') or k=='th_cmc'})
