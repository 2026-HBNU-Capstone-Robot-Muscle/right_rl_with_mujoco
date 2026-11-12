"""Project diagnostic model boxes on stored pictures; no parameter fitting."""
import itertools
import json
from pathlib import Path
import cv2 as cv
import numpy as np
ROOT=Path(__file__).resolve().parent
folder=ROOT/'cube_supported_rotation_trial_20261023_034640_750784'
r=json.loads((folder/'image_joint_contact_audit.json').read_bytes())
with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
signs=np.array(list(itertools.product((-1,1),repeat=3)))
edges=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(signs[i]!=signs[j])==1]
colors={'if_':(255,130,50),'mf_':(30,220,30),'rf_':(30,170,245),'th_':(50,50,240),'palm':(180,180,180)}
for f in r['snapshots']:
    im=cv.imdecode(np.frombuffer((folder/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    h=f['hypotheses'][0];R=np.array(h['world_to_camera_rotation']);tv=np.array(h['world_to_camera_translation']);rv=cv.Rodrigues(R)[0]
    def project(points):return cv.projectPoints(np.array(points,np.float64),rv,tv,K,dist)[0].reshape(-1,2)
    show=im.copy()
    for box in f['boxes']:
        pts=project(box['vertices'])
        if not np.isfinite(pts).all() or np.max(abs(pts))>6000:continue
        col=next((v for k,v in colors.items() if box['body'].startswith(k)),(200,200,200))
        for i,j in edges:cv.line(show,tuple(np.rint(pts[i]).astype(int)),tuple(np.rint(pts[j]).astype(int)),col,1,cv.LINE_AA)
    for name,p in f['joint_anchors_world'].items():
        xy=np.rint(project([p])[0]).astype(int)
        if max(abs(xy))>6000:continue
        col=next((v for k,v in colors.items() if name.startswith(k)),(255,255,255))
        cv.circle(show,tuple(xy),3,col,-1)
        cv.putText(show,name,tuple(xy+[5,-5]),cv.FONT_HERSHEY_SIMPLEX,.4,col,1,cv.LINE_AA)
    cv.rectangle(show,(0,0),(1280,53),(0,0,0),-1)
    cv.putText(show,'Existing marker-to-model registration: diagnostic, NOT calibrated',(8,22),cv.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
    cv.putText(show,'Blue index / green middle / gold ring / red thumb',(8,43),cv.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
    target=folder/(Path(f['image']).stem+'_conditional_model.jpg')
    cv.imencode('.jpg',show)[1].tofile(str(target));print(target.name)
    print('ROOTS',{k:np.round(project([v])[0],1).tolist() for k,v in f['joint_anchors_world'].items() if k.endswith('mcp') or k=='th_cmc'})
