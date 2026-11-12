"""Conditional photo overlays; uses saved images, no camera or motor access."""
import itertools, json
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'full_mapping_pose_review_20261019'
report=json.loads((OUT/'analysis.json').read_text(encoding='utf-8'))
cal=np.load(ROOT/'camera1_720.npz');K=cal['K'];dist=cal['dist']
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[41],hand_id=40,required_cube_count=1)
s=.0282;obj=np.array([[-s/2,s/2,0],[s/2,s/2,0],[s/2,-s/2,0],[-s/2,-s/2,0]],np.float64)
signs=np.array(list(itertools.product((-1,1),repeat=3)))
edges=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(signs[i]!=signs[j])==1]
colors={'if_':(255,140,50),'mf_':(40,235,40),'rf_':(30,180,240),'th_':(80,40,240),'palm':(210,210,210)}
results=[]
for row in report['poses']:
    im=cv.imdecode(np.fromfile(ROOT/row['source']/'paired_photo.png',np.uint8),cv.IMREAD_COLOR)
    assert im.shape[:2]==(int(cal['height']),int(cal['width']))
    corners,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
    ids=[] if ids is None else ids.ravel().tolist()
    result=dict(pose=row['label'],ids=ids,registration_verified=False,solutions=[])
    if 40 in ids:
        pts=np.asarray(corners[ids.index(40)]).reshape(4,2).astype(np.float64)
        _,rvs,tvs,_=cv.solvePnPGeneric(obj,pts,K,dist,flags=cv.SOLVEPNP_IPPE_SQUARE)
        for n,(rv,tv) in enumerate(zip(rvs,tvs)):
            def project(v):return cv.projectPoints(np.asarray(v,np.float64),rv,tv,K,dist)[0].reshape(-1,2)
            residual=float(np.linalg.norm(project(obj)-pts,axis=1).mean())
            result['solutions'].append(dict(index=n,marker_residual_px=residual,rvec=rv.ravel().tolist(),tvec=tv.ravel().tolist()))
            for key,case in row['interpretations'].items():
                show=im.copy()
                for box in case['boxes_marker_m']:
                    pp=project(box['vertices'])
                    if not np.isfinite(pp).all() or np.max(abs(pp))>10000:continue
                    color=next((v for prefix,v in colors.items() if box['body'].startswith(prefix)),(180,180,180))
                    for i,j in edges:cv.line(show,tuple(np.rint(pp[i]).astype(int)),tuple(np.rint(pp[j]).astype(int)),color,1,cv.LINE_AA)
                cv.rectangle(show,(0,0),(1280,56),(0,0,0),-1)
                cv.putText(show,f'{key} | IPPE {n} marker residual {residual:.2f}px',(8,22),cv.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
                cv.putText(show,'UNVERIFIED marker-to-palm transform. Box overlays are NOT a calibrated fit.',(8,45),cv.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
                label='corrected' if key.startswith('User') else 'legacy'
                cv.imencode('.png',show)[1].tofile(str(OUT/f'{row["label"]}_overlay_{n}_{label}.png'))
    results.append(result)
(OUT/'projection.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(json.dumps(results,indent=2))
