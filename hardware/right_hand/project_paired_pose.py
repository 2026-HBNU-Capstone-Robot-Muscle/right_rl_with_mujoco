"""Offline provisional marker projection; never an installed registration."""
import json,itertools
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector
root=Path(__file__).resolve().parent;out=root/'paired_pose_audit_20261017'
geo=json.loads((out/'geometry.json').read_text())
photo=root/Path(geo['photo']).parent.name/Path(geo['photo']).name
image=cv.imdecode(np.fromfile(photo,np.uint8),cv.IMREAD_COLOR)
cal=np.load(root/'camera1_720.npz');K=cal['K'];dist=cal['dist']
assert image.shape[:2]==(int(cal['height']),int(cal['width']))
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[41],hand_id=40,required_cube_count=1)
corners,ids,_=det.detectMarkers(cv.cvtColor(image,cv.COLOR_BGR2GRAY))
decoded=[] if ids is None else ids.ravel().tolist()
print('IDs',decoded,flush=True)
if 40 not in decoded:
    (out/'marker_projection.json').write_text(json.dumps(dict(status='id40_not_detected',decoded_ids=decoded,
        registration_verified=False,reason='No provisional projection produced; paired encoder/photo data remain valid.'),indent=2))
    raise SystemExit('ID40 unavailable; no projection produced. Paired capture remains valid.')
s=.0282;obj=np.array([[-s/2,s/2,0],[s/2,s/2,0],[s/2,-s/2,0],[-s/2,-s/2,0]],np.float64)
img=np.asarray(corners[decoded.index(40)]).reshape(4,2).astype(np.float64)
ret,rvecs,tvecs,errs=cv.solvePnPGeneric(obj,img,K,dist,flags=cv.SOLVEPNP_IPPE_SQUARE)
signs=np.array(list(itertools.product((-1,1),repeat=3)))
edges=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(signs[i]!=signs[j])==1]
colors={'if_':(255,140,50),'mf_':(50,255,50),'rf_':(40,180,255),'th_':(80,60,255)}
reports=[]
for k,(rv,tv) in enumerate(zip(rvecs,tvecs)):
    show=image.copy()
    def project(pts):return cv.projectPoints(np.array(pts,np.float64),rv,tv,K,dist)[0].reshape(-1,2)
    for b in geo['boxes']:
        prefix=b['body'][:3]
        if prefix not in colors:continue
        pts=project(b['vertices_marker'])
        if not np.isfinite(pts).all() or abs(pts).max()>10000:continue
        for i,j in edges:cv.line(show,tuple(np.rint(pts[i]).astype(int)),tuple(np.rint(pts[j]).astype(int)),colors[prefix],1,cv.LINE_AA)
    jointprojection={ident:project([v])[0].tolist() for ident,v in geo['joint_axes_marker'].items()}
    for ident,p in jointprojection.items():
        xy=tuple(np.rint(p).astype(int));cv.circle(show,xy,4,(255,255,255),-1)
        cv.putText(show,ident,(xy[0]+5,xy[1]-5),cv.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1,cv.LINE_AA)
    residual=float(np.linalg.norm(project(obj)-img,axis=1).mean())
    cv.rectangle(show,(0,0),(1280,54),(0,0,0),-1)
    cv.putText(show,f'PROVISIONAL marker solution {k}: residual {residual:.2f}px | NOT calibration',(10,22),cv.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
    cv.putText(show,'Fixed old marker-to-palm ruler assumptions; no pose fitting or servo changes',(10,45),cv.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
    cv.imencode('.png',show)[1].tofile(str(out/f'provisional_overlay_{k}.png'))
    reports.append(dict(solution=k,rvec=rv.ravel().tolist(),tvec=tv.ravel().tolist(),mean_corner_error_px=residual,joints_px=jointprojection))
(out/'marker_projection.json').write_text(json.dumps(dict(marker_corners=img.tolist(),solutions=reports,registration_verified=False),indent=2))
print(json.dumps([dict(solution=r['solution'],error=r['mean_corner_error_px']) for r in reports]),flush=True)
