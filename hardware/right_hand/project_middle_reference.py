"""Project offline model hypotheses onto an existing image; never opens devices."""
import argparse,itertools,json
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out',default='middle_reference_review_20261017')
parser.add_argument('--photo',default='pose_pair_readonly_20261017_061429_504276/paired_photo.png')
args=parser.parse_args()
out=ROOT/args.out
report=json.loads((out/'analysis.json').read_text())
photo=ROOT/args.photo
im=cv.imdecode(np.fromfile(photo,np.uint8),cv.IMREAD_COLOR)
cal=np.load(ROOT/'camera1_720.npz');K=cal['K'];dist=cal['dist']
assert im.shape[:2]==(int(cal['height']),int(cal['width']))
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[41],hand_id=40,required_cube_count=1)
corners,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
decoded=[] if ids is None else ids.ravel().tolist()
result=dict(decoded_ids=decoded,registration_verified=False,marker_size_m_assumed=.0282,solutions=[])
if 40 in decoded:
    s=.0282;obj=np.array([[-s/2,s/2,0],[s/2,s/2,0],[s/2,-s/2,0],[-s/2,-s/2,0]],np.float64)
    pts=np.asarray(corners[decoded.index(40)]).reshape(4,2).astype(np.float64)
    _,rvs,tvs,_=cv.solvePnPGeneric(obj,pts,K,dist,flags=cv.SOLVEPNP_IPPE_SQUARE)
    signs=np.array(list(itertools.product((-1,1),repeat=3)))
    edges=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(signs[i]!=signs[j])==1]
    for solution,(rv,tv) in enumerate(zip(rvs,tvs)):
        def project(p):return cv.projectPoints(np.array(p,np.float64),rv,tv,K,dist)[0].reshape(-1,2)
        residual=float(np.linalg.norm(project(obj)-pts,axis=1).mean())
        result['solutions'].append(dict(index=solution,marker_residual_px=residual,rvec=rv.ravel().tolist(),tvec=tv.ravel().tolist()))
        for index,case in enumerate(report['cases'][:3]):
            show=im.copy()
            for b in case['middle_boxes_marker_m']:
                pp=project(b['vertices'])
                if not np.isfinite(pp).all() or abs(pp).max()>10000:continue
                for i,j in edges:cv.line(show,tuple(np.rint(pp[i]).astype(int)),tuple(np.rint(pp[j]).astype(int)),(0,255,0),1,cv.LINE_AA)
            for ident,point in case['joints_marker_m'].items():
                p=np.rint(project([point])[0]).astype(int)
                if abs(p).max()>10000:continue
                cv.circle(show,tuple(p),4,(0,0,255),-1)
                cv.putText(show,ident,(int(p[0]+5),int(p[1]-5)),cv.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1,cv.LINE_AA)
            cv.rectangle(show,(0,0),(1280,55),(0,0,0),-1)
            cv.putText(show,f'PROVISIONAL IPPE {solution}: {residual:.2f}px | '+case['hypothesis'],(8,22),cv.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
            cv.putText(show,'Old marker-to-palm assumptions. NOT fitted or calibrated. Green=middle model boxes.',(8,45),cv.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
            cv.imencode('.png',show)[1].tofile(str(out/f'overlay_{solution}_{index}.png'))
else:result['status']='id40_not_detected_no_projection'
(out/'projection.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
