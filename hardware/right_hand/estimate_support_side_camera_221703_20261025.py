"""Provisional image/CAD comparison, explicitly not physical registration."""
import itertools,json,sys
from pathlib import Path
import cv2 as cv,numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/support_photo_review_221703'
sys.path.insert(0,str(ROOT.parent/'work/Leap_Test/vision/rubik_center'));import cube_board
SOURCE=ROOT/'quarter_support_photo_20261025_221703_805528';im=cv.imdecode(np.frombuffer((SOURCE/'cube_support_side.jpg').read_bytes(),np.uint8),cv.IMREAD_COLOR)
params=cv.aruco.DetectorParameters();params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_SUBPIX;det=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),params)
cs,ids,_=det.detectMarkers(im);corners={int(i):c.reshape(4,2) for i,c in zip(ids.ravel(),cs)}
h=.0107/2;obj2=np.array([[-h,h],[h,h],[h,-h],[-h,-h]],np.float32);H=cv.getPerspectiveTransform(obj2,corners[1].astype(np.float32))
grid=np.array([(i*.055/3,j*.055/3) for i in (-1,0,1) for j in (-1,0,1) if (i,j)!=(0,0)],np.float64);pred=cv.perspectiveTransform(grid.reshape(-1,1,2),H).reshape(-1,2)
poly=cv.perspectiveTransform(np.array([[-.03,.03],[.03,.03],[.03,-.03],[-.03,-.03]],np.float64).reshape(-1,1,2),H).reshape(-1,2)
hsv=cv.cvtColor(im,cv.COLOR_BGR2HSV);mask=cv.inRange(hsv,np.array([18,35,65]),np.array([48,255,255]));roi=np.zeros_like(mask);cv.fillConvexPoly(roi,np.rint(poly).astype(np.int32),255);mask&=roi;mask=cv.morphologyEx(mask,cv.MORPH_OPEN,np.ones((3,3),np.uint8))
contours,_=cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE);centers=[]
for c in contours:
    if cv.contourArea(c)<120:continue
    q=cv.approxPolyDP(c,.035*cv.arcLength(c,True),True).reshape(-1,2).astype(float)
    if len(q)==4:
        try:t=np.linalg.solve(np.column_stack((q[2]-q[0],-(q[3]-q[1]))),q[1]-q[0])[0];center=q[0]+t*(q[2]-q[0])
        except np.linalg.LinAlgError:center=q.mean(0)
    else:
        m=cv.moments(c);center=np.array([m['m10']/m['m00'],m['m01']/m['m00']])
    centers.append(center)
centers=np.array(centers);objectpoints=[];pixels=[];used=set()
for i,p in enumerate(pred):
    index=next(int(j) for j in np.argsort(np.linalg.norm(centers-p,axis=1)) if int(j) not in used)
    if np.linalg.norm(centers[index]-p)>25:continue
    used.add(index);objectpoints.append([*grid[i],0]);pixels.append(centers[index])
if len(pixels)<6:raise RuntimeError('Too few yellow grid centers')
objectpoints=np.array(objectpoints,np.float64);pixels=np.array(pixels,np.float64)
with np.load(ROOT/'camera1_720.npz') as cal:K=cal['K'].copy();dist=cal['dist'].copy()
rotations=tuple(json.loads((ROOT/'cube_face_rot_result.json').read_bytes())['face_rot']);op,_=cube_board.object_points(cube_mm=55,marker_mm=10.7,layout='centers',face_rot=rotations)
u=(op[1,1]-op[1,0])/.0107;v=(op[1,0]-op[1,3])/.0107;R_cube_marker=np.column_stack([u,v,np.cross(u,v)])
R_world_cube_upright=np.array([[1,0,0],[0,0,1],[0,-1,0]],float)
_,rvs,tvs,_=cv.solvePnPGeneric(objectpoints,pixels,K,dist,flags=cv.SOLVEPNP_IPPE);branches=[]
for rv,tv in zip(rvs,tvs):
    R_marker=cv.Rodrigues(rv)[0];Rcube=R_marker@R_cube_marker.T;tCube=tv.ravel()-Rcube@np.array([0,0,-.0275]);pr,_=cv.projectPoints(objectpoints,rv,tv,K,dist)
    nOrange=-Rcube[:,0];orangeCenter=tCube+nOrange*.0275
    branches.append(dict(error_px=float(np.linalg.norm(pr.reshape(-1,2)-pixels,axis=1).mean()),
        orange_face_visibility=float(-nOrange@orangeCenter),R_cam_cube=Rcube.tolist(),t_cam_cube=tCube.tolist(),
        R_cam_world_if_upright=(Rcube@R_world_cube_upright.T).tolist()))
branches.sort(key=lambda x:x['error_px'])
for i,p in enumerate(pixels):cv.circle(im,tuple(np.rint(p).astype(int)),4,(255,0,255),2);cv.putText(im,str(i),tuple(np.rint(p).astype(int)),cv.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
cv.imencode('.png',im)[1].tofile(str(OUT/'yellow_grid_side.png'))
r=dict(grid_points=len(pixels),branches=branches,green_up_and_orange_toward_wrist_is_an_assumption=True,calibration_identity_reverified=False,
    camera_to_hand_registered=False,hardware_execution_allowed=False,physical_rotation_verified=False)
(OUT/'provisional_camera_orientation.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r))
