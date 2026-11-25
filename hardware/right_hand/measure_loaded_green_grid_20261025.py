"""Large green-face grid pose comparison; retain both planar solutions."""
import json
from pathlib import Path
import numpy as np,cv2 as cv
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'loaded_quarter_trial_20261025_171607_019752'
OUT=ROOT/'right_hand_rotation_20261024/actual_loaded_review_171607'
params=cv.aruco.DetectorParameters();params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_SUBPIX
det=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),params)
with np.load(ROOT/'camera1_720.npz') as cal:K=np.asarray(cal['K'],np.float64);dist=np.asarray(cal['dist'],np.float64)
h=.0107/2;marker=np.array([[-h,h],[h,h],[h,-h],[-h,-h]],np.float32)
grid=np.array([(i*.055/3,j*.055/3) for i in (-1,0,1) for j in (-1,0,1) if (i,j)!=(0,0)],np.float64)
def corners_and_grid(path):
    im=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR);gray=cv.cvtColor(im,cv.COLOR_BGR2GRAY)
    cs,ids,_=det.detectMarkers(gray);found={int(i):c.reshape(4,2) for i,c in zip(ids.ravel(),cs)}
    H=cv.getPerspectiveTransform(marker,found[5].astype(np.float32));pred=cv.perspectiveTransform(grid.reshape(-1,1,2),H).reshape(-1,2)
    poly=cv.perspectiveTransform((np.array([[-.03,.03],[.03,.03],[.03,-.03],[-.03,-.03]],np.float64)).reshape(-1,1,2),H).reshape(-1,2)
    hsv=cv.cvtColor(im,cv.COLOR_BGR2HSV);mask=cv.inRange(hsv,np.array([30,28,70]),np.array([105,255,255]));roi=np.zeros_like(mask);cv.fillConvexPoly(roi,np.rint(poly).astype(np.int32),255);mask&=roi
    mask=cv.morphologyEx(mask,cv.MORPH_OPEN,np.ones((3,3),np.uint8));contours,_=cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    points=[]
    for contour in contours:
        if cv.contourArea(contour)<80:continue
        q=cv.approxPolyDP(contour,.035*cv.arcLength(contour,True),True).reshape(-1,2).astype(float)
        if len(q)==4:
            A=np.column_stack((q[2]-q[0],-(q[3]-q[1])))
            try:t=np.linalg.solve(A,q[1]-q[0])[0];center=q[0]+t*(q[2]-q[0])
            except np.linalg.LinAlgError:center=q.mean(0)
        else:
            mom=cv.moments(contour);center=np.array([mom['m10']/mom['m00'],mom['m01']/mom['m00']])
        points.append(center)
    points=np.array(points);obj=[];image=[];used=set()
    for i,expected in enumerate(pred):
        order=np.argsort(np.linalg.norm(points-expected,axis=1));index=next(int(j) for j in order if int(j) not in used)
        error=np.linalg.norm(points[index]-expected)
        if error>25:continue
        used.add(index);obj.append([*grid[i],0]);image.append(points[index])
    if len(obj)<6:raise RuntimeError(f'Only {len(obj)} green grid correspondences')
    return im,np.asarray(obj,np.float64),np.asarray(image,np.float64),dict(predicted=pred.tolist(),matched_centers=image,detected_components=points.tolist())
first,obj0,img0,debug0=corners_and_grid(SOURCE/'cube_before_turn.jpg');last,obj1,img1,debug1=corners_and_grid(SOURCE/'cube_after_turn.jpg')
def solve(obj,img,intrinsic):
    _,rvs,tvs,_=cv.solvePnPGeneric(obj,img,intrinsic,dist,flags=cv.SOLVEPNP_IPPE);rows=[]
    for rv,tv in zip(rvs,tvs):
        projected,_=cv.projectPoints(obj,rv,tv,intrinsic,dist);err=float(np.linalg.norm(projected.reshape(-1,2)-img,axis=1).mean())
        rows.append((err,cv.Rodrigues(rv)[0],tv))
    return sorted(rows,key=lambda r:r[0])
results=[]
for scale in (.8,1.,1.2):
    intrinsic=K.copy();intrinsic[0,0]*=scale;intrinsic[1,1]*=scale
    a=solve(obj0,img0,intrinsic);b=solve(obj1,img1,intrinsic)
    for i,(err,R,t) in enumerate(b):
        angle=float(np.linalg.norm(cv.Rodrigues(R@a[0][1].T)[0])*180/np.pi)
        results.append(dict(focal_scale=scale,after_branch=i,before_error_px=a[0][0],before_alternative_error_px=a[1][0],after_error_px=err,orientation_change_deg=angle))
mask=np.zeros(first.shape[:2],np.uint8);mask[610:720,190:365]=255
g0=cv.cvtColor(first,cv.COLOR_BGR2GRAY);g1=cv.cvtColor(last,cv.COLOR_BGR2GRAY);p0=cv.goodFeaturesToTrack(g0,maxCorners=60,qualityLevel=.02,minDistance=4,mask=mask)
p1,status,_=cv.calcOpticalFlowPyrLK(g0,g1,p0,None);back,good,_=cv.calcOpticalFlowPyrLK(g1,g0,p1,None)
valid=status.ravel().astype(bool)&good.ravel().astype(bool)&(np.linalg.norm(back.reshape(-1,2)-p0.reshape(-1,2),axis=1)<.5)
shifts=p1.reshape(-1,2)[valid]-p0.reshape(-1,2)[valid]
anchor=dict(valid_tracks=int(valid.sum()),median_image_shift_px=np.median(shifts,axis=0).tolist(),not_a_3d_camera_motion_compensation=True)
report=dict(results=results,matched_before=len(obj0),matched_after=len(obj1),anchor_image_motion=anchor,
    before_grid_objects=obj0.tolist(),before_grid_pixels=img0.tolist(),after_grid_objects=obj1.tolist(),after_grid_pixels=img1.tolist(),
    calibration_identity_not_reverified=True,grid_pitch_assumed_mm=55/3,diagnostic_only=True,physical_success_certified=False)
(OUT/'green_grid_orientation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
for name,im,points in (('green_grid_before.png',first,img0),('green_grid_after.png',last,img1)):
    for i,p in enumerate(points):cv.circle(im,tuple(np.rint(p).astype(int)),4,(255,0,255),2);cv.putText(im,str(i),tuple(np.rint(p).astype(int)),cv.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
    cv.imencode('.png',im)[1].tofile(str(OUT/name))
print(json.dumps({k:v for k,v in report.items() if not k.endswith(('_objects','_pixels'))},ensure_ascii=False))
