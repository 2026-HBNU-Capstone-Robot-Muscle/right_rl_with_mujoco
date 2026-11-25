"""Image geometry only. No motor, camera or window construction at import."""
import json
from pathlib import Path
import cv2 as cv,numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent
cal=np.load(ROOT/'camera1_720.npz');K,D=cal['K'],cal['dist']
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[3,5,41],hand_id=40,required_cube_count=1)
RG=np.array([[0.,0.,-1.],[0.,1.,0.],[1.,0.,0.]])
def poses(points,size):
    h=size/2;obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],float)
    ok,rs,ts,_=cv.solvePnPGeneric(obj,np.asarray(points,float),K,D,flags=cv.SOLVEPNP_IPPE_SQUARE)
    out=[]
    for rv,tv in zip(rs,ts):
        if tv[2,0]<=0:continue
        rv,tv=cv.solvePnPRefineLM(obj,np.asarray(points,float),K,D,rv,tv)
        projected=cv.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)
        error=float(np.sqrt(np.mean(np.sum((projected-points)**2,axis=1))))
        if error<=1.5:out.append(dict(rotation=cv.Rodrigues(rv)[0],translation=tv.ravel(),rmse_px=error))
    return sorted(out,key=lambda p:p['rmse_px'])
def decoded(im):
 cs,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY));return {} if ids is None else {int(i):np.asarray(c).reshape(4,2) for c,i in zip(cs,ids.ravel())}

def grid_poses(im,found,ident):
 if ident not in found:return dict(accepted=False,reason='marker_not_decoded',candidates=[])
 q=found[ident].astype(np.float32);unit=np.float32([[-.5,.5],[.5,.5],[.5,-.5],[-.5,-.5]]);H=cv.getPerspectiveTransform(q,unit);extent=.055/.0107*.5*1.15;boundary=cv.perspectiveTransform(np.float32([[-extent,extent],[extent,extent],[extent,-extent],[-extent,-extent]])[None],np.linalg.inv(H))[0];roi=np.zeros(im.shape[:2],np.uint8);cv.fillConvexPoly(roi,np.rint(boundary).astype(np.int32),255);hsv=cv.cvtColor(im,cv.COLOR_BGR2HSV)
 low,high=([0,70,60],[25,255,255]) if ident==3 else ([35,55,35],[95,255,255]);mask=cv.bitwise_and(cv.inRange(hsv,np.array(low),np.array(high)),roi);n,labels,stats,centers=cv.connectedComponentsWithStats(mask);assigned={};pitch=.055/3;ratio=pitch/.0107
 for i in range(1,n):
  x,y,w,h,area=stats[i]
  if not 100<=area<=10000 or not 10<=w<=180 or not 6<=h<=150:continue
  p=cv.perspectiveTransform(np.float32(centers[i])[None,None],H)[0,0]/ratio;key=tuple(np.rint(p).astype(int));err=float(np.linalg.norm(p-key))
  if key==(0,0) or max(abs(v) for v in key)>1 or err>.4:continue
  if key not in assigned or err<assigned[key][0]:assigned[key]=(err,centers[i])
 row=dict(accepted=False,sticker_count=len(assigned),candidates=[])
 if len(assigned)<6:return row
 keys=sorted(assigned);obj=np.float64([[x*pitch,y*pitch,0.] for x,y in keys]);points=np.float64([assigned[k][1] for k in keys]);und=cv.undistortPoints(points[:,None],K,D,P=K).reshape(-1,2);_,inl=cv.findHomography(obj[:,:2],und,cv.RANSAC,1.5,maxIters=5000,confidence=.999)
 if inl is None or inl.sum()<6:return row
 valid=inl.ravel().astype(bool);obj=obj[valid];points=points[valid];ok,rs,ts,_=cv.solvePnPGeneric(obj,points,K,D,flags=cv.SOLVEPNP_IPPE)
 for rv,tv in zip(rs,ts):
  if tv[2,0]<=0:continue
  rv,tv=cv.solvePnPRefineLM(obj,points,K,D,rv,tv);proj=cv.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2);err=float(np.sqrt(np.mean(np.sum((proj-points)**2,axis=1))));rot=cv.Rodrigues(rv)[0];center=tv.ravel()-rot[:,2]*.0275
  if err<2:row['candidates'].append(dict(rotation=rot.tolist(),translation=tv.ravel().tolist(),cube_center_camera=center.tolist(),rmse_px=err))
 row['candidates'].sort(key=lambda p:p['rmse_px']);row.update(accepted=bool(row['candidates']),grid_inliers=int(inl.sum()),grid_obj_points=obj.tolist(),grid_image_points=points.tolist(),metric_scale_source='55mm_cube_pitch_over_three_cubies')
 return row
def fused(im,found):
 first=grid_poses(im,found,3);out=dict(accepted=False,orange_grid=first,top_marker_decoded=5 in found,candidates=[],attempts=[])
 if not first['accepted']:return out
 hsv=cv.cvtColor(im,cv.COLOR_BGR2HSV);pitch=.055/3;top_center=np.array([-.0275,0.,-.0275]);all_top=np.array([top_center+RG@np.array([x*pitch,y*pitch,0.]) for x in (-1,0,1) for y in (-1,0,1) if (x,y)!=(0,0)])
 top_corners=np.array([top_center+RG@np.array([x*.0275,y*.0275,0.]) for x,y in ((-1,1),(1,1),(1,-1),(-1,-1))]);unit=np.float32([[-1,1],[1,1],[1,-1],[-1,-1]])
 for initial in first['candidates']:
  rot=np.asarray(initial['rotation']);rv=cv.Rodrigues(rot)[0];tv=np.asarray(initial['translation']).reshape(3,1);corners=cv.projectPoints(top_corners,rv,tv,K,D)[0].reshape(-1,2);center=cv.projectPoints(top_center[None],rv,tv,K,D)[0].reshape(2);expanded=center+(corners-center)*1.1;roi=np.zeros(im.shape[:2],np.uint8);cv.fillConvexPoly(roi,np.rint(expanded).astype(np.int32),255)
  # Green light can tint white plastic separators. Split saturation inside
  # the projected green face instead of connecting all eight stickers.
  hue=((hsv[:,:,0]>=35)&(hsv[:,:,0]<=100)&(hsv[:,:,2]>=35)&(roi>0));sat=hsv[:,:,1][hue];threshold=cv.threshold(sat.reshape(-1,1),0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)[0] if len(sat) else 45;smin=max(45,int(threshold));mask=cv.bitwise_and(cv.inRange(hsv,np.array([35,smin,35]),np.array([100,255,255])),roi);n,labels,stats,centers=cv.connectedComponentsWithStats(mask);H=cv.getPerspectiveTransform(corners.astype(np.float32),unit);assigned={}
  for i in range(1,n):
   x,y,w,h,area=stats[i]
   if not 90<=area<=10000 or w<9 or h<5:continue
   p=cv.perspectiveTransform(np.float32(centers[i])[None,None],H)[0,0]*1.5;key=tuple(np.rint(p).astype(int));error=float(np.linalg.norm(p-key))
   if key==(0,0) or max(abs(v) for v in key)>1 or error>.38:continue
   if key not in assigned or error<assigned[key][0]:assigned[key]=(error,centers[i])
  attempt=dict(green_assigned=len(assigned),green_saturation_threshold=smin,green_points=[dict(key=[int(x) for x in k],point=v[1].tolist(),error=v[0]) for k,v in assigned.items()],projected_corners=corners.tolist());out['attempts'].append(attempt)
  if len(assigned)<5:continue
  keys=sorted(assigned);objo=np.array(first['grid_obj_points']);imgo=np.array(first['grid_image_points']);objg=np.array([top_center+RG@np.array([x*pitch,y*pitch,0.]) for x,y in keys]);imgg=np.array([assigned[k][1] for k in keys]);obj=np.vstack([objo,objg]);img=np.vstack([imgo,imgg]);ok,rnew,tnew,inliers=cv.solvePnPRansac(obj,img,K,D,rvec=rv.copy(),tvec=tv.copy(),useExtrinsicGuess=True,iterationsCount=200,reprojectionError=2.,confidence=.999,flags=cv.SOLVEPNP_ITERATIVE)
  attempt.update(pnp_ok=bool(ok),pnp_inliers=None if inliers is None else inliers.ravel().tolist())
  if not ok or inliers is None:continue
  selected=inliers.ravel();ng=int((selected>=len(objo)).sum());no=len(selected)-ng
  # Nonplanar fit: at least four side points, five top points and eleven
  # combined inliers; the independent side initialization still required6.
  if no<4 or ng<5 or len(selected)<11:continue
  rnew,tnew=cv.solvePnPRefineLM(obj[selected],img[selected],K,D,rnew,tnew);project=cv.projectPoints(obj[selected],rnew,tnew,K,D)[0].reshape(-1,2);error=float(np.sqrt(np.mean(np.sum((project-img[selected])**2,axis=1))));rn=cv.Rodrigues(rnew)[0]
  attempt.update(rmse_px=error,orange_inliers=no,green_inliers=ng)
  if error<=1.5 and tnew[2,0]>0:out['candidates'].append(dict(rotation=rn.tolist(),translation=tnew.ravel().tolist(),center_camera=(tnew.ravel()-rn[:,2]*.0275).tolist(),orange_inliers=no,green_colour_inliers=ng,rmse_px=error,green_is_colour_grid_not_inferred_marker=True))
 out['candidates'].sort(key=lambda r:r['rmse_px']);out['accepted']=bool(out['candidates']);return out
