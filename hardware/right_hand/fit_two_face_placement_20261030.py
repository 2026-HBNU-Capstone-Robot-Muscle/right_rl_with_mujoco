"""55mm nonplanar Orange/Green grid fit; an unread top marker is never invented."""
import json
from pathlib import Path
import cv2 as cv,numpy as np
from compare_alignment_placement_20261030 import R,F,O,K,D,read_image,decoded,grid_poses,frame_paths,poses
RELATIVE=R/'yaw_roundtrip_trial_20261030_011416_269699/offline_review/side3_to_green_rotation_review.json'
def top_frame():
 relation=np.asarray(json.loads(RELATIVE.read_bytes())['green_frame_in_side3']);exact=np.zeros((3,3))
 for col in range(3):
  i=int(np.argmax(abs(relation[:,col])));exact[i,col]=np.sign(relation[i,col])
 if not np.allclose(exact.T@exact,np.eye(3)) or np.linalg.det(exact)<.99 or not np.allclose(exact[:,2],[-1,0,0]):raise RuntimeError('Existing co-visible face relation does not support expected cube geometry')
 return exact
RG=top_frame()
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
def analyze(path):
 im=read_image(path);found=decoded(im);fit=fused(im,found);row=dict(image=path.relative_to(R).as_posix(),decoded_ids=list(found),fit=fit,body_relative={})
 for b,size in ((40,.0282),(41,.027)):
  if b not in found or not fit['accepted']:continue
  bp=poses(found[b],size)
  if not bp:continue
  b0=bp[0];c0=fit['candidates'][0];br=b0['rotation'];cr=np.asarray(c0['rotation']);row['body_relative'][str(b)]=dict(center_mm=((br.T@(np.asarray(c0['center_camera'])-b0['translation']))*1000).tolist(),face_rotation_body=(br.T@cr).tolist(),green_up_marker_normal_angle_deg=float(np.degrees(np.arccos(np.clip((cr@[-1,0,0])@br[:,2],-1,1)))),body_marker_rmse_px=b0['rmse_px'],body_marker_corners=found[b].tolist(),body_rotation_camera=br.tolist(),body_translation_camera=b0['translation'].tolist())
 return row
def main():
 folders=[R/'single_forward_trial_20261030_052636_068402',R/'single_forward_trial_20261030_053131_966030',F];groups={}
 for folder in folders:
  rows=[analyze(p) for p in frame_paths(folder)];summary={}
  for b in ('40','41'):
   valid=[r['body_relative'][b] for r in rows if b in r['body_relative']]
   if valid:
    cs=[v['center_mm'] for v in valid];summary[b]=dict(frames=len(valid),center_median_mm=np.median(cs,axis=0).tolist(),center_span_mm=np.ptp(cs,axis=0).tolist(),up_to_body_normal_median_deg=float(np.median([v['green_up_marker_normal_angle_deg'] for v in valid])))
  groups[folder.name]=dict(rows=rows,summary=summary);print(folder.name,json.dumps(summary),flush=True)
 comparisons={};base=groups[folders[0].name]['summary']
 for folder in folders[1:]:
  summary=groups[folder.name]['summary'];comparisons[folder.name]={}
  for b in base.keys()&summary.keys():
   delta=np.array(summary[b]['center_median_mm'])-base[b]['center_median_mm'];comparisons[folder.name][b]=dict(center_delta_marker_mm=delta.tolist(),norm_mm=float(np.linalg.norm(delta)))
 report=dict(status='nonplanar_two_colour_face_grid_placement_diagnostic',source=F.name,comparisons=comparisons,groups=groups,green_frame_in_orange_axes=RG.tolist(),missing_marker_not_counted_as_detected=True,metric_scale='55mm cube with18.333mm cubie pitch; body marker40 black28.2mm',absolute_robot_registration_verified=False,camera_calibration_systematic_error_not_identified=True,source_images_unchanged=True,motor_writes=0)
 (O/'two_face_placement_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print('COMPARISON',json.dumps(comparisons),flush=True)
if __name__=='__main__':main()
