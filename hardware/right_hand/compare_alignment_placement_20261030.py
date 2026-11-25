"""Compare actual placements in decoded palm/mount marker frames. No devices.

Cube face translation uses 55mm cubie pitch, not a tiny-marker distance scale.
All plausible planar branches and their ambiguity remain in the diagnostic.
"""
import json,hashlib
from pathlib import Path
import cv2 as cv,numpy as np
from marker_detector import MarkerDetector
from measure_inverse_side_marker_20261029 import K,D,poses
R=Path(__file__).resolve().parent;LATEST=json.loads((R/'leap_placement_alignment_capture_result.json').read_bytes());F=Path(LATEST['camera_recording']['folder']).parent;O=F/'offline_review';O.mkdir(exist_ok=True)
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[3,5,41],hand_id=40,required_cube_count=1)
def read_image(p):return cv.imdecode(np.frombuffer(p.read_bytes(),np.uint8),cv.IMREAD_COLOR)
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
def analyze(path):
 im=read_image(path);found=decoded(im);row=dict(image=path.relative_to(R).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),decoded_ids=list(found),marker_corners={str(k):v.tolist() for k,v in found.items()},faces={},body_relative={})
 for ident in (3,5):row['faces'][str(ident)]=grid_poses(im,found,ident)
 for body,size in ((40,.0282),(41,.027)):
  if body not in found:continue
  options=poses(found[body],size)
  if not options:continue
  row['body_relative'][str(body)]={};body_options=[p for p in options if p['rmse_px']<=options[0]['rmse_px']+.25]
  for ident in (3,5):
   g=row['faces'][str(ident)]
   if not g['accepted']:continue
   fo=[p for p in g['candidates'] if p['rmse_px']<=g['candidates'][0]['rmse_px']+.25];combinations=[]
   for bp in body_options:
    for fp in fo:
     rb=bp['rotation'];rf=np.asarray(fp['rotation']);c=rb.T@(np.asarray(fp['cube_center_camera'])-bp['translation']);up=rf@np.array([-1.,0,0]) if ident==3 else rf[:,2];tilt=float(np.degrees(np.arccos(np.clip(up@rb[:,2],-1,1))))
     combinations.append(dict(center_body_mm=(c*1000).tolist(),face_rotation_body=(rb.T@rf).tolist(),body_rmse_px=bp['rmse_px'],face_rmse_px=fp['rmse_px'],green_up_to_body_marker_normal_deg=tilt))
   row['body_relative'][str(body)][str(ident)]=combinations
 return row
def frame_paths(folder):
 r=json.loads((folder/'result.json').read_bytes());named=folder/r['placement_photo']['image'];stamp=r['placement_photo']['camera_read_monotonic'];frames=json.loads((folder/'camera/frames.json').read_bytes())['frames'];near=[f for f in frames if stamp<=f['camera_read_monotonic']<=stamp+.85];chosen=near[::max(1,len(near)//7)][:8];return [named]+[folder/'camera'/f['image'] for f in chosen]
def main():
 folders=[R/'single_forward_trial_20261030_052636_068402',R/'single_forward_trial_20261030_053131_966030',F];groups={}
 for folder in folders:
  rows=[analyze(p) for p in frame_paths(folder)];stats={}
  for b in ('40','41'):
   stats[b]={}
   for face in ('3','5'):
    values=[p['center_body_mm'] for row in rows for p in row['body_relative'].get(b,{}).get(face,[])];best=[row['body_relative'][b][face][0]['center_body_mm'] for row in rows if row['body_relative'].get(b,{}).get(face)]
    if values:stats[b][face]=dict(frames=len(best),plausible_planar_pose_centers=len(values),center_median_mm=np.median(best,axis=0).tolist(),center_percentiles_all_branches_mm=np.percentile(values,[5,50,95],axis=0).tolist(),maximum_all_branch_span_mm=np.ptp(values,axis=0).tolist())
  groups[folder.name]=dict(rows=rows,summary=stats);print(folder.name,json.dumps(stats),flush=True)
 ref=groups[folders[0].name]['summary'];comparison={}
 for folder in folders[1:]:
  stats=groups[folder.name]['summary'];comparison[folder.name]={}
  for b in ('40','41'):
   comparison[folder.name][b]={}
   for face in ('3','5'):
    if face in ref[b] and face in stats[b]:
     delta=np.asarray(stats[b][face]['center_median_mm'])-ref[b][face]['center_median_mm'];comparison[folder.name][b][face]=dict(delta_body_marker_mm=delta.tolist(),norm_mm=float(np.linalg.norm(delta)))
 result=dict(status='saved_grid_based_cube_center_relative_to_decoded_body_markers',source=F.name,cube_side_mm=55,body_marker_black_side_mm={'40':28.2,'41':27},little_cube_marker_size_not_used_as_metric_distance_scale=True,comparison=comparison,groups=groups,frame_spread_not_absolute_accuracy=True,body_marker_frame_not_robot_coordinates=True,physical_grasp_or_rotation_not_tested_in_capture=True,source_images_unchanged=True,motor_writes=0)
 (O/'placement_metric_comparison.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print('COMPARISON',json.dumps(comparison),flush=True)
if __name__=='__main__':main()
