"""Same-run decoded side3 / Green-grid frame transfer, with explicit limits."""
import json,math
from pathlib import Path
import cv2 as cv,numpy as np
from measure_inverse_side_marker_20261029 import poses,det,K,D
from marker_detector import MarkerDetector
from measure_supported_side_face_20261029 import face_up_vote,AXES
from measure_successful_quarter_20261028 import relative
from measure_roundtrip_orange_grid_20261030 import mean_rotation,distance
R=Path(__file__).resolve().parent;F=R/'yaw_roundtrip_trial_20261030_011416_269699';O=F/'offline_review'
detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),required_cube_count=1)
def data(row):
 im=cv.imdecode(np.frombuffer((F/'camera'/row['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
 cs,ids,_=detector.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY));found={} if ids is None else {int(i):np.asarray(c).reshape(4,2) for i,c in zip(ids.ravel(),cs)}
 return im,found
def angle(a,b):return math.degrees(math.acos(float(np.clip(np.array(a)@np.array(b),-1,1))))
def main():
 primary=json.loads((O/'physical_and_angle_review.json').read_bytes());relations=[];cal=[];votes=[];ups=[]
 for row in primary['rows']:
  if not row.get('accepted') or 3 not in row.get('ids',[]):continue
  im,found=data(row);options=poses(found[3],.0107) if 3 in found else []
  if not options:continue
  up=np.array(row['rotation'])[:,2];valid=[]
  for p in options:
   if p['rmse_px']>options[0]['rmse_px']+.25:continue
   v=face_up_vote(im,p,found[3])
   if v is None:continue
   error=angle(p['rotation']@AXES[v['axis_index']],up)
   if error<12:valid.append((error,p,v))
  if not valid:continue
  error,p,v=min(valid,key=lambda t:t[0]);rel=p['rotation'].T@np.array(row['rotation']);relations.append(rel);ups.append(up);votes.append(v['axis_index'])
  cal.append(dict(image=row['image'],leg_label=row['leg_label'],side_RMSE_px=p['rmse_px'],up_error_deg=error,axis_index=v['axis_index'],relation=rel.tolist()))
 relation,kept=mean_rotation(relations);counts=np.bincount(votes,minlength=4);axis_index=int(np.argmax(counts));purity=float(max(counts)/sum(counts))
 if purity<.9:raise RuntimeError('Side3 up-axis identity ambiguous')
 up_mean=np.mean(ups,0);up_mean/=np.linalg.norm(up_mean);before=[]
 for row in primary['rows']:
  if row['leg_label']!='placed' or 3 not in row.get('ids',[]):continue
  im,found=data(row);options=poses(found[3],.0107) if 3 in found else []
  if not options:continue
  near=[p for p in options if p['rmse_px']<=options[0]['rmse_px']+.25]
  p=min(near,key=lambda p:angle(p['rotation']@AXES[axis_index],up_mean))
  vote=face_up_vote(im,p,found[3])
  if vote is None or vote['axis_index']!=axis_index or angle(p['rotation']@AXES[axis_index],up_mean)>15:continue
  before.append(dict(rotation=(p['rotation']@relation).tolist(),image=row['image'],method='decoded_side3_with_same_run_green_frame_transfer',RMSE_px=p['rmse_px']))
 after=[row for row in primary['rows'] if row['leg_label']=='forward_hold' and row.get('accepted')]
 if len(before)<3 or len(after)<3:raise RuntimeError('Transferred before/after support insufficient')
 values=[relative(a,b) for a in before for b in after];result={key:np.percentile([x[key] for x in values],[5,50,95]).tolist() for key in ('twist_deg','total_deg','up_axis_change_deg')}
 layer_groups={'forward_hold':[],'inverse_hold':[]}
 for row in primary['rows']:
  if row['leg_label'] not in layer_groups or not row.get('accepted'):continue
  im,found=data(row);options=poses(found[0],.0107) if 0 in found else []
  if not options:continue
  near=[p for p in options if p['rmse_px']<=options[0]['rmse_px']+.25];up=np.array(row['rotation'])[:,2]
  p=min(near,key=lambda p:angle(p['rotation']@[-1,0,0],up))
  if angle(p['rotation']@[-1,0,0],up)<12:layer_groups[row['leg_label']].append(p['rotation'].T@np.array(row['rotation']))
 layers=[distance(a,b) for a in layer_groups['forward_hold'] for b in layer_groups['inverse_hold']]
 report=dict(status='same_run_decoded_side3_to_green_grid_transfer',source=F.name,calibration_pairs=len(cal),consistent_calibration_pairs=len(kept),axis3_votes=counts.tolist(),axis3_purity=purity,
     green_frame_in_side3=relation.tolist(),calibration_rotation_spread_p50_p95_deg=np.percentile([distance(relation,x) for x in kept],[50,95]).tolist(),
     placement_frames=len(before),forward_frames=len(after),placement_to_forward_camera_relative_p05_median_p95_deg=result,
     green_white_relative_pose_change_after_up_disambiguation_p05_median_p95_deg=np.percentile(layers,[5,50,95]).tolist() if layers else None,
     unreadable_placed_ID5_not_fabricated=True,body_motion_not_compensated=True,metric_robot_registration_verified=False,frame_spread_not_absolute_accuracy=True,
     source_images_unchanged=True,motor_writes=0,calibration=cal,before=before)
 (O/'side3_to_green_rotation_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k not in ('calibration','before')}),flush=True)
if __name__=='__main__':main()
