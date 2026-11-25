"""Display-only body-relative placement guide. Never modifies the raw frame."""
import json,itertools
from pathlib import Path
import cv2 as cv,numpy as np
from placement_reference_geometry_20261030 import K,D,decoded,poses,fused
ROOT=Path(__file__).resolve().parent
MODEL_PATH=ROOT/'right_hand_rotation_20261030/placement_alignment_capture/image_reference.json'
def load_model():
 m=json.loads(MODEL_PATH.read_bytes())
 if not m['display_only'] or m['absolute_robot_registration_verified'] or m['body_marker_id']!=40 or m['cube_side_m']!=.055:raise ValueError('Image reference scope changed')
 return m
def project_target(model,body_rotation,body_translation):
 cube=np.array(list(itertools.product((-1.,1.),repeat=3)))*model['cube_side_m']/2
 points=np.asarray(model['center_body_m'])+cube@np.asarray(model['orange_rotation_body']).T
 camera=points@np.asarray(body_rotation).T+np.asarray(body_translation)
 if np.any(camera[:,2]<=0):raise ValueError('Reference volume behind camera')
 return cv.projectPoints(points,cv.Rodrigues(np.asarray(body_rotation))[0],np.asarray(body_translation).reshape(3,1),K,D)[0].reshape(-1,2)
def estimate(frame,model,*,include_grid=True):
 found=decoded(frame);out=dict(decoded_ids=sorted(found),target_visible=False,metric_available=False,display_only=True,absolute_robot_registration_verified=False)
 if 40 not in found:return dict(out,reason='SHOW PALM ID40 for yellow target; photo remains available')
 options=poses(found[40],model['body_marker_black_side_m'])
 if not options:return dict(out,reason='Palm marker pose not usable; photo remains available')
 best=options[0];r,t=best['rotation'],best['translation'];out.update(target_visible=True,target_points=project_target(model,r,t).tolist(),body_marker_rmse_px=best['rmse_px'])
 if not include_grid:return out
 fit=fused(frame,found)
 if fit['accepted']:
  c=fit['candidates'][0];center=r.T@(np.array(c['center_camera'])-t);delta=(center-np.array(model['center_body_m']))*1000
  out.update(metric_available=True,estimated_center_delta_mm=delta.tolist(),estimated_distance_mm=float(np.linalg.norm(delta)),fit_rmse_px=c['rmse_px'],green_is_colour_grid_not_inferred_marker=True,estimated_height_delta_mm=float(delta[2]))
 else:out['reason']='Yellow target visible; distance needs orange marker + green/orange grid'
 return out
def draw(frame,measurement):
 view=frame.copy()
 if measurement.get('target_visible'):
  points=np.rint(measurement['target_points']).astype(int)
  for a,b in itertools.combinations(range(8),2):
   if bin(a^b).count('1')==1:cv.line(view,tuple(points[a]),tuple(points[b]),(0,255,255),3,cv.LINE_AA)
  at=np.rint(np.mean(points,axis=0)).astype(int);cv.drawMarker(view,tuple(at),(0,255,255),cv.MARKER_CROSS,18,2)
 if measurement.get('metric_available'):
  label='TARGET GAP approx %.0f mm | HEIGHT delta approx %+.0f mm'%(measurement['estimated_distance_mm'],measurement['estimated_height_delta_mm'])
 else:label=measurement.get('reason','Yellow = saved placement projected onto current camera')
 cv.rectangle(view,(0,0),(view.shape[1],44),(15,15,15),-1);cv.putText(view,label[:112],(12,30),cv.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2)
 return view
