"""Review saved free-standing placements; do not demand the old target height."""
import json,hashlib
from pathlib import Path
import numpy as np,cv2 as cv
from placement_reference_overlay_20261030 import load_model,estimate
from placement_reference_geometry_20261030 import decoded,fused,poses
R=Path(__file__).resolve().parent;F=R/'placement_alignment_capture_20261030_101046_586473';O=R/'right_hand_rotation_20261030/stable_support_research';O.mkdir(exist_ok=True);meta=json.loads((F/'camera/frames.json').read_bytes())['frames'];frames=[v for v in meta if v['phase']=='place_cube'];rows=[];model=load_model()
# The user explicitly confirms the cube remains in the root recess hands-free.
# Review multiple late PLACE frames, before OFF moves the thumb.
late=[v for v in frames if v['camera_read_monotonic']>=frames[-1]['camera_read_monotonic']-3.]
for frame in late[::max(1,len(late)//8)]:
 path=F/'camera'/frame['image'];im=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR);found=decoded(im);fit=fused(im,found);row=dict(image=path.relative_to(R).as_posix(),stamp=frame['camera_read_monotonic'],decoded_ids=sorted(found),accepted=False)
 if 40 in found and fit['accepted']:
  body=poses(found[40],.0282)
  if body:
   b=body[0];c=fit['candidates'][0];center=b['rotation'].T@(np.array(c['center_camera'])-b['translation']);rot=b['rotation'].T@np.array(c['rotation']);row.update(accepted=True,center_body_mm=(center*1000).tolist(),orange_rotation_body=rot.tolist(),face_fit_rmse_px=c['rmse_px'],body_fit_rmse_px=b['rmse_px'],delta_from_old_reference_mm=((center-np.array(model['center_body_m']))*1000).tolist())
 rows.append(row)
valid=[v for v in rows if v['accepted']];assert len(valid)>=5
centers=np.array([v['center_body_mm'] for v in valid]);delta=np.array([v['delta_from_old_reference_mm'] for v in valid]);mean=np.mean([v['orange_rotation_body'] for v in valid],axis=0);u,s,v=np.linalg.svd(mean);rot=u@v;assert np.linalg.det(rot)>.999
report=dict(status='user_confirmed_natural_root_recess_placement_saved',source_folder=F.name,user_confirmed_cube_remains_without_hand_support=True,body_marker_id=40,cube_side_mm=55,frames=len(rows),accepted_frames=len(valid),center_median_body_mm=np.median(centers,axis=0).tolist(),center_span_body_mm=np.ptp(centers,axis=0).tolist(),delta_from_old_reference_median_mm=np.median(delta,axis=0).tolist(),orange_rotation_body=rot.tolist(),rows=rows,absolute_robot_registration_verified=False,calibration_accuracy_not_certified=True,no_new_capture_required_for_this_comparison=True,hardware_execution_allowed=False,motor_writes=0)
(O/'natural_placement_measurement.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
