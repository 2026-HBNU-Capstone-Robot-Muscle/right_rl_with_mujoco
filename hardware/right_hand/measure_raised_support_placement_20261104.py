"""Saved-image placement estimates under the existing provisional marker model."""
import json
from pathlib import Path
import numpy as np,cv2 as cv
from placement_reference_geometry_20261030 import decoded,poses,fused

R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261104/rotation_hold_session';O.mkdir(exist_ok=True)
old=json.loads((R/'right_hand_rotation_20261030/stable_support_research/natural_placement_measurement.json').read_bytes())
rows=[]
for folder in sorted(R.glob('raised_support_capture_20261104_*')):
    result=json.loads((folder/'result.json').read_bytes())
    a=result['placement_photo']['camera_read_monotonic'];b=result['support_observation_photo']['camera_read_monotonic']
    meta=json.loads((folder/'camera/frames.json').read_bytes())['frames']
    frames=[v for v in meta if a-.25<=v['camera_read_monotonic']<=b+.25]
    for v in frames[::max(1,len(frames)//10)]:
        p=folder/'camera'/v['image'];img=cv.imdecode(np.frombuffer(p.read_bytes(),np.uint8),cv.IMREAD_COLOR)
        found=decoded(img);row=dict(file=p.relative_to(R).as_posix(),decoded_IDs=sorted(found),accepted=False)
        if 40 in found:
            fit=fused(img,found);body=poses(found[40],.0282)
            row.update(orange_grid_accepted=fit['orange_grid']['accepted'],orange_sticker_count=fit['orange_grid'].get('sticker_count'),green_assigned=[r.get('green_assigned') for r in fit.get('attempts',[])],body_pose_candidates=len(body))
            if fit['accepted'] and body:
                f=fit['candidates'][0];h=body[0];center=h['rotation'].T@(np.array(f['center_camera'])-h['translation'])
                row.update(accepted=True,center_body_mm=(center*1000).tolist(),orange_rotation_body=(h['rotation'].T@np.array(f['rotation'])).tolist(),cube_fit_rmse_px=f['rmse_px'],body_fit_rmse_px=h['rmse_px'],orange_inliers=f['orange_inliers'],green_colour_inliers=f['green_colour_inliers'])
        rows.append(row)
valid=[r for r in rows if r['accepted']]
report=dict(status='provisional_saved_image_placement_comparison',body_marker_side_mm=28.2,body_marker_size_provisional=True,cube_side_mm=55,absolute_robot_registration_verified=False,hardware_execution_allowed=False,devices_opened=False,rows=rows)
if valid:
    points=np.array([r['center_body_mm'] for r in valid]);M=np.mean([r['orange_rotation_body'] for r in valid],axis=0);u,s,v=np.linalg.svd(M)
    report.update(valid_frames=len(valid),center_median_body_mm=np.median(points,axis=0).tolist(),center_span_body_mm=np.ptp(points,axis=0).tolist(),delta_from_previous_natural_mm=(np.median(points,axis=0)-np.array(old['center_median_body_mm'])).tolist(),orange_rotation_body=(u@v).tolist())
(O/'actual_placement_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
