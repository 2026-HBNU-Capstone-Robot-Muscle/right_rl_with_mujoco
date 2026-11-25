"""Sensitivity hypotheses for approximate photo angles, not confidence bounds."""
import json,hashlib
from pathlib import Path
import cv2,numpy as np
from center_marker_cube_pose_20261026 import estimate
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip';source=O/'model_discrepancy/saved_marker_rotation/review.json';raw=json.loads(source.read_bytes());frames=raw['runs'][-1]['frames'];cal=np.load(R/'camera1_720.npz');K0,dist=cal['K'],cal['dist'];rng=np.random.default_rng(20261105);cases=[]
def twist(B,E):
    vector=cv2.Rodrigues(E@B.T)[0].ravel();theta=float(np.linalg.norm(vector));xyz=vector*(np.sin(theta/2)/theta) if theta>1e-12 else np.zeros(3);value=float(np.degrees(2*np.arctan2(xyz@(-B[:,1]),np.cos(theta/2))));return (value+180)%360-180
for i in range(80):
    marker=float(rng.uniform(.0105,.0109));cube=float(rng.uniform(.0548,.0552));K=K0.copy();K[0,0]*=rng.uniform(.98,1.02);K[1,1]*=rng.uniform(.98,1.02);K[:2,2]+=rng.uniform(-2,2,2);sigma=.3 if i<40 else .75;poses={}
    for frame in frames:
        corners={ident:(np.array(value)+rng.normal(0,sigma,(4,2))) for ident,value in frame['corners'].items() if int(ident)<6};p=estimate(corners,K,dist,cube_m=cube,marker_m=marker)
        if p['full_orientation_observable']:poses[frame['file']]=np.array(p['R_cube_camera'])
    row=dict(index=i,corner_noise_sigma_px=sigma,accepted_frames=sorted(poses),whole_turn_deg=None,grasp_to_turn_end_deg=None)
    after=poses.get('cube_after_fixed_hold.jpg')
    if after is not None:
        if 'cube_before_grasp.jpg' in poses:row['whole_turn_deg']=twist(poses['cube_before_grasp.jpg'],after)
        if 'cube_grasp_hold.jpg' in poses:row['grasp_to_turn_end_deg']=twist(poses['cube_grasp_hold.jpg'],after)
    cases.append(row)
summary=[]
for sigma in (.3,.75):
    group=[c for c in cases if c['corner_noise_sigma_px']==sigma];row=dict(corner_noise_sigma_px=sigma,hypotheses=len(group))
    for key in ('whole_turn_deg','grasp_to_turn_end_deg'):
        values=[c[key] for c in group if c[key] is not None];row[key]=dict(accepted=len(values),p05_median_p95=None if not values else np.percentile(values,[5,50,95]).tolist(),minimum=None if not values else min(values),maximum=None if not values else max(values))
    summary.append(row)
report=dict(status='assumed_dimension_intrinsic_and_pixel_noise_sensitivity_only',source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),random_seed=20261105,summary=summary,cases=cases,hypotheses=dict(common_marker_side_mm=[10.5,10.9],cube_side_mm=[54.8,55.2],focal_scale=[.98,1.02],principal_point_shift_px=[-2,2],corner_noise_sigma_px=[.3,.75]),not_a_statistical_confidence_interval=True,face_specific_mount_offsets_not_modelled=True,intrinsic_to_current_device_binding_not_verified=True,camera_motion_between_photos_not_verified=True,physical_exact90_verified=False,devices_opened=False,hardware_execution_allowed=False)
(O/'model_discrepancy/photo_angle_sensitivity.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(summary=summary,physical_exact90_verified=False)),flush=True)
