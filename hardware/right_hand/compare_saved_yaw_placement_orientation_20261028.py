"""Compare stored real-image orientation with the old model seed; no devices."""
import hashlib,json
from pathlib import Path
import cv2
import numpy as np
from center_marker_cube_pose_20261026 import estimate

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/two_axis_connection/placement_orientation_review'
SOURCES=('calibrated_quarter_trial_20261028_174723_390729/cube_before_grasp.jpg',
         'calibrated_quarter_trial_20261028_052456_066430/cube_before_grasp.jpg')

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist']
    geom=json.loads((ROOT/'right_hand_rotation_20261026/saved_marker_registration/conditional_raw_frame_start_pose.json').read_bytes())
    world=json.loads((ROOT/'right_hand_rotation_20261026/saved_marker_registration/model_coordinate_frames.json').read_bytes())
    Rpm=np.asarray(geom['R_palm_marker40']);Rwp=np.asarray(world['palm_world_rotation'])
    up=-Rwp[:,2];wrist=-Rwp[:,0]
    prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    seed=next(v for v in prior['loaded_physics'] if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10.,1.,.3))
    quat=np.asarray(seed['settled_cube_qpos'][-4:]);length=np.linalg.norm(quat[1:])
    B=cv2.Rodrigues(quat[1:]/length*(2*np.arctan2(length,quat[0])))[0]
    angle=lambda a,b:float(np.degrees(np.arccos(np.clip(a@b,-1,1))))
    model=dict(green_up_error_deg=angle(-B[:,1],up),orange_wrist_error_deg=angle(-B[:,0],wrist),R_cube_world=B.tolist())
    observations=[]
    for name in SOURCES:
        path=ROOT/name
        if not path.exists():continue
        frame=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        for constant in (3.,5.,7.,9.):
            dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            dictionary.maxCorrectionBits=0
            params=cv2.aruco.DetectorParameters();params.errorCorrectionRate=0
            params.adaptiveThreshConstant=constant;params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
            corners,ids,_=cv2.aruco.ArucoDetector(dictionary,params).detectMarkers(gray)
            ids=[] if ids is None else [int(i) for i in ids.ravel()]
            observed={i:np.asarray(c).reshape(4,2) for i,c in zip(ids,corners)}
            row=dict(source=name,source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),threshold_constant=constant,ids=ids)
            if (40 in observed or 41 in observed) and len([i for i in observed if i in range(6)])>=2:
                cube=estimate(observed,K,dist)
                row['cube_fit']=cube
                row['cube_fit_status']=cube['status']
                row['cube_fit_rms_px']=cube.get('reprojection_rms_px')
                # Retain rejected fits as labelled hypotheses for diagnosis;
                # the original estimator's acceptance rule is not relaxed.
                if 'R_cube_camera' in cube:
                    row['cube_pose_fit_accepted']=bool(cube.get('full_orientation_observable'))
                    anchor=40 if 40 in observed else 41
                    s=(.0282 if anchor==40 else .027)/2
                    obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],np.float64)
                    ok,rv,tv=cv2.solvePnP(obj,observed[anchor].astype(np.float64),K,dist)
                    if not ok:raise ValueError('Marker40 fit failed')
                    R40=cv2.Rodrigues(rv)[0]
                    if anchor==41:
                        R40=R40@np.asarray(geom['R_anchor_marker40'])
                    row['hand_reference']=('same_frame_ID40' if anchor==40 else 'ID41_with_historical_rigid_ID40_transform')
                    R=Rwp@Rpm@R40.T@np.asarray(cube['R_cube_camera'])
                    vector=cv2.Rodrigues(R@B.T)[0].ravel();rotation=np.linalg.norm(vector)
                    axis=vector/rotation if rotation>1e-12 else np.zeros(3)
                    row.update(green_up_error_deg=angle(-R[:,1],up),orange_wrist_error_deg=angle(-R[:,0],wrist),
                               twist_from_model_seed_deg=float(np.degrees(2*np.arctan2((axis@up)*np.sin(rotation/2),np.cos(rotation/2)))),
                               R_cube_world_candidate=R.tolist())
            observations.append(row)
            print(json.dumps({k:v for k,v in row.items() if k!='cube_fit'}),flush=True)
    result=dict(status='saved_multi_face_yaw_placement_orientation_comparison',model_seed=model,observations=observations,
                hand_marker_to_palm_alignment_is_provisional=True,marker_sizes_and_mounting_are_approximate=True,
                physical_metric_registration_verified=False,model_or_operator_changed=False,
                real_devices_opened=False,hardware_execution_allowed=False)
    (OUT/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('MODEL',json.dumps(model),flush=True)

if __name__=='__main__':main()
