"""Compare saved actual cube placements; no camera, motor, or model mutation.

Rejected image fits stay rejected. Hand mounting remains a provisional transform.
"""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from center_marker_cube_pose_20261026 import estimate

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/two_axis_connection/actual_transfer_seating_review'
RUN='pitch_handoff_observe_20261028_221727_181336'
SOURCES=[
    ('old_yaw_start','calibrated_quarter_trial_20261028_174723_390729/cube_before_grasp.jpg'),
    ('placed_after_pitch_began',RUN+'/camera/frame_00860.jpg'),
    ('pitch_hold',RUN+'/cube_pitch_hold.jpg'),
    ('transfer_end',RUN+'/camera/frame_01911.jpg'),
    ('transfer_hold',RUN+'/cube_transfer_hold.jpg'),
    ('transfer_final',RUN+'/cube_after_transfer.jpg')]

def angle(a,b):
    return float(np.degrees(np.arccos(np.clip(np.dot(a,b),-1,1))))

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist']
    geometry=json.loads((ROOT/'right_hand_rotation_20261026/saved_marker_registration/conditional_raw_frame_start_pose.json').read_bytes())
    world=json.loads((ROOT/'right_hand_rotation_20261026/saved_marker_registration/model_coordinate_frames.json').read_bytes())
    Rpm=np.asarray(geometry['R_palm_marker40']);mp=np.asarray(geometry['marker40_center_palm_m'])
    Rwp=np.asarray(world['palm_world_rotation']);wp=np.asarray(world['palm_world_position'])
    up=-Rwp[:,2];wrist=-Rwp[:,0]
    old=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    seed=next(v for v in old['loaded_physics'] if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10.,1.,.3))
    nominal_center=np.asarray(seed['settled_cube_qpos'][:3])
    observations=[]
    for label,name in SOURCES:
        path=ROOT/name
        image=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        best=None
        for constant in (3.,5.,7.,9.):
            dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50);dictionary.maxCorrectionBits=0
            params=cv2.aruco.DetectorParameters();params.errorCorrectionRate=0
            params.adaptiveThreshConstant=constant;params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
            corners,ids,_=cv2.aruco.ArucoDetector(dictionary,params).detectMarkers(gray)
            ids=[] if ids is None else [int(v) for v in ids.ravel()]
            observed={i:np.asarray(c).reshape(4,2) for i,c in zip(ids,corners)}
            for marker_m in (.0105,.0106,.0107):
                fit=estimate(observed,K,dist,marker_m=marker_m)
                row=dict(label=label,source=name,source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    threshold_constant=constant,marker_size_mm=marker_m*1000,ids=ids,
                    cube_fit_status=fit['status'],cube_fit_accepted=fit['full_orientation_observable'],
                    cube_fit_rms_px=fit.get('reprojection_rms_px'),cube_fit=fit)
                if (40 in observed or 41 in observed) and 'R_cube_camera' in fit:
                    anchor=40 if 40 in observed else 41
                    s=(.0282 if anchor==40 else .027)/2
                    obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],np.float64)
                    ok,rv,tv=cv2.solvePnP(obj,observed[anchor].astype(np.float64),K,dist)
                    if ok:
                        Ranchor=cv2.Rodrigues(rv)[0]
                        R40=Ranchor if anchor==40 else Ranchor@np.asarray(geometry['R_anchor_marker40'])
                        t40=tv.ravel() if anchor==40 else tv.ravel()+Ranchor@np.asarray(geometry['t_anchor_marker40_m'])
                        c=mp+Rpm@R40.T@(np.asarray(fit['cube_center_in_camera_m'])-t40)
                        cw=wp+Rwp@c
                        R=Rwp@Rpm@R40.T@np.asarray(fit['R_cube_camera'])
                        projected=cv2.projectPoints(obj,rv,tv,K,dist)[0].reshape(4,2)
                        row.update(reference='same_frame_ID40' if anchor==40 else 'ID41_with_provisional_historical_rigid_transform',palm_marker_fit_rms_px=float(np.sqrt(np.mean(np.sum((projected-observed[anchor])**2,axis=1)))),
                            cube_center_palm_m=c.tolist(),cube_center_world_m=cw.tolist(),R_cube_world_candidate=R.tolist(),
                            green_up_error_deg=angle(-R[:,1],up),red_up_error_deg=angle(R[:,0],up),
                            green_wrist_error_deg=angle(-R[:,1],wrist),orange_wrist_error_deg=angle(-R[:,0],wrist),
                            distance_from_nominal_yaw_seed_mm=float(np.linalg.norm(cw-nominal_center)*1000))
                observations.append(row)
                if 'cube_center_palm_m' in row and (best is None or row['cube_fit_rms_px']<best['cube_fit_rms_px']):
                    best=row
            if constant==5.:
                diagnostic=image.copy()
                if ids:cv2.aruco.drawDetectedMarkers(diagnostic,corners,np.asarray(ids,dtype=np.int32))
                cv2.imwrite(str(OUT/(label+'_detections.jpg')),diagnostic)
        diagnostics=[dict(threshold=r['threshold_constant'],ids=r['ids'],status=r['cube_fit_status'],rms=r['cube_fit_rms_px'])
                     for r in observations if r['label']==label and r['marker_size_mm']==10.7]
        print(json.dumps(dict(label=label,cases=12,diagnostics=diagnostics,best=None if best is None else {k:v for k,v in best.items() if k not in ('cube_fit','source_sha256')})),flush=True)
    grouped=[]
    for label,name in SOURCES:
        valid=[r for r in observations if r['label']==label and r['cube_fit_accepted'] and 'cube_center_palm_m' in r]
        hypotheses=[r for r in observations if r['label']==label and 'cube_center_palm_m' in r]
        row=dict(label=label,source=name,accepted_image_fits=len(valid),metric_registration_verified=False)
        for field,values in [('accepted',valid),('including_rejected_hypotheses',hypotheses)]:
            if values:
                centers=np.asarray([v['cube_center_palm_m'] for v in values])*1000
                row[field]=dict(center_palm_mm_median=np.median(centers,axis=0).tolist(),
                    center_palm_mm_min=centers.min(axis=0).tolist(),center_palm_mm_max=centers.max(axis=0).tolist(),
                    maximum_spread_mm=float(np.max(np.linalg.norm(centers[:,None]-centers[None,:],axis=2))))
        grouped.append(row)
    result=dict(status='saved_actual_handoff_seating_image_comparison',observations=observations,summary=grouped,
        nominal_yaw_seed_center_world_m=nominal_center.tolist(),image_rms_acceptance_unchanged_px=2.,
        marker_size_sensitivity_mm=[10.5,10.6,10.7],palm_marker_size_mm_assumed=28.2,
        palm_marker_to_hand_mount_uncalibrated=True,physical_metric_registration_verified=False,
        rejected_fits_not_promoted=True,model_or_operator_changed=False,devices_opened=False,robot_commands=0)
    (OUT/'review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('SUMMARY',json.dumps(grouped),flush=True)

if __name__=='__main__':main()
