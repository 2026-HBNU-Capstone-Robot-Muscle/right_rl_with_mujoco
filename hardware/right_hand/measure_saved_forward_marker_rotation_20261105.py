"""Try spin-invariant two-face fits in existing photos; no new capture.

An approximate photographic estimate cannot certify exact90 degrees, metric
registration or a new grip. Rejected marker fits remain rejected.
"""
import json,hashlib
from pathlib import Path
import cv2,numpy as np
from marker_detector import MarkerDetector
from center_marker_cube_pose_20261026 import estimate
from analyze_cube_visibility_pose_20261026 import pose

R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip/model_discrepancy/saved_marker_rotation';O.mkdir(parents=True,exist_ok=True)
calpath=R/'camera1_720.npz';cal=np.load(calpath);K,dist=cal['K'],cal['dist']
detector=MarkerDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),range(6),hand_id=40,required_cube_count=2)
reports=[]
for run in ('forward_hold_observe_20261105_030930_516359','forward_hold_observe_20261105_040030_646453'):
    frames=[]
    for filename in ('cube_before_grasp.jpg','cube_grasp_hold.jpg','cube_after_fixed_hold.jpg'):
        source=R/run/filename
        if not source.exists():continue
        frame=cv2.imdecode(np.frombuffer(source.read_bytes(),dtype=np.uint8),cv2.IMREAD_COLOR)
        if frame is None:raise ValueError('Cannot decode saved image '+filename)
        corners,ids,_=detector.detectMarkers(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY));observed={} if ids is None else {str(int(ident)):np.asarray(c).reshape(4,2).tolist() for c,ident in zip(corners,ids.ravel())}
        p=estimate(observed,K,dist,marker_m=.0107)
        anchors=pose(observed['40'],.027,K,dist) if '40' in observed else []
        anchors=[v for v in anchors if v['reprojection_rms_px']<=2]
        row=dict(file=filename,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),detected_ids=[] if ids is None else ids.ravel().tolist(),corners=observed,pose=p,palm_pose_candidates=anchors)
        if p['full_orientation_observable'] and anchors:
            row['R_cube_in_palm_best_planar_branch']=(np.asarray(anchors[0]['R']).T@np.asarray(p['R_cube_camera'])).tolist()
        frames.append(row)
    pairs=[]
    for before_name in ('cube_before_grasp.jpg','cube_grasp_hold.jpg'):
        before=next((f for f in frames if f['file']==before_name),None);after=next((f for f in frames if f['file']=='cube_after_fixed_hold.jpg'),None)
        for mode,key in (('camera','R_cube_camera'),('palm','R_cube_in_palm_best_planar_branch')):
            b=None if before is None else before['pose'].get(key) if mode=='camera' else before.get(key)
            e=None if after is None else after['pose'].get(key) if mode=='camera' else after.get(key)
            accepted=before is not None and after is not None and before['pose']['full_orientation_observable'] and after['pose']['full_orientation_observable'] and b is not None and e is not None
            row=dict(before=before_name,after='cube_after_fixed_hold.jpg',frame=mode,accepted=accepted)
            if accepted:
                B,E=np.asarray(b),np.asarray(e);rv=cv2.Rodrigues(E@B.T)[0].ravel();theta=float(np.linalg.norm(rv));xyz=rv*(np.sin(theta/2)/theta) if theta>1e-12 else np.zeros(3);up=-B[:,1];yaw=np.degrees(2*np.arctan2(float(xyz@up),float(np.cos(theta/2))));yaw=(yaw+180)%360-180;tilt=np.degrees(np.arccos(np.clip(B[:,1]@E[:,1],-1,1)));row.update(approximate_up_axis_twist_deg=float(yaw),up_axis_change_deg=float(tilt),total_rotation_deg=float(np.degrees(theta)))
            pairs.append(row)
    reports.append(dict(source_run=run,frames=frames,comparisons=pairs))
report=dict(status='saved_photo_two_face_marker_fit_attempt',intrinsic_source=str(calpath.relative_to(R)),intrinsic_sha256=hashlib.sha256(calpath.read_bytes()).hexdigest(),runs=reports,marker_m=.0107,cube_m=.055,approximate_dimensions=True,face_mount_offsets_not_calibrated=True,planar_palm_branch_not_independently_verified=True,physical_exact90_verified=False,hardware_execution_allowed=False,devices_opened=False)
(O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(runs=[dict(run=r['source_run'],fits=[dict(file=f['file'],ids=f['detected_ids'],status=f['pose']['status'],observable=f['pose']['full_orientation_observable'],rms=f['pose'].get('reprojection_rms_px')) for f in r['frames']],comparisons=r['comparisons']) for r in reports]),ensure_ascii=False),flush=True)
