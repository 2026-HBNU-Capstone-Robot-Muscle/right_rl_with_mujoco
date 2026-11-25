"""Relative marker rotation; marker sizes and printed face offsets cancel.

At least two independently tracked cube markers are compared for a rigid
rotation. This is saved-image analysis, not a live controller. It cannot by
itself certify that no cube layer slipped.
"""
import itertools
import cv2
import numpy as np

def square_candidates(corners,K,dist,normalizing_side=.01):
    h=normalizing_side/2
    obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],float)
    img=np.asarray(corners,float).reshape(4,2)
    result=cv2.solvePnPGeneric(obj,img,K,dist,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    out=[]
    if not result[0]:return out
    for r,t in zip(result[1],result[2]):
        R=cv2.Rodrigues(r)[0];t=t.reshape(3)
        if np.min((obj@R.T+t)[:,2])<=0:continue
        pr=cv2.projectPoints(obj,r,t,K,dist)[0].reshape(4,2)
        error=float(np.sqrt(np.mean(np.sum((pr-img)**2,axis=1))))
        if np.isfinite(error):out.append((R,error))
    return out

def rotation_distance_deg(A,B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A@B.T)-1)/2,-1,1))))

def mean_rotation(matrices):
    U,_,Vt=np.linalg.svd(sum(matrices));D=np.eye(3);D[2,2]=np.linalg.det(U@Vt)
    return U@D@Vt

def angle_about_axis(D,axis):
    r=cv2.Rodrigues(D)[0].reshape(3);angle=np.linalg.norm(r)
    if angle<1e-10:return 0.
    vector=r/angle*np.sin(angle/2);scalar=np.cos(angle/2)
    x=float(vector@axis)
    if np.hypot(scalar,x)<1e-10:return None
    return float((np.degrees(2*np.arctan2(x,scalar))+180)%360-180)

def compare_marker_frames(before,after,reference_id,cube_ids,K,dist,normalizing_side=.01,top_marker_id=None,up_from_cross=None,max_reference_pose_change_deg=None):
    shared=[i for i in cube_ids if i in before and i in after]
    if reference_id not in before or reference_id not in after or len(shared)<2:
        return dict(qualified=False,reason='two_shared_cube_markers_and_reference_required',shared_cube_ids=shared)
    ids=[reference_id]+shared
    hypotheses={}
    for i in ids:
        a=square_candidates(before[i],K,dist,normalizing_side);b=square_candidates(after[i],K,dist,normalizing_side)
        if not a or not b:return dict(qualified=False,reason='no_positive_depth_pose',marker_id=i)
        hypotheses[i]=list(itertools.product(a,b))
    candidates=[]
    for href,hcur in hypotheses[reference_id]:
        if max_reference_pose_change_deg is not None and rotation_distance_deg(href[0],hcur[0])>max_reference_pose_change_deg:continue
        for selected in itertools.product(*(hypotheses[i] for i in shared)):
            normals={i:(pair[0][0][:,2],pair[1][0][:,2]) for i,pair in zip(shared,selected)}
            # Adjacent face normals of a cube must be perpendicular. This uses
            # no face-centre placement or printed sticker quarter rotation.
            if any(abs(a[t]@b[t])>np.sin(np.radians(20)) for a,b in itertools.combinations(normals.values(),2) for t in (0,1)):continue
            if top_marker_id is not None:
                if top_marker_id not in normals:continue
                axis=href[0].T@normals[top_marker_id][0]
            elif up_from_cross is not None:
                if any(i not in normals for i in up_from_cross):continue
                axis=href[0].T@np.cross(normals[up_from_cross[0]][0],normals[up_from_cross[1]][0]);axis/=np.linalg.norm(axis)
            else:axis=np.array([0.,0.,1.])
            deltas=[];errors=[]
            for (cref,ccur) in selected:
                a=href[0].T@cref[0];b=hcur[0].T@ccur[0]
                deltas.append(b@a.T);errors.extend([cref[1],ccur[1]])
            disagreement=max(rotation_distance_deg(a,b) for a,b in itertools.combinations(deltas,2))
            D=mean_rotation(deltas);yaw=angle_about_axis(D,axis)
            if yaw is None:continue
            rms_sum=float(sum(errors)+href[1]+hcur[1]);score=disagreement+.1*rms_sum
            candidates.append(dict(score=score,between_cube_marker_rotation_disagreement_deg=disagreement,reference_axis_twist_deg=yaw,reference_axis_tilt_change_deg=float(np.degrees(np.arccos(np.clip(axis@(D@axis),-1,1)))),axis_in_reference=axis.tolist(),mean_delta_rotation_in_reference=D.tolist(),cube_marker_twists_deg=[angle_about_axis(v,axis) for v in deltas],reprojection_rms_sum_px=rms_sum))
    if not candidates:return dict(qualified=False,reason='no_rotation_hypothesis')
    candidates.sort(key=lambda v:v['score']);best=candidates[0]
    close=[v for v in candidates if v['score']<=best['score']+2]
    spread=max(abs((v['reference_axis_twist_deg']-best['reference_axis_twist_deg']+180)%360-180) for v in close)
    return dict(qualified=bool(best['between_cube_marker_rotation_disagreement_deg']<=5 and spread<=5),shared_cube_ids=shared,reference_id=reference_id,top_marker_id=top_marker_id,up_from_cross=up_from_cross,max_reference_pose_change_deg=max_reference_pose_change_deg,best=best,near_best_hypothesis_yaw_spread_deg=spread,pose_hypotheses=len(candidates),physical_marker_dimension_required=False,face_center_or_sticker_angle_required=False,cube_layer_slip_not_independently_excluded=True,absolute_translation_not_estimated=True)

def detect_markers(image):
    params=cv2.aruco.DetectorParameters();params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;params.cornerRefinementWinSize=3
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),params)
    corners,ids,_=detector.detectMarkers(image)
    return {} if ids is None else {int(i):c.reshape(4,2).tolist() for i,c in zip(ids.ravel(),corners)}
