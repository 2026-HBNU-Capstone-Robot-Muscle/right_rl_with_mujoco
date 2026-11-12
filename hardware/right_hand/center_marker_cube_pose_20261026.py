"""Image-only cube pose that allows each centre marker to spin independently.

Face IDs 0..5 identify fixed face normals. A layer turn may rotate its centre
marker's corners in place; the face-to-marker quarter rotation is therefore
estimated per observation. One visible face never certifies whole-cube yaw.
This module has no camera, servo, or policy interface.
"""
import itertools
import cv2
import numpy as np
FACE_BASES=(
    ((0,0,1),(1,0,0),(0,1,0)),((0,0,-1),(-1,0,0),(0,1,0)),
    ((1,0,0),(0,1,0),(0,0,1)),((-1,0,0),(0,-1,0),(0,0,1)),
    ((0,1,0),(0,0,1),(1,0,0)),((0,-1,0),(0,0,-1),(1,0,0)))
FACE_NAMES=('white','yellow','red','orange','blue','green')

def object_corners(face,turn,cube_m=.055,marker_m=.0107):
    n,u,v=(np.asarray(x,dtype=np.float64) for x in FACE_BASES[face])
    for _ in range(turn%4):u,v=v,-u
    c=n*cube_m/2;s=marker_m/2
    return np.asarray([c-s*u+s*v,c+s*u+s*v,c+s*u-s*v,c-s*u-s*v])

def estimate(corners_by_id,K,dist,*,cube_m=.055,marker_m=.0107,maximum_rms_px=2.):
    if not np.isfinite([cube_m,marker_m,maximum_rms_px]).all() or not 0<marker_m<cube_m or maximum_rms_px<=0:raise ValueError('Invalid dimensions/tolerance')
    observed={int(k):np.asarray(v,dtype=np.float64).reshape(4,2) for k,v in corners_by_id.items() if int(k) in range(6)}
    if any(not np.isfinite(v).all() for v in observed.values()):raise ValueError('Nonfinite marker corners')
    faces=sorted(observed);out=dict(visible_face_ids=faces,visible_face_names=[FACE_NAMES[f] for f in faces],full_orientation_observable=False,
        hardware_execution_allowed=False,real_devices_opened=False,centre_marker_spin_estimated_per_observation=True)
    if not faces:return dict(out,status='no_cube_markers')
    if any(np.dot(FACE_BASES[a][0],FACE_BASES[b][0])<-.5 for a,b in itertools.combinations(faces,2)):
        return dict(out,status='incompatible_opposite_faces')
    if len(faces)==1:
        from analyze_cube_visibility_pose_20261026 import pose
        f=faces[0];planes=pose(observed[f],marker_m,np.asarray(K),np.asarray(dist));candidates=[]
        for p in planes:
            if p['reprojection_rms_px']<=maximum_rms_px:
                normal=np.asarray(p['normal']);center=np.asarray(p['t'])-normal*cube_m/2
                candidates.append(dict(face_id=f,normal_in_camera=normal.tolist(),cube_center_in_camera_m=center.tolist(),reprojection_rms_px=p['reprojection_rms_px']))
        return dict(out,status='single_face_normal_only' if candidates else 'single_face_fit_failed',normal_candidates=candidates,
                    whole_cube_yaw_not_observable=True)
    image=np.concatenate([observed[f] for f in faces]);solutions=[]
    for spins in itertools.product(range(4),repeat=len(faces)):
        obj=np.concatenate([object_corners(f,k,cube_m,marker_m) for f,k in zip(faces,spins)])
        try:ok,rv,tv=cv2.solvePnP(obj,image,K,dist,flags=cv2.SOLVEPNP_SQPNP)
        except cv2.error:continue
        if not ok or not np.isfinite(rv).all() or not np.isfinite(tv).all():continue
        R=cv2.Rodrigues(rv)[0];t=tv.ravel();pc=(R@obj.T).T+t
        if not np.all(pc[:,2]>0):continue
        if any((R@np.asarray(FACE_BASES[f][0]))@(t+R@np.asarray(FACE_BASES[f][0])*cube_m/2)>=0 for f in faces):continue
        projected=cv2.projectPoints(obj,rv,tv,K,dist)[0].reshape(-1,2);error=float(np.sqrt(np.mean(np.sum((projected-image)**2,axis=1))))
        solutions.append((error,spins,R.copy(),t.copy(),rv.copy(),obj))
    if not solutions:return dict(out,status='multi_face_fit_failed')
    solutions.sort(key=lambda v:v[0]);best=solutions[0];error,spins,R,t,rv,obj=best
    # Refine the same discrete assignment; retain its independently scored rivals.
    rv,tv=cv2.solvePnPRefineLM(obj,image,K,dist,rv,t.reshape(3,1));R=cv2.Rodrigues(rv)[0];t=tv.ravel()
    if (not np.isfinite(R).all() or not np.isfinite(t).all() or not np.all(((R@obj.T).T+t)[:,2]>0)
        or any((R@np.asarray(FACE_BASES[f][0]))@(t+R@np.asarray(FACE_BASES[f][0])*cube_m/2)>=0 for f in faces)):
        return dict(out,status='refined_pose_not_physically_visible')
    projected=cv2.projectPoints(obj,rv,tv,K,dist)[0].reshape(-1,2);error=float(np.sqrt(np.mean(np.sum((projected-image)**2,axis=1))))
    rival=min((s[0] for s in solutions[1:] if np.degrees(np.arccos(np.clip((np.trace(R.T@s[2])-1)/2,-1,1)))>10),default=float('inf'))
    normals={str(f):(R@np.asarray(FACE_BASES[f][0])).tolist() for f in range(6)}
    accepted=bool(error<=maximum_rms_px and rival-error>.25)
    return dict(out,status='multi_face_pose_candidate' if accepted else 'multi_face_pose_uncertain',full_orientation_observable=accepted,
        R_cube_camera=R.tolist(),cube_center_in_camera_m=t.tolist(),face_normals_in_camera=normals,
        marker_quarter_rotations={str(f):int(k) for f,k in zip(faces,spins)},reprojection_rms_px=error,
        distinct_rotation_rival_error_gap_px=None if not np.isfinite(rival) else float(rival-error),
        dimensions_approximate=True,marker_mount_offsets_not_calibrated=True)
