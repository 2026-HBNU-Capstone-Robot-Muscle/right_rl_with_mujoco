"""Direct same-frame marker poses with contemporaneous OFF encoder pairs.

Both planar PnP branches are retained. Marker/model registration is not fitted
to cube contact. A solved-green grid is only an independent scale diagnostic.
"""
import hashlib
import itertools
import json
from pathlib import Path

import cv2 as cv
import numpy as np

from hand_registration_geometry import candidate
from review_prepared_cube_20261022 import marker_solutions

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_pose_readonly_20261023_171233_601943'


def analyze_grid(raw,marker,K,dist):
    """Independent solved-face diagnostic with explicit saturation sensitivity.

The older S>=55 mask joins cyan-lit plastic to green stickers in this view.
Keep four stricter masks and their centroid spread instead of altering pixels.
"""
    marker=np.asarray(marker,float);center=marker.mean(0)
    scale=np.mean(np.linalg.norm(marker-np.roll(marker,1,axis=0),axis=1))
    hsv=cv.cvtColor(raw,cv.COLOR_BGR2HSV);candidates=[]
    for saturation in (80,100,120,140):
        mask=cv.inRange(hsv,np.array([35,saturation,35]),np.array([100,255,255]))
        _,labels,stats,centers=cv.connectedComponentsWithStats(mask)
        pts=[]
        for stat,point in zip(stats[1:],centers[1:]):
            if .45*scale**2<stat[4]<6*scale**2 and .8*scale<np.linalg.norm(point-center)<3.2*scale:
                pts.append(point)
        if len(pts)!=8:continue
        pts=np.asarray(pts);pts=pts[np.argsort(pts[:,1])]
        pts=np.concatenate([p[np.argsort(p[:,0])] for p in (pts[:3],pts[3:5],pts[5:])])
        candidates.append(dict(saturation=saturation,centers=pts.tolist()))
    if len(candidates)<3:raise ValueError('Eight surrounding green tiles not stable across three masks')
    pts=np.median([x['centers'] for x in candidates],axis=0)
    grid=np.array([[(i%3-1)*55/3,(i//3-1)*55/3] for i in range(9) if i!=4],float)
    und=cv.undistortPoints(pts[:,None],K,dist,P=K).reshape(-1,2)
    H,_=cv.findHomography(grid,und,0)
    prediction=cv.perspectiveTransform(grid[None],H)[0]
    undmarker=cv.undistortPoints(marker[:,None],K,dist,P=K).reshape(-1,2)
    marker_on_grid=cv.perspectiveTransform(undmarker[None],np.linalg.inv(H))[0]
    objects=np.c_[grid,np.zeros(8)]/1000
    ok,rv,tv=cv.solvePnP(objects,pts,K,dist,flags=cv.SOLVEPNP_ITERATIVE)
    if not ok:raise ValueError('Grid PnP failed')
    prediction_rigid=cv.projectPoints(objects,rv,tv,K,dist)[0].reshape(-1,2)
    return dict(grid_centers_px=pts.tolist(),saturation_sensitivity=candidates,
        maximum_mask_centroid_spread_px=float(np.max(np.linalg.norm(np.array([x['centers'] for x in candidates])-pts,axis=2))),
        homography_mean_error_px=float(np.mean(np.linalg.norm(prediction-und,axis=1))),
        marker_side_from_grid_mm=np.linalg.norm(marker_on_grid-np.roll(marker_on_grid,1,axis=0),axis=1).tolist(),
        marker_center_in_grid_mm=marker_on_grid.mean(0).tolist(),
        grid_pnp_mean_error_px=float(np.mean(np.linalg.norm(prediction_rigid-pts,axis=1))),
        grid_plane_center_camera_m=tv.ravel().tolist(),grid_rotation_camera=cv.Rodrigues(rv)[0].tolist()),pts


def main():
    blob=(FOLDER/'result.json').read_bytes();source=json.loads(blob)
    assert source['status']=='readonly_cube_pose_pairs_saved'
    assert source['all_completed_pairs_have_off_endpoints'] and source['camera_restoration_verified']
    reg=candidate();Rwm=np.array(reg['hand_to_world_candidate']);gm=np.array(reg['grasp_site_in_marker_m_candidate'])
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    rows=[];hypotheses=[];grids=[]
    for pair in source['pairs']:
        if not pair['usable_paired_observation']:continue
        path=FOLDER/pair['image'];image_blob=path.read_bytes()
        assert hashlib.sha256(image_blob).hexdigest()==pair['image_sha256']
        points=pair['markers']['corners']
        poses40=marker_solutions([np.array(points['40'])],[40],40,.0282,K,dist)
        poses5=marker_solutions([np.array(points['5'])],[5],5,.0107,K,dist)
        try:
            image=cv.imdecode(np.frombuffer(image_blob,np.uint8),cv.IMREAD_COLOR)
            grid,_=analyze_grid(image,points['5'],K,dist)
            Rg=np.array(grid['grid_rotation_camera']);tg=np.array(grid['grid_plane_center_camera_m'])
            normal=Rg[:,2].copy()
            if normal@tg>0:normal*=-1;Rg=Rg@np.diag([1.,-1.,-1.])
            grid['cube_center_camera_m']=(tg-normal*.0275).tolist()
            grid['cube_rotation_camera']=Rg.tolist()
            grid['top_outward_normal_camera']=normal.tolist()
            grids.append(dict(image=pair['image'],**grid))
        except ValueError as exc:grid=dict(error=str(exc))
        for (ip,(Rp,tp,ep)),(ic,(Rc,tc,ec)) in itertools.product(enumerate(poses40),enumerate(poses5)):
            Rcw=Rp@Rwm.T;tcw=tp+Rp@gm-Rcw@np.array([.11,0,.03])
            normal=Rc[:,2];center=tc-normal*.0275
            hypotheses.append(dict(image=pair['image'],source='direct_ID40_and_ID5',palm_branch=ip,cube_branch=ic,
                marker_errors_px=[ep,ec],cube_center_camera_m=center.tolist(),cube_rotation_camera=Rc.tolist(),
                cube_center_marker40_m=(Rp.T@(center-tp)).tolist(),
                cube_top_rotation_marker40=(Rp.T@Rc).tolist(),
                cube_center_world_m=(Rcw.T@(center-tcw)).tolist(),cube_rotation_world=(Rcw.T@Rc).tolist(),
                world_to_camera_rotation=Rcw.tolist(),world_to_camera_translation=tcw.tolist(),
                measured_motor_ticks=pair['before']['snapshot']['position_ticks'],
                encoder_endpoint_max_drift_ticks=pair['maximum_drift_ticks']))
        if 'cube_center_camera_m' in grid:
            for ip,(Rp,tp,ep) in enumerate(poses40):
                Rcw=Rp@Rwm.T;tcw=tp+Rp@gm-Rcw@np.array([.11,0,.03])
                center=np.array(grid['cube_center_camera_m']);Rc=np.array(grid['cube_rotation_camera'])
                hypotheses.append(dict(image=pair['image'],source='direct_ID40_and_solved_green_grid_diagnostic',
                    palm_branch=ip,cube_branch=None,marker_errors_px=[ep,grid['grid_pnp_mean_error_px']],
                    cube_center_camera_m=center.tolist(),cube_rotation_camera=Rc.tolist(),
                    cube_center_marker40_m=(Rp.T@(center-tp)).tolist(),cube_top_rotation_marker40=(Rp.T@Rc).tolist(),
                    cube_center_world_m=(Rcw.T@(center-tcw)).tolist(),cube_rotation_world=(Rcw.T@Rc).tolist(),
                    world_to_camera_rotation=Rcw.tolist(),world_to_camera_translation=tcw.tolist(),
                    measured_motor_ticks=pair['before']['snapshot']['position_ticks'],
                    encoder_endpoint_max_drift_ticks=pair['maximum_drift_ticks']))
        rows.append(dict(image=pair['image'],marker_corners=points,marker40_solutions=len(poses40),
            marker5_solutions=len(poses5),grid_diagnostic=grid,
            camera_read_monotonic=pair['camera_read_monotonic'],before=pair['before'],after=pair['after']))
    summaries=[]
    for key in sorted({(h['source'],h['palm_branch'],h['cube_branch']) for h in hypotheses},key=str):
        hs=[h for h in hypotheses if (h['source'],h['palm_branch'],h['cube_branch'])==key]
        centers=np.array([h['cube_center_marker40_m'] for h in hs])*1000
        summaries.append(dict(source=key[0],palm_branch=key[1],cube_branch=key[2],frames=len(hs),
            median_center_marker40_mm=np.median(centers,axis=0).tolist(),
            coordinate_spread_mm=np.ptp(centers,axis=0).tolist(),
            maximum_reprojection_sum_px=max(sum(h['marker_errors_px']) for h in hs)))
    result=dict(status='direct_palm_cube_encoders_offline_diagnostic',source=FOLDER.name,
        source_sha256=hashlib.sha256(blob).hexdigest(),directly_paired_frames=len(rows),
        source_marker_sizes_mm=dict(palm40=28.2,cube5=10.7),cube_side_mm=55,
        rows=rows,hypotheses=hypotheses,branch_summaries=summaries,
        marker_to_model_candidate=reg,old_joint_values_used=False,old_palm_bridge_used=False,
        marker_to_model_registration_verified=False,hardware_execution_allowed=False,
        real_camera_or_serial_opened=False,servo_writes_sent=0,
        limitations=['Small marker pose depth is sensitive to corner and physical-size error.',
            'The solved-green equal-tile-pitch grid is a diagnostic; it is not scrambled-cube tracking.',
            'Repeatability across 10 photos is not absolute geometry accuracy.',
            'Both IPPE branches and unchanged provisional marker/model registration are retained.'])
    (FOLDER/'direct_marker_pose_review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('Direct same-frame marker/encoder observations',len(rows),'hypotheses',len(hypotheses))
    print(json.dumps(summaries,indent=2))
    print('GRID',[(r['image'],r['grid_diagnostic'].get('grid_pnp_mean_error_px'),r['grid_diagnostic'].get('error')) for r in rows])


if __name__=='__main__':main()
