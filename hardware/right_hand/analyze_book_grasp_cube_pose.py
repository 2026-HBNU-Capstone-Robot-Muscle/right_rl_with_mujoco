"""Same-frame palm/cube pose hypotheses from the completed book grasp.

Uses saved frames and contemporaneous encoder samples only. Both planar pose
branches are retained. The marker-to-model transform remains unregistered.
"""
import hashlib
import itertools
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from hand_registration_geometry import candidate
from review_prepared_cube_20261022 import marker_solutions
from analyze_readonly_direct_pose_20261023 import analyze_grid

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'book_cube_grasp_trial_20261024_212855_824047'
data=(FOLDER/'result.json').read_bytes();trial=json.loads(data)
if trial['status']!='book_cube_grasp_trial_complete' or not trial['all_torque_off_verified']:
    raise ValueError('Incomplete source')
frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
samples=trial['samples'];reg=candidate()
Rwm=np.array(reg['hand_to_world_candidate']);gm=np.array(reg['grasp_site_in_marker_m_candidate'])
with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
params=cv.aruco.DetectorParameters();params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_SUBPIX
detector=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),params)
selected=[f for f in frames if f['phase'] in ('stage_2_fixed_hold','grasp_observation')]
selected+=[f for f in frames if f['phase']=='remove_cube'][:8]
observations=[];hypotheses=[]
gravity=np.array(json.loads((ROOT/'sac_stage0_reset_audit_result.json').read_bytes())['gravity_m_s2'])
up=-gravity/np.linalg.norm(gravity)
for f in selected:
    image_bytes=(FOLDER/'camera'/f['image']).read_bytes()
    image=cv.imdecode(np.frombuffer(image_bytes,np.uint8),cv.IMREAD_COLOR)
    corners,ids,_=detector.detectMarkers(cv.cvtColor(image,cv.COLOR_BGR2GRAY))
    ids=[] if ids is None else ids.ravel().tolist()
    sample=min(samples,key=lambda s:abs(trial['enabled_at']+s['t']-f['camera_read_monotonic']))
    row=dict(image='camera/'+f['image'],image_sha256=hashlib.sha256(image_bytes).hexdigest(),
        ids=ids,phase=f['phase'],sample_phase=sample['phase'],actual_ticks=sample['actual'],
        encoder_time_difference_s=abs(trial['enabled_at']+sample['t']-f['camera_read_monotonic']),
        marker_corners={str(i):np.asarray(c).reshape(4,2).tolist() for i,c in zip(ids,corners)})
    palms=marker_solutions(corners,ids,40,.0282,K,dist)
    cubes=marker_solutions(corners,ids,5,.0107,K,dist)
    grid=None
    if 5 in ids:
        try:
            grid,_=analyze_grid(image,row['marker_corners']['5'],K,dist)
            Rg=np.array(grid['grid_rotation_camera']);tg=np.array(grid['grid_plane_center_camera_m'])
            if Rg[:,2]@tg>0:Rg=Rg@np.diag([1.,-1.,-1.])
            grid['cube_center_camera_m']=(tg-Rg[:,2]*.0275).tolist()
            grid['cube_rotation_camera']=Rg.tolist()
            row['grid_diagnostic']=grid
        except ValueError as exc:row['grid_error']=str(exc)
    for branch,(Rp,tp,ep) in enumerate(palms):
        Rcw=Rp@Rwm.T;tcw=tp+Rp@gm-Rcw@np.array([.11,0,.03])
        poses=[('marker',i,Rc,tc-Rc[:,2]*.0275,ec) for i,(Rc,tc,ec) in enumerate(cubes)]
        if grid:
            poses.append(('grid_diagnostic',None,np.array(grid['cube_rotation_camera']),
                np.array(grid['cube_center_camera_m']),grid['grid_pnp_mean_error_px']))
        for kind,cbranch,Rc,center,err in poses:
            worldR=Rcw.T@Rc;worldcenter=Rcw.T@(center-tcw)
            hypotheses.append(dict(image=row['image'],source=kind,palm_branch=branch,cube_branch=cbranch,
                marker_errors_px=[ep,err],cube_center_world_m=worldcenter.tolist(),
                cube_rotation_world=worldR.tolist(),world_to_camera_rotation=Rcw.tolist(),
                world_to_camera_translation=tcw.tolist(),actual_ticks=sample['actual'],
                top_to_assumed_gravity_up_degrees=float(np.degrees(np.arccos(np.clip(worldR[:,2]@up,-1,1))))))
    observations.append(row)
summary=[]
for key in sorted({(h['source'],h['palm_branch'],h['cube_branch']) for h in hypotheses},key=str):
    hs=[h for h in hypotheses if (h['source'],h['palm_branch'],h['cube_branch'])==key]
    centers=np.array([h['cube_center_world_m'] for h in hs])
    summary.append(dict(source=key[0],palm_branch=key[1],cube_branch=key[2],frames=len(hs),
        median_center_m=np.median(centers,axis=0).tolist(),spread_mm=(np.ptp(centers,axis=0)*1000).tolist(),
        median_gravity_up_error_deg=float(np.median([h['top_to_assumed_gravity_up_degrees'] for h in hs])),
        maximum_reprojection_sum_px=max(sum(h['marker_errors_px']) for h in hs)))
out=dict(source=FOLDER.name,source_sha256=hashlib.sha256(data).hexdigest(),observations=observations,
    hypotheses=hypotheses,branch_summaries=summary,marker_to_model_registration_verified=False,
    cube_side_mm=55.,cube_marker_mm=10.7,palm_marker_mm=28.2,
    solved_color_grid_used_only_for_this_offline_diagnostic=True,
    hardware_execution_allowed=False,hardware_commands_sent=0)
(FOLDER/'cube_pose_diagnostic.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(dict(frames=len(observations),hypotheses=len(hypotheses),summary=summary,
    detected_ids=[r['ids'] for r in observations]),ensure_ascii=False))
