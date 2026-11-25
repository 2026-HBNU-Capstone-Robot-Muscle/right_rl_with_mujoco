"""Compare archived placement estimates using reported dimensions, without devices."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from sac_registration_from_anchor import detect,marker_pose,candidate,cube_board,track_cube
from anchor_pattern_diagnostic import refine
ROOT=Path(__file__).resolve().parent

def main():
    measured=json.loads((ROOT/'cube_measurement_20261014.json').read_text())
    previous=json.loads((ROOT/'sac_revised_placement_audit_result.json').read_text());folder=Path(previous['source'])
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    _,ec,ei,_=detect(folder/'anchor_empty_preflight.png',dictionary)
    raw,pc,pi,_=detect(folder/'placement_at_space.png',dictionary)
    Rh,th,_=marker_pose(ec,ei,40,.0282,K,dist);Ra0,ta0,_=marker_pose(ec,ei,41,.027,K,dist)
    Ra1,ta1,_=marker_pose(pc,pi,41,.027,K,dist)
    Rrel=Ra0.T@Rh;trel=Ra0.T@(th-ta0)
    fit=json.loads((ROOT/'sac_anchor_bundle_screened_result.json').read_text())
    geometry=candidate();Rwh=np.array(geometry['hand_to_world_candidate']);grasp=np.array(geometry['grasp_site_in_marker_m_candidate'])
    rotations=tuple(json.loads((ROOT/'cube_face_rot_result.json').read_text())['face_rot'])
    corner=np.array(pc[pi.index(5)],np.float64).reshape(4,2)
    rows=[];overlay=raw.copy()
    for label,side,mside,pattern in [('previous_dimensions',54.5,10.6,False),('measured_dimensions',measured['cube_side_mm'],measured['id5_black_outer_side_mm_approximate'],False),('inner_pattern_experiment',measured['cube_side_mm'],measured['id5_black_outer_side_mm_approximate'],True)]:
        obj,bids=cube_board.object_points(cube_mm=side,marker_mm=mside,layout='centers',face_rot=rotations)
        obj=np.asarray(obj[list(np.asarray(bids).ravel()).index(5)],np.float64)
        corners=corner.copy();score=None
        if pattern:corners,score=refine(cv.cvtColor(raw,cv.COLOR_BGR2GRAY),corner,'5')
        ok,rv,tv=cv.solvePnP(obj,corners,K,dist)
        if not ok:raise ValueError('PnP failed')
        projected=cv.projectPoints(obj,rv,tv,K,dist)[0].reshape(4,2)
        t=tv.ravel();R=cv.Rodrigues(rv)[0]
        ph=Rrel.T@(Ra1.T@(t-ta1)-trel);pw=np.array([.11,0,.03])+Rwh@(ph-grasp)
        br=np.array(fit['relative_rotation_41_to_40']);bt=np.array(fit['marker40_origin_in_marker41_m'])
        bpw=np.array([.11,0,.03])+Rwh@(br.T@(Ra1.T@(t-ta1)-bt)-grasp)
        quat=track_cube.mat_to_quat(Rwh@Rrel.T@Ra1.T@R)
        face=obj.mean(axis=0)+(obj-obj.mean(axis=0))*side/mside
        face_px=cv.projectPoints(face,rv,tv,K,dist)[0].reshape(4,2)
        color=(255,0,255) if pattern else (0,255,0) if label=='measured_dimensions' else (0,0,255)
        cv.polylines(overlay,[np.rint(face_px).astype(np.int32)],True,color,2)
        rows.append(dict(case=label,cube_side_mm=side,marker_side_mm=mside,corner_px=corners.tolist(),
                         pattern_correlation=score,marker_reprojection_px=float(np.linalg.norm(projected-corners,axis=-1).mean()),
                         center_world_candidate_m=pw.tolist(),quat_world_candidate_wxyz=quat.tolist(),
                         center_world_cross_session_bundle_candidate_m=bpw.tolist(),
                         camera_depth_m=float(t[2]),experiment_unvalidated=pattern))
    out=dict(status='measured_dimension_offline_comparison',source=str(folder),cases=rows,
             motor_commands_sent=0,hardware_authorized=False,
             limitations=['All camera-to-model transforms remain provisional.',
                          'Single-marker pose is ambiguous; pattern experiment is not a controller.',
                          'User dimensions approximate; no runtime parameters overwritten.'])
    (ROOT/'sac_measured_dimensions_audit_result.json').write_text(json.dumps(out,indent=2)+'\n')
    (ROOT/'sac_measured_dimensions_audit_overlay.png').write_bytes(cv.imencode('.png',overlay)[1].tobytes())
    print(json.dumps(out,indent=2))

if __name__=='__main__':main()
