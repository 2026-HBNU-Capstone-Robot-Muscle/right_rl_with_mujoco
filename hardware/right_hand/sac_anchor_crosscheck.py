"""Compare new multiview anchor against archived same-run references, offline."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from sac_registration_from_anchor import detect,marker_pose,candidate,cube_board,track_cube

ROOT=Path(__file__).resolve().parent

def main():
    fit=json.loads((ROOT/'sac_anchor_bundle_screened_result.json').read_text())
    if not fit['diagnostic_fit_passed']:raise ValueError('Bundle diagnostic failed')
    Ra_h=np.array(fit['relative_rotation_41_to_40']);ta_h=np.array(fit['marker40_origin_in_marker41_m'])
    audit=json.loads((ROOT/'sac_revised_placement_audit_result.json').read_text());folder=Path(audit['source'])
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    h=.0282/2;obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float32)
    rows=[]
    for name in ('anchor_empty_preflight.png','anchor_empty_stage0.png'):
        _,c,ids,_=detect(folder/name,dictionary)
        Rh,th,_=marker_pose(c,ids,40,.0282,K,dist);Ra,ta,_=marker_pose(c,ids,41,.027,K,dist)
        predicted_R=Ra@Ra_h;predicted_t=ta+Ra@ta_h
        pixels=cv.projectPoints(obj,cv.Rodrigues(predicted_R)[0],predicted_t,K,dist)[0].reshape(4,2)
        measured=np.asarray(c[ids.index(40)]).reshape(4,2)
        angle=np.degrees(np.arccos(np.clip((np.trace(Ra_h.T@Ra.T@Rh)-1)/2,-1,1)))
        rows.append(dict(image=name,relation_translation_difference_mm=float(np.linalg.norm(Ra.T@(th-ta)-ta_h)*1000),
                         relation_rotation_difference_deg=float(angle),
                         marker40_prediction_mean_px=float(np.linalg.norm(pixels-measured,axis=-1).mean())))
    # Comparison only. Do not overwrite the same-run estimate or any runtime transform.
    raw,c,ids,det=detect(folder/'placement_at_space.png',dictionary)
    Ra,ta,_=marker_pose(c,ids,41,.027,K,dist)
    rotations=tuple(json.loads((ROOT/'cube_face_rot_result.json').read_text())['face_rot'])
    points,bids=cube_board.object_points(cube_mm=54.5,marker_mm=10.6,layout='centers',face_rot=rotations)
    cube=track_cube.step(raw,cv.aruco.Board(points,dictionary,bids),det,dictionary,K,dist)
    ph=Ra_h.T@(Ra.T@(np.array(cube['t_cube'])-ta)-ta_h)
    Rhc=Ra_h.T@Ra.T@np.array(cube['R_cube'])
    geometry=candidate();Rwh=np.array(geometry['hand_to_world_candidate'])
    pw=np.array([.11,0,.03])+Rwh@(ph-geometry['grasp_site_in_marker_m_candidate'])
    result=dict(status='cross_session_diagnostic_only',motor_commands_sent=0,hardware_authorized=False,
                archived_source=str(folder),anchor_fit='sac_anchor_bundle_screened_result.json',comparisons=rows,
                hypothetical_cube_world_m=pw.tolist(),hypothetical_cube_quaternion_wxyz=track_cube.mat_to_quat(Rwh@Rhc).tolist(),
                change_from_original_same_run_mm=((pw-audit['registration']['cube_center_in_sim_world_candidate_m'])*1000).tolist(),
                limitations=['Mount constancy across sessions is not assumed.',
                             'Low image residual does not establish metric dimensions or marker-to-model calibration.',
                             'This hypothetical pose is not installed in a controller.'])
    (ROOT/'sac_anchor_crosscheck_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
