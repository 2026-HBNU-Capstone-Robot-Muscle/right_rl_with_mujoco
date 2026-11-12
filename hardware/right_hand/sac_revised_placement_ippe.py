"""Check both planar solutions for the same-run placement, offline only."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from sac_registration_from_anchor import detect, marker_pose, cube_board, track_cube, candidate
ROOT=Path(__file__).resolve().parent

def main():
    run=json.loads((ROOT/'leap_sac_start_capture_result.json').read_text(encoding='utf-8'))
    folder=Path(run['vision_recording']['folder'])
    with np.load(ROOT/'camera1_720.npz') as z: K,dist=z['K'],z['dist']
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    _,ec,ei,_=detect(folder/'anchor_empty_preflight.png',dictionary)
    Rh,th,_=marker_pose(ec,ei,40,.0282,K,dist)
    Ra,ta,_=marker_pose(ec,ei,41,.030,K,dist)
    Rah=Ra.T@Rh;tah=Ra.T@(th-ta)
    reg=candidate();Rwh=np.asarray(reg['hand_to_world_candidate']);grasp=np.asarray(reg['grasp_site_in_marker_m_candidate'])
    rotations=tuple(json.loads((ROOT/'cube_face_rot_result.json').read_text())['face_rot'])
    objects,boardids=cube_board.object_points(cube_mm=54.5,marker_mm=10.6,layout='centers',face_rot=rotations)
    reset=json.loads((ROOT/'sac_supported_reset_result.json').read_text())['candidates'][0]
    target=np.asarray(reset['qpos']);qt=target[-4:]/np.linalg.norm(target[-4:])
    results=[]
    for name in ('placement_at_space.png','anchor_cube_candidate.png','place_cube_end.png'):
        frame,cs,ids,_=detect(folder/name,dictionary)
        if 41 not in ids or 5 not in ids:
            results.append(dict(image=name,detected_ids=ids,status='missing markers'));continue
        Ra1,ta1,_=marker_pose(cs,ids,41,.030,K,dist)
        corners=np.asarray(cs[ids.index(5)],np.float32).reshape(4,2)
        obj=np.asarray(objects[list(np.asarray(boardids).ravel()).index(5)],np.float32)
        ok,rvs,tvs,_=cv.solvePnPGeneric(obj,corners,K,dist,flags=cv.SOLVEPNP_IPPE)
        solutions=[]
        for rv,tv in zip(rvs,tvs):
            Rc=cv.Rodrigues(rv)[0];tc=tv.ravel()
            ph=Rah.T@(Ra1.T@(tc-ta1)-tah)
            pw=np.array([.11,0,.03])+Rwh@(ph-grasp)
            qw=np.asarray(track_cube.mat_to_quat(Rwh@Rah.T@Ra1.T@Rc));qw/=np.linalg.norm(qw)
            proj=cv.projectPoints(obj,rv,tv,K,dist)[0].reshape(4,2)
            # Predict the physical top face extent from each small-marker fit.
            n,u,v=cube_board.FACES[5];h=.0545/2
            face=np.asarray([h*n+h*x*u+h*y*v for x,y in ((-1,-1),(-1,1),(1,1),(1,-1))])
            facepx=cv.projectPoints(face,rv,tv,K,dist)[0].reshape(4,2)
            solutions.append(dict(reprojection_px=float(np.linalg.norm(proj-corners,axis=1).mean()),
                center_world_m=pw.tolist(),cube_quat_world=qw.tolist(),
                center_distance_from_reset_mm=float(np.linalg.norm(pw-target[-7:-4])*1000),
                orientation_error_deg=float(np.degrees(2*np.arccos(np.clip(abs(qw@qt),0,1)))),
                predicted_top_face_corners_px=facepx.tolist(),
                predicted_face_outside_image=bool(np.any(facepx[:,1]<0) or np.any(facepx[:,0]<0) or
                    np.any(facepx[:,0]>=frame.shape[1]) or np.any(facepx[:,1]>=frame.shape[0]))))
        results.append(dict(image=name,detected_ids=ids,solutions=solutions))
    source=json.loads((ROOT/'sac_stage0_reset_audit_result.json').read_text())
    q0=np.asarray(source['cube_quat_wxyz']);q0/=np.linalg.norm(q0)
    out=dict(status='offline_planar_hypothesis_comparison',results=results,
        reset_settling_orientation_change_deg=float(np.degrees(2*np.arccos(np.clip(abs(q0@qt),0,1)))),
        motor_commands_sent=0,hardware_authorized=False,
        caveat='Both cube solutions share an unverified marker-to-model transform and preflight planar anchor estimates.')
    (ROOT/'sac_revised_placement_ippe_result.json').write_text(json.dumps(out,indent=2)+'\n')
    for r in results:
        print(r['image'],[{k:v for k,v in s.items() if k in ('reprojection_px','center_distance_from_reset_mm','orientation_error_deg','predicted_face_outside_image')} for s in r.get('solutions',[])])
    print('Reset settling rotation:',out['reset_settling_orientation_change_deg'])

if __name__=='__main__':main()
