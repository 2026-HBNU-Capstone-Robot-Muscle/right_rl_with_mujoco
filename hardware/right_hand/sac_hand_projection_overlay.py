"""Project unchanged model geometry into the existing empty-pose photograph."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from sac_registration_from_anchor import detect,marker_pose,candidate
ROOT=Path(__file__).resolve().parent

def main():
    geometry=json.loads((ROOT/'sac_hand_projection_geometry.json').read_text())
    folder=Path(geometry['source_folder'])
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    _,cs,ids,_=detect(folder/'anchor_empty_preflight.png',dictionary)
    Rh,th,_=marker_pose(cs,ids,40,.0282,K,dist)
    Ra,ta,_=marker_pose(cs,ids,41,.027,K,dist)
    frame,cs1,ids1,_=detect(folder/'revised_start_final_empty_diagnostic.png',dictionary)
    Ra1,ta1,_=marker_pose(cs1,ids1,41,.027,K,dist)
    registration=candidate();Rwh=np.asarray(registration['hand_to_world_candidate'])
    marker_world=np.array([.11,0,.03])-Rwh@np.asarray(registration['grasp_site_in_marker_m_candidate'])
    Rcw=Ra1@Ra.T@Rh@Rwh.T
    tcw=ta1+Ra1@Ra.T@(th-ta)-Rcw@marker_world
    rv=cv.Rodrigues(Rcw)[0]
    overlay=frame.copy();rows=[]
    for mesh in geometry['meshes']:
        points=np.asarray(mesh['vertices_world_m'])
        if np.any((points@Rcw.T+tcw)[:,2]<=0):continue
        px=cv.projectPoints(points,rv,tcw,K,dist)[0].reshape(-1,2)
        if not np.isfinite(px).all():continue
        hull=cv.convexHull(px.astype(np.float32)).astype(np.int32)
        cv.polylines(overlay,[hull],True,(255,170,0),1,cv.LINE_AA)
    for anchor in geometry['joint_anchors']:
        px=cv.projectPoints(np.asarray([anchor['world_m']]),rv,tcw,K,dist)[0].reshape(2)
        point=tuple(np.round(px).astype(int));ident=anchor['motor_id']
        cv.drawMarker(overlay,point,(0,0,255),cv.MARKER_CROSS,12,1)
        cv.putText(overlay,str(ident),(point[0]+6,point[1]-4),cv.FONT_HERSHEY_SIMPLEX,.45,(0,0,255),1)
        rows.append(dict(motor_id=ident,projected_px=px.tolist()))
    cv.putText(overlay,'OFFLINE MODEL PROJECTION | cyan: model hull; red: joint axis',
               (10,705),cv.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
    (ROOT/'sac_hand_projection_overlay.png').write_bytes(cv.imencode('.png',overlay)[1].tobytes())
    report=dict(status='unverified_geometry_projection',source=str(folder),anchor_side_mm=27,
                joint_anchors=rows,R_camera_world=Rcw.tolist(),t_camera_world_m=tcw.tolist(),
                hardware_authorized=False,motor_commands_sent=0,
                caveat=geometry['caveat']+' Image correspondences have not been independently measured.')
    (ROOT/'sac_hand_projection_overlay_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Saved sac_hand_projection_overlay.png; diagnostic only.')

if __name__=='__main__':main()
