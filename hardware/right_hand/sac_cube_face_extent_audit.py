"""Independent, explicitly manual outer-face extent diagnostic; no live tracking."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from sac_registration_from_anchor import detect,cube_board,marker_pose,candidate,track_cube
ROOT=Path(__file__).resolve().parent

def main():
    folder=ROOT/'sequence_vision_20261013_213113_258536'
    raw,c,ids,_=detect(folder/'placement_at_space.png',cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50))
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    rotations=tuple(json.loads((ROOT/'cube_face_rot_result.json').read_text())['face_rot'])
    pts,bids=cube_board.object_points(cube_mm=54.5,marker_mm=10.6,layout='centers',face_rot=rotations)
    obj=np.array(pts[list(np.asarray(bids).ravel()).index(5)],np.float64)
    marker=np.array(c[ids.index(5)]).reshape(4,2)
    center=obj.mean(axis=0);face=center+(obj-center)*(54.5/10.6)
    ok,rv,tv=cv.solvePnP(obj,marker,K,dist)
    predicted=cv.projectPoints(face,rv,tv,K,dist)[0].reshape(4,2)
    # Approximate intersections of the four OUTER white-plastic top edges.
    # Upper-left is near the frame border, so these are not survey-grade points.
    observed=np.array([[956,18],[951,263],[615,232],[652,0]],np.float64)
    face_xy=np.array([[54.5,0],[54.5,54.5],[0,54.5],[0,0]],np.float32)
    un_face=cv.undistortPoints(observed[:,None],K,dist,P=K).reshape(4,2)
    un_marker=cv.undistortPoints(marker[:,None],K,dist,P=K).reshape(4,2)
    H=cv.getPerspectiveTransform(un_face.astype(np.float32),face_xy)
    recovered=cv.perspectiveTransform(un_marker[None].astype(np.float32),H)[0]
    sides=np.linalg.norm(recovered-np.roll(recovered,1,axis=0),axis=1)
    ok,fr,ft=cv.solvePnP(face,observed,K,dist)
    face_fit=cv.projectPoints(face,fr,ft,K,dist)[0].reshape(4,2)
    # Pixel-annotation sensitivity, not a calibrated confidence interval.
    rng=np.random.default_rng(20261014);samples=[]
    for _ in range(300):
        jitter=observed+rng.uniform(-2,2,observed.shape)
        und=cv.undistortPoints(jitter[:,None],K,dist,P=K).reshape(4,2)
        h=cv.getPerspectiveTransform(und.astype(np.float32),face_xy)
        mp=cv.perspectiveTransform(un_marker[None].astype(np.float32),h)[0]
        samples.append(np.linalg.norm(mp-np.roll(mp,1,axis=0),axis=1).mean())
    result=dict(status='manual_extent_diagnostic_only',source=str(folder/'placement_at_space.png'),
                hardware_authorized=False,motor_commands_sent=0,assumed_cube_side_mm=54.5,
                manual_face_corners_px=observed.tolist(),marker_side_lengths_in_face_plane_mm=sides.tolist(),
                marker_mean_side_mm=float(sides.mean()),annotation_2px_sensitivity_min_max_mm=[float(min(samples)),float(max(samples))],
                face_pose_mean_reprojection_px=float(np.linalg.norm(face_fit-observed,axis=1).mean()),
                face_pose_camera_center_m=ft.ravel().tolist(),
                marker_pose_camera_center_m=tv.ravel().tolist(),
                limitations=['Manual approximate boundary intersections; rounded edges and top image clipping introduce bias.',
                             'Assumes cube is 54.5mm wide and marker is coplanar with its top.',
                             'No marker dimension is fitted to a desired simulation position.',
                             'This diagnostic does not change marker dimensions or runtime calibration.'])
    (ROOT/'sac_cube_face_extent_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    cv.polylines(raw,[np.rint(predicted).astype(np.int32)],True,(0,0,255),2)
    cv.polylines(raw,[np.rint(observed).astype(np.int32)],True,(0,255,0),2)
    for i,p in enumerate(predicted):cv.putText(raw,str(i),tuple(np.rint(p).astype(int)),cv.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),2)
    (ROOT/'sac_cube_face_extent_overlay.png').write_bytes(cv.imencode('.png',raw)[1].tobytes())

if __name__=='__main__':main()
