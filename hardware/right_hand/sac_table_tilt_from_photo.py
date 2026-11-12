"""Estimate hand-marker plane tilt relative to a flat tabletop chessboard.

This estimates mounting orientation only; it does not verify cube contact or
authorize a motor policy. Both planar PnP branches are reported.
"""

import json
from pathlib import Path

import cv2 as cv
import numpy as np

from hand_registration_geometry import candidate

ROOT = Path(__file__).resolve().parent
BOARD = (9,6)
SQUARE_M = .024  # camera_calibrate.py: printed nominal 25 mm measured as 24 mm


def analyze(folder):
    folder = Path(folder)
    with np.load(ROOT/'camera1_720.npz') as f:
        K, dist = f['K'].copy(), f['dist'].copy()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    detector = cv.aruco.ArucoDetector(dictionary, cv.aruco.DetectorParameters())
    board_obj = np.zeros((BOARD[0]*BOARD[1],3),np.float32)
    board_obj[:,:2] = np.mgrid[0:BOARD[0],0:BOARD[1]].T.reshape(-1,2)*SQUARE_M
    half = .0282/2
    marker_obj = np.asarray([[-half,half,0],[half,half,0],
                             [half,-half,0],[-half,-half,0]],np.float32)
    R_wm = np.asarray(candidate()['hand_to_world_candidate'])
    rows=[]
    for path in sorted(folder.glob('frame_*.png')):
        image=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)
        gray=cv.cvtColor(image,cv.COLOR_BGR2GRAY)
        found, chess = cv.findChessboardCornersSB(gray,BOARD,
                                                 flags=cv.CALIB_CB_NORMALIZE_IMAGE)
        markers, ids, _ = detector.detectMarkers(gray)
        if not found or ids is None or 40 not in ids:
            continue
        image_board=np.asarray(chess,np.float32).reshape(-1,2)
        ok,rv_b,tv_b=cv.solvePnP(board_obj,image_board,K,dist)
        if not ok:
            continue
        R_b=cv.Rodrigues(rv_b)[0]
        camera_in_board=-R_b.T@tv_b.ravel()
        upward=R_b[:,2]*np.sign(camera_in_board[2])
        image_marker=np.asarray(markers[list(ids.ravel()).index(40)],np.float32).reshape(4,2)
        count, rvecs, tvecs, _=cv.solvePnPGeneric(
            marker_obj,image_marker,K,dist,flags=cv.SOLVEPNP_IPPE)
        branches=[]
        for rv_m,tv_m in zip(rvecs,tvecs):
            R_m=cv.Rodrigues(rv_m)[0]
            projected,_=cv.projectPoints(marker_obj,rv_m,tv_m,K,dist)
            error=float(np.linalg.norm(projected.reshape(4,2)-image_marker,axis=1).mean())
            angle=float(np.degrees(np.arccos(np.clip(abs(float(R_m[:,2]@upward)),-1,1))))
            gravity_marker=R_m.T@(-9.81*upward)
            gravity_sim=R_wm@gravity_marker
            branches.append({'marker_reprojection_px':error,
                             'marker_table_angle_deg':angle,
                             'gravity_in_sim_world_m_s2':gravity_sim.tolist()})
        board_projected,_=cv.projectPoints(board_obj,rv_b,tv_b,K,dist)
        board_error=float(np.linalg.norm(board_projected.reshape(-1,2)-image_board,axis=1).mean())
        rows.append({'image':path.name,'board_reprojection_px':board_error,
                     'branches':sorted(branches,key=lambda x:x['marker_reprojection_px'])})
    if len(rows)<3:
        raise RuntimeError('Need at least three usable board + ID40 frames')
    selected=[r['branches'][0] for r in rows]
    angles=[r['marker_table_angle_deg'] for r in selected]
    gravity=np.median(np.asarray([r['gravity_in_sim_world_m_s2'] for r in selected]),axis=0)
    report={'status':'offline_table_orientation_measurement','motor_port_opened':False,
            'frames_used':len(rows),'marker_table_angle_deg':float(np.median(angles)),
            'angle_range_deg':[float(min(angles)),float(max(angles))],
            'gravity_in_sim_world_m_s2_candidate':gravity.tolist(),
            'rows':rows,
            'caveat':'Board must lie flat on a horizontal rigid table; planar ID40 PnP branch and candidate marker-to-sim transform remain uncertain. This is not policy approval.'}
    (folder/'tilt_result.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    import sys
    if len(sys.argv)!=2:
        raise SystemExit('Usage: python sac_table_tilt_from_photo.py FOLDER')
    print(json.dumps(analyze(sys.argv[1]),indent=2))
