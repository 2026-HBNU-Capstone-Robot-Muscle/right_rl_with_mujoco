"""Offline two-solution pose test for same-run one-marker cube capture."""

import json
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

from hand_registration_geometry import candidate
from sac_registration_from_anchor import detect, marker_pose


ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'sequence_vision_20261009_221628_356082'
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'vision' / 'rubik_center'))
import cube_board
import track_cube


def main():
    with np.load(ROOT / 'camera1_720.npz') as cal:
        K, dist = cal['K'].copy(), cal['dist'].copy()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    _, ec, ei, _ = detect(FOLDER / 'anchor_empty_stage0.png', dictionary)
    _, pc, pi, _ = detect(FOLDER / 'placement_at_space.png', dictionary)
    Rh, th, _ = marker_pose(ec, ei, 40, .0282, K, dist)
    Ra0, ta0, _ = marker_pose(ec, ei, 41, .030, K, dist)
    Ra1, ta1, _ = marker_pose(pc, pi, 41, .030, K, dist)
    marker_corners = np.asarray(next(c for c, i in zip(pc, pi) if i == 5)).reshape(4, 2)
    rot = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    points, ids = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                            layout='centers', face_rot=rot)
    obj = points[int(np.where(ids == 5)[0][0])]
    ok, rvecs, tvecs, _ = cv.solvePnPGeneric(
        obj, marker_corners, K, dist, flags=cv.SOLVEPNP_IPPE)
    if not ok or len(rvecs) != 2:
        raise RuntimeError('Expected two planar-PnP solutions')
    reg = candidate()
    Rwh = np.asarray(reg['hand_to_world_candidate'])
    grasp = np.asarray(reg['grasp_site_in_marker_m_candidate'])
    Rah = Ra0.T @ Rh
    tah = Ra0.T @ (th - ta0)
    rows = []
    for index, (rv, tv) in enumerate(zip(rvecs, tvecs)):
        Rc, _ = cv.Rodrigues(rv)
        tc = tv.ravel()
        projected, _ = cv.projectPoints(obj, rv, tv, K, dist)
        err = float(np.linalg.norm(projected.reshape(4, 2) - marker_corners, axis=1).mean())
        cube_anchor = Ra1.T @ (tc - ta1)
        cube_hand = Rah.T @ (cube_anchor - tah)
        Rwc = Rwh @ Rah.T @ Ra1.T @ Rc
        cube_world = np.array([.11, 0, .03]) + Rwh @ (cube_hand - grasp)
        rows.append({'solution': index, 'reprojection_px': err,
                     'green_face5_up_dot_candidate': float((Rwc @ np.array([0., -1., 0.]))[2]),
                     'cube_center_hand_marker_m': cube_hand.tolist(),
                     'cube_center_world_m': cube_world.tolist(),
                     'cube_quat_world_candidate': np.asarray(track_cube.mat_to_quat(Rwc)).tolist()})
    report = {'status': 'offline_same_run_ippe_diagnostic', 'hardware_authorized': False,
              'source_folder': str(FOLDER), 'solutions': rows,
              'center_separation_mm': float(1000 * np.linalg.norm(
                  np.asarray(rows[0]['cube_center_world_m']) - rows[1]['cube_center_world_m'])),
              'caveat': 'One small planar marker allows two 3D poses; color and contact are independent checks.'}
    (ROOT / 'sac_anchor_ippe_20261009_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
