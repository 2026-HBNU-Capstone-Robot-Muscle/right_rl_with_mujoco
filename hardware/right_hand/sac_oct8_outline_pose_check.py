"""Alternative Oct-8 cube pose from green outline, for sensitivity only."""
import json
from pathlib import Path
import sys

import cv2
import numpy as np

from hand_registration_geometry import candidate
from sac_registration_from_anchor import detect, marker_pose

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/vision/rubik_center'))
import cube_board
import track_cube


def main():
    with np.load(ROOT / 'camera1_720.npz') as cal:
        K, dist = cal['K'].copy(), cal['dist'].copy()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    old = ROOT / 'sequence_vision_20261006_203648_332236/anchor_empty_stage0.png'
    new = ROOT / 'sequence_vision_20261008_184238_359328/placement_at_space.png'
    _, old_corners, old_ids, _ = detect(old, dictionary)
    R_h0, t_h0, _ = marker_pose(old_corners, old_ids, 40, .0282, K, dist)
    R_a0, t_a0, _ = marker_pose(old_corners, old_ids, 41, .030, K, dist)
    R_ah = R_a0.T @ R_h0
    t_ah = R_a0.T @ (t_h0 - t_a0)
    frame, corners, ids, _ = detect(new, dictionary)
    R_a1, t_a1, _ = marker_pose(corners, ids, 41, .030, K, dist)
    outline = json.loads((ROOT / 'sac_cube_top_outline_oct8_h40_result.json').read_text(encoding='utf-8'))
    polygon = np.asarray(outline['green_face_polygon_px'], np.float32)
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    points, _ = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                         layout='centers', face_rot=rotations)
    marker5 = next(np.asarray(c, np.float32).reshape(4, 2)
                   for c, i in zip(corners, ids) if i == 5)
    normal, u0, v0 = cube_board.FACES[5]
    u, v = cube_board._rot_uv(u0, v0, rotations[5])
    h, s = 54.5 / 2000, 50.5 / 2000
    center = h * normal
    obj = np.asarray([center-s*u+s*v, center+s*u+s*v,
                      center+s*u-s*v, center-s*u-s*v], np.float32)
    choices = []
    for reverse in (False, True):
        base = polygon[::-1] if reverse else polygon
        for shift in range(4):
            img = np.roll(base, -shift, axis=0).astype(np.float32)
            ok, rv, tv = cv2.solvePnP(obj, img, K, dist)
            if not ok:
                continue
            projected, _ = cv2.projectPoints(points[5], rv, tv, K, dist)
            err = float(np.linalg.norm(projected.reshape(4, 2) - marker5, axis=1).mean())
            choices.append((err, rv, tv))
    err, rv, tv = min(choices, key=lambda x: x[0])
    R_c1, t_c1 = cv2.Rodrigues(rv)[0], tv.reshape(3)
    center_a = R_a1.T @ (t_c1 - t_a1)
    R_ca = R_a1.T @ R_c1
    center_h = R_ah.T @ (center_a - t_ah)
    R_ch = R_ah.T @ R_ca
    reg = candidate()
    R_wh = np.asarray(reg['hand_to_world_candidate'])
    grasp = np.asarray(reg['grasp_site_in_marker_m_candidate'])
    center_w = np.array([.11, 0, .03]) + R_wh @ (center_h - grasp)
    quat_w = track_cube.mat_to_quat(R_wh @ R_ch)
    marker = json.loads((ROOT / 'sac_cross_session_latest_registration_result.json').read_text(encoding='utf-8'))
    report = {
        'status': 'outline_sensitivity_only', 'hardware_authorized': False,
        'sticker_extent_assumption_mm': 50.5,
        'marker5_reprojection_disagreement_px': err,
        'cube_center_in_hand_marker_m': center_h.tolist(),
        'cube_center_in_sim_world_candidate_m': center_w.tolist(),
        'cube_quat_world_candidate': np.asarray(quat_w).tolist(),
        'center_shift_from_marker_pose_mm': float(1000 * np.linalg.norm(
            center_w - np.asarray(marker['cube_center_in_sim_world_candidate_m']))),
        'limitation': 'Face contour corners are approximate; marker disagreement exceeds the 3 px deployment gate. Do not use as calibrated pose or policy start.',
    }
    path = ROOT / 'sac_oct8_outline_pose_check_result.json'
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
