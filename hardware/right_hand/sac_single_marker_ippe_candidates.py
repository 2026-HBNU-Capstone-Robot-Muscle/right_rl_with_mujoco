"""Compute both planar-PnP cube solutions from the saved stage-0 marker image."""

import json
import sys
from pathlib import Path

import cv2
import numpy as np

from hand_registration_geometry import candidate
from sac_registration_from_anchor import detect, marker_pose


ROOT = Path(__file__).resolve().parent
EMPTY = ROOT / 'sequence_vision_20261006_203648_332236/anchor_empty_stage0.png'
PLACED = ROOT / 'sequence_vision_20261008_191152_096085/placement_at_space.png'
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/vision/rubik_center'))
import cube_board
import track_cube


def main():
    with np.load(ROOT / 'camera1_720.npz') as cal:
        K, distortion = cal['K'].copy(), cal['dist'].copy()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    _, old_corners, old_ids, _ = detect(EMPTY, dictionary)
    R_h0, t_h0, _ = marker_pose(old_corners, old_ids, 40, .0282, K, distortion)
    R_a0, t_a0, _ = marker_pose(old_corners, old_ids, 41, .030, K, distortion)
    R_anchor_hand = R_a0.T @ R_h0
    t_anchor_hand = R_a0.T @ (t_h0 - t_a0)
    _, new_corners, new_ids, _ = detect(PLACED, dictionary)
    R_a1, t_a1, _ = marker_pose(new_corners, new_ids, 41, .030, K, distortion)

    recovered = json.loads((ROOT / 'sac_recover_191152_markers_result.json').read_text(encoding='utf-8'))
    found = [row for row in recovered['detections']
             if row['image'] == 'placement_at_space.png' and row['id'] == 5]
    if not found:
        raise RuntimeError('Cube ID5 not recovered from placed frame')
    preference = {'raw': 0, 'clahe': 1, 'equalized': 2, 'gamma_070': 3,
                  'gamma_130': 4, 'blur_sharpen': 5}
    found.sort(key=lambda row: (preference.get(row['variant'], 99),
                                abs(row['scale'] - 1.0)))
    chosen = found[0]
    corners = np.asarray(chosen['corners_px'], dtype=np.float32)
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    points, ids = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                           layout='centers', face_rot=rotations)
    obj = points[int(np.where(ids == 5)[0][0])]
    ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        obj, corners, K, distortion, flags=cv2.SOLVEPNP_IPPE)
    if not ok or len(rvecs) != 2:
        raise RuntimeError(f'Expected two IPPE solutions; got {len(rvecs)}')
    reg = candidate()
    R_world_hand = np.asarray(reg['hand_to_world_candidate'])
    grasp_in_hand = np.asarray(reg['grasp_site_in_marker_m_candidate'])
    output = []
    for index, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        R_c, _ = cv2.Rodrigues(rvec)
        t_c = tvec.ravel()
        reprojected, _ = cv2.projectPoints(obj, rvec, tvec, K, distortion)
        reproj = float(np.linalg.norm(reprojected.reshape(4, 2) - corners, axis=1).mean())
        cube_in_anchor = R_a1.T @ (t_c - t_a1)
        cube_in_hand = R_anchor_hand.T @ (cube_in_anchor - t_anchor_hand)
        R_cube_in_hand = R_anchor_hand.T @ R_a1.T @ R_c
        cube_world = np.array([.11, 0, .03]) + R_world_hand @ (cube_in_hand - grasp_in_hand)
        R_world_cube = R_world_hand @ R_cube_in_hand
        quat = track_cube.mat_to_quat(R_world_cube)
        output.append({'solution': index, 'reprojection_px': reproj,
                       'visible_green_face5_up_dot_candidate': float(
                           (R_world_cube @ np.array([0., -1., 0.]))[2]),
                       'cube_center_in_hand_marker_m': cube_in_hand.tolist(),
                       'cube_center_in_sim_world_candidate_m': cube_world.tolist(),
                       'cube_quat_world_candidate': np.asarray(quat).tolist()})
    centers = [np.asarray(item['cube_center_in_sim_world_candidate_m']) for item in output]
    report = {'status': 'offline_single_marker_ippe_ambiguity',
              'hardware_authorized': False,
              'empty_image': str(EMPTY), 'placed_image': str(PLACED),
              'cube_marker_id': 5, 'detector_variant': chosen['variant'],
              'detector_scale': chosen['scale'], 'solutions': output,
              'center_separation_mm': float(1000 * np.linalg.norm(centers[0] - centers[1])),
              'caveat': 'Both poses fit one marker. Old ID40-to-ID41 and approximate hand-model transforms remain unverified; no hardware use.'}
    (ROOT / 'sac_single_marker_ippe_candidates_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
