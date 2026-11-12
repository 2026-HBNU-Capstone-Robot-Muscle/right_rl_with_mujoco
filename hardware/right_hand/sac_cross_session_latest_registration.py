"""Diagnostic cross-session ID41 bridge from saved images; never opens devices."""
import json
import argparse
from pathlib import Path
import sys

import cv2
import numpy as np

from hand_registration_geometry import candidate
from sac_registration_from_anchor import detect, marker_pose

ROOT = Path(__file__).resolve().parent
OLD = ROOT / 'sequence_vision_20261006_203648_332236/anchor_empty_stage0.png'
NEW = ROOT / 'sequence_vision_20261008_184238_359328/placement_at_space.png'
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/vision/rubik_center'))
import cube_board
import track_cube
from marker_detector import MarkerDetector


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--new-image', default=str(NEW))
    parser.add_argument('--output-file', default='sac_cross_session_latest_registration_result.json')
    args = parser.parse_args()
    new_image = Path(args.new_image)
    if not new_image.is_absolute():
        new_image = ROOT / new_image
    with np.load(ROOT / 'camera1_720.npz') as cal:
        K, distortion = cal['K'].copy(), cal['dist'].copy()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    _, old_corners, old_ids, _ = detect(OLD, dictionary)
    R_h0, t_h0, err_h0 = marker_pose(old_corners, old_ids, 40, .0282, K, distortion)
    R_a0, t_a0, err_a0 = marker_pose(old_corners, old_ids, 41, .030, K, distortion)
    R_anchor_hand = R_a0.T @ R_h0
    t_anchor_hand = R_a0.T @ (t_h0 - t_a0)

    frame = cv2.imdecode(np.frombuffer(new_image.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f'Cannot read {new_image}')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    marker_corners, marker_ids, _ = aruco.detectMarkers(gray)
    marker_ids = [] if marker_ids is None else [int(i) for i in marker_ids.ravel()]
    R_a1, t_a1, err_a1 = marker_pose(marker_corners, marker_ids, 41, .030, K, distortion)

    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    points, ids = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                           layout='centers', face_rot=rotations)
    board = cv2.aruco.Board(points, dictionary, ids)
    cube_detector = MarkerDetector(dictionary, ids)
    gray_variants = {
        'raw': frame,
        'clahe': cv2.cvtColor(cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray),
                              cv2.COLOR_GRAY2BGR),
        'equalized': cv2.cvtColor(cv2.equalizeHist(gray), cv2.COLOR_GRAY2BGR),
        'gamma_070': cv2.cvtColor(np.uint8(255 * (gray / 255.0) ** .7), cv2.COLOR_GRAY2BGR),
        'gamma_130': cv2.cvtColor(np.uint8(255 * (gray / 255.0) ** 1.3), cv2.COLOR_GRAY2BGR),
        'sharpened': cv2.cvtColor(cv2.addWeighted(
            gray, 2.0, cv2.GaussianBlur(gray, (0, 0), 1.5), -1.0, 0), cv2.COLOR_GRAY2BGR),
    }
    cube = None
    cube_variant = None
    for variant, candidate_frame in gray_variants.items():
        found = track_cube.step(candidate_frame, board, cube_detector, dictionary, K, distortion)
        if found is not None and 't_cube' in found:
            cube, cube_variant = found, variant
            break
    if cube is None or 't_cube' not in cube:
        raise RuntimeError('No cube pose in new frame')
    R_c1, t_c1 = np.asarray(cube['R_cube']), np.asarray(cube['t_cube'])
    cube_in_anchor = R_a1.T @ (t_c1 - t_a1)
    R_cube_in_anchor = R_a1.T @ R_c1
    cube_in_hand = R_anchor_hand.T @ (cube_in_anchor - t_anchor_hand)
    R_cube_in_hand = R_anchor_hand.T @ R_cube_in_anchor
    reg = candidate()
    R_world_hand = np.asarray(reg['hand_to_world_candidate'])
    grasp_in_hand = np.asarray(reg['grasp_site_in_marker_m_candidate'])
    cube_world = np.array([.11, 0, .03]) + R_world_hand @ (cube_in_hand - grasp_in_hand)
    cube_quat = track_cube.mat_to_quat(R_world_hand @ R_cube_in_hand)
    old_result = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    old_center = np.asarray(old_result['cube_center_in_hand_marker_m'])
    report = {
        'status': 'cross_session_single_marker_diagnostic',
        'hardware_authorized': False,
        'reference_empty_image': str(OLD), 'new_placed_image': str(new_image),
        'new_marker_ids': marker_ids,
        'old_hand_reprojection_px': err_h0,
        'old_anchor_reprojection_px': err_a0,
        'new_anchor_reprojection_px': err_a1,
        'cube_reprojection_px': float(cube['reproj']),
        'cube_marker_count': int(cube['n_cube_markers']),
        'cube_image_variant': cube_variant,
        'cube_center_in_anchor_m': cube_in_anchor.tolist(),
        'cube_center_in_hand_marker_m': cube_in_hand.tolist(),
        'cube_center_in_sim_world_candidate_m': cube_world.tolist(),
        'cube_quat_world_candidate': np.asarray(cube_quat).tolist(),
        'center_shift_from_old_placement_mm': float(1000 * np.linalg.norm(cube_in_hand - old_center)),
        'limitation': 'Uses old ID40-to-ID41 transform across sessions. New ID40 was obscured; new cube has only one marker. Neither mount rigidity nor cube orientation/contact is verified.',
    }
    path = ROOT / args.output_file
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
