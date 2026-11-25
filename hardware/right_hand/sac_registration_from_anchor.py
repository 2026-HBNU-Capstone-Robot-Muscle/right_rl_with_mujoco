"""Offline two-frame cube registration using a rigidly mounted ID41 marker.

Empty stage-0 frame: ID40 + ID41. Placed frame: ID41 + one or more cube IDs.
The anchor is assumed rigid to the palm mount between the two images.
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from hand_registration_geometry import candidate
from marker_detector import MarkerDetector

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'vision' / 'rubik_center'))
import cube_board
import track_cube


def marker_pose(corners, ids, target, side_m, intrinsic, distortion):
    matches = [np.asarray(c).reshape(4, 2) for c, i in zip(corners, ids) if int(i) == target]
    if len(matches) != 1:
        raise RuntimeError(f'Expected exactly one ID{target}; found {len(matches)}')
    s = side_m / 2
    obj = np.asarray([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, matches[0], intrinsic, distortion)
    if not ok:
        raise RuntimeError(f'ID{target} pose solve failed')
    projected, _ = cv2.projectPoints(obj, rvec, tvec, intrinsic, distortion)
    err = float(np.linalg.norm(projected.reshape(4, 2) - matches[0], axis=1).mean())
    return cv2.Rodrigues(rvec)[0], tvec.reshape(3), err


def detect(path, dictionary):
    frame = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f'Cannot decode {path}')
    detector = MarkerDetector(dictionary, range(41), hand_id=41,
                              required_cube_count=1)
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    decoded = [] if ids is None else [int(i) for i in ids.ravel()]
    return frame, corners, decoded, detector


def estimate(folder, empty_image='anchor_empty_stage0.png', placed_image='placement_at_space.png'):
    empty_path = folder / empty_image
    placed_path = folder / placed_image
    with np.load(ROOT / 'camera1_720.npz') as calibration:
        intrinsic, distortion = calibration['K'].copy(), calibration['dist'].copy()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    empty_frame, empty_corners, empty_ids, _ = detect(empty_path, dictionary)
    placed_frame, placed_corners, placed_ids, placed_detector = detect(placed_path, dictionary)
    if 40 not in empty_ids or 41 not in empty_ids or 41 not in placed_ids:
        raise RuntimeError(f'Anchor pair incomplete: empty={empty_ids}, placed={placed_ids}')
    if sum(i < 40 for i in placed_ids) < 1:
        raise RuntimeError(f'Need at least one cube marker in placed frame: {placed_ids}')
    R_h0, t_h0, hand_err = marker_pose(empty_corners, empty_ids, 40, .0282, intrinsic, distortion)
    R_a0, t_a0, anchor_empty_err = marker_pose(empty_corners, empty_ids, 41, .030, intrinsic, distortion)
    R_a1, t_a1, anchor_placed_err = marker_pose(placed_corners, placed_ids, 41, .030, intrinsic, distortion)
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    points, ids = cube_board.object_points(cube_mm=54.5, marker_mm=10.6, layout='centers', face_rot=rotations)
    board = cv2.aruco.Board(points, dictionary, ids)
    cube = track_cube.step(placed_frame, board, placed_detector, dictionary, intrinsic, distortion)
    if cube is None or 't_cube' not in cube or cube.get('n_cube_markers', 0) < 1:
        raise RuntimeError('Cube board solve failed in placed frame')
    R_c1, t_c1 = np.asarray(cube['R_cube']), np.asarray(cube['t_cube'])
    cube_in_anchor = R_a1.T @ (t_c1 - t_a1)
    R_cube_in_anchor = R_a1.T @ R_c1
    cube_in_empty_camera = t_a0 + R_a0 @ cube_in_anchor
    R_cube_in_empty_camera = R_a0 @ R_cube_in_anchor
    cube_in_hand = R_h0.T @ (cube_in_empty_camera - t_h0)
    R_cube_in_hand = R_h0.T @ R_cube_in_empty_camera
    registration = candidate()
    R_world_hand = np.asarray(registration['hand_to_world_candidate'])
    grasp_in_hand = np.asarray(registration['grasp_site_in_marker_m_candidate'])
    cube_world = np.array([.11, 0, .03]) + R_world_hand @ (cube_in_hand - grasp_in_hand)
    cube_quat = track_cube.mat_to_quat(R_world_hand @ R_cube_in_hand)
    report = {
        'status': ('anchor_pair_candidate' if cube['n_cube_markers'] >= 2
                   else 'anchor_single_marker_candidate'),
        'hardware_authorized': False,
        'source_folder': str(folder),
        'empty_image': empty_image, 'placed_image': placed_image,
        'empty_ids': empty_ids, 'placed_ids': placed_ids,
        'hand_reprojection_px': hand_err,
        'anchor_empty_reprojection_px': anchor_empty_err,
        'anchor_placed_reprojection_px': anchor_placed_err,
        'cube_reprojection_px': float(cube['reproj']),
        'cube_marker_count': int(cube['n_cube_markers']),
        'cube_center_in_hand_marker_m': cube_in_hand.tolist(),
        'cube_center_in_sim_world_candidate_m': cube_world.tolist(),
        'cube_quat_world_candidate': np.asarray(cube_quat).tolist(),
        'limitation': ('ID41 must be rigid to the hand base. A single small cube marker has an ambiguous/noisy pose; '
                       'marker-to-model transform and MuJoCo contact must be validated before any policy use.'),
    }
    # The recorder's first generic preflight example can include a hand, a
    # lighting adjustment, or a moving reference. Use the same qualified
    # ID40+ID41 frame that the live rigidity check used.
    preflight_path = folder / 'anchor_empty_preflight.png'
    stage0_path = folder / 'anchor_empty_stage0.png'
    if preflight_path.exists() and stage0_path.exists():
        pair = []
        stage0_hand_corners = None
        for path in (preflight_path, stage0_path):
            _, pair_corners, pair_ids, _ = detect(path, dictionary)
            R_h, t_h, _ = marker_pose(pair_corners, pair_ids, 40, .0282, intrinsic, distortion)
            R_a, t_a, _ = marker_pose(pair_corners, pair_ids, 41, .030, intrinsic, distortion)
            pair.append((R_a.T @ R_h, R_a.T @ (t_h - t_a), R_a, t_a))
            if path == stage0_path:
                stage0_hand_corners = next(np.asarray(c).reshape(4, 2)
                                           for c, i in zip(pair_corners, pair_ids) if i == 40)
        shift_mm = float(np.linalg.norm(pair[1][1] - pair[0][1]) * 1000)
        delta_R = pair[0][0].T @ pair[1][0]
        angle_deg = float(np.degrees(np.arccos(np.clip((np.trace(delta_R)-1)/2, -1, 1))))
        predicted_R_hand = pair[1][2] @ pair[0][0]
        predicted_t_hand = pair[1][3] + pair[1][2] @ pair[0][1]
        rvec, _ = cv2.Rodrigues(predicted_R_hand)
        s = .0282 / 2
        marker40_obj = np.asarray([[-s, s, 0], [s, s, 0],
                                   [s, -s, 0], [-s, -s, 0]], np.float32)
        projected, _ = cv2.projectPoints(marker40_obj, rvec, predicted_t_hand,
                                         intrinsic, distortion)
        pixel_error = float(np.linalg.norm(projected.reshape(4, 2) - stage0_hand_corners,
                                            axis=1).mean())
        report['empty_marker_relation_shift_mm'] = shift_mm
        report['empty_marker_relation_rotation_deg'] = angle_deg
        report['empty_marker_rigid_projection_error_px'] = pixel_error
        if shift_mm > 10 or angle_deg > 10 or pixel_error > 10:
            report['status'] = 'anchor_hand_relation_inconsistent'
            report['limitation'] = ('ID40-to-ID41 transform changed between empty frames; '
                                    'the wrist marker or planar pose estimate is not a valid rigid reference. '
                                    'Do not use this cube pose for SAC deployment.')
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', type=Path)
    parser.add_argument('--empty-image', default='anchor_empty_stage0.png',
                        choices=('anchor_empty_stage0.png', 'preflight_not_ready.png'))
    parser.add_argument('--placed-image', default='placement_at_space.png',
                        choices=('placement_at_space.png', 'anchor_cube_candidate.png'))
    args = parser.parse_args()
    if args.folder is None:
        run = json.loads((ROOT / 'leap_anchor_capture_result.json').read_text(encoding='utf-8'))
        if run.get('status') != 'placement_capture_complete' or not run.get('all_torque_off_verified'):
            raise RuntimeError('Anchor capture did not complete with verified torque OFF')
        folder = Path(run['vision_recording']['folder'])
    else:
        folder = args.folder
    report = estimate(folder, empty_image=args.empty_image, placed_image=args.placed_image)
    if args.empty_image != 'anchor_empty_stage0.png':
        report['status'] = 'support_photo_diagnostic'
        report['diagnostic_only'] = True
        report['limitation'] += (' The empty hand/reference pair precedes stage-0 movement; '
                                 'rigidity was not verified after movement in this capture.')
    if args.placed_image == 'anchor_cube_candidate.png':
        report['diagnostic_only'] = True
        report['limitation'] += ' Candidate may be stale or never confirmed by SPACE.'
    output = ROOT / ('sac_registration_support_photo_diagnostic.json'
                     if args.empty_image != 'anchor_empty_stage0.png' else
                     'sac_registration_anchor_candidate_diagnostic.json'
                     if args.placed_image == 'anchor_cube_candidate.png' else
                     'sac_registration_anchor_result.json')
    output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
