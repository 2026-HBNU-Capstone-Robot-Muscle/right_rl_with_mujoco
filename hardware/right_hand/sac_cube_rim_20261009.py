"""Diagnostic cube pose from same-run ID5 and approximate visible top rim."""

import itertools
import argparse
import json
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

from sac_registration_from_anchor import detect


ROOT = Path(__file__).resolve().parent
FRAME = ROOT / 'sequence_vision_20261009_221628_356082/placement_at_space.png'
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/vision/rubik_center'))
import cube_board


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frame', type=Path, default=FRAME)
    parser.add_argument('--rim', nargs=8, type=float,
                        default=[426, 113, 690, 108, 716, 275, 435, 281])
    parser.add_argument('--output-stem', default='sac_cube_rim_20261009')
    args = parser.parse_args()
    with np.load(ROOT / 'camera1_720.npz') as f:
        K, dist = f['K'].copy(), f['dist'].copy()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    frame, corners, ids, _ = detect(args.frame, dictionary)
    inner_img = np.asarray(next(c for c, i in zip(corners, ids) if i == 5)).reshape(4, 2).astype(np.float32)
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    points, _ = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                          layout='centers', face_rot=rotations)
    inner_obj = np.asarray(points[5], np.float32)
    face_n = np.asarray(cube_board.FACES[5][0], dtype=float)
    face_center = face_n * (54.5 / 2000)
    outer_obj = (face_center + (inner_obj - face_center) * (54.5 / 10.6)).astype(np.float32)
    # Approximate visible top plastic corners, clockwise from top-left.
    rim = np.asarray(args.rim, np.float32).reshape(4, 2)
    ok, initial_rv, initial_tv = cv.solvePnP(inner_obj, inner_img, K, dist)
    if not ok:
        raise RuntimeError('ID5 solve failed')
    best = None
    for perm in itertools.permutations(range(4)):
        outer_img = rim[list(perm)]
        obj = np.concatenate((inner_obj, outer_obj))
        img = np.concatenate((inner_img, outer_img))
        ok, rv, tv = cv.solvePnP(obj, img, K, dist, initial_rv.copy(), initial_tv.copy(), True)
        if not ok:
            continue
        proj, _ = cv.projectPoints(obj, rv, tv, K, dist)
        errs = np.linalg.norm(proj.reshape(8, 2) - img, axis=1)
        score = float(errs.mean())
        if best is None or score < best['score_px']:
            best = {'perm': perm, 'score_px': score, 'inner_error_px': float(errs[:4].mean()),
                    'rim_error_px': float(errs[4:].mean()), 'rv': rv.copy(), 'tv': tv.copy(),
                    'projected': proj.reshape(8, 2).copy()}
    if best is None:
        raise RuntimeError('Combined PnP failed')
    report = {'status': 'offline_same_run_rim_diagnostic', 'hardware_authorized': False,
              'source': str(args.frame), 'marker_corners_px': inner_img.tolist(),
              'hand_read_outer_corners_px': rim.tolist(),
              'rim_assignment': list(best['perm']), 'joint_reprojection_px': best['score_px'],
              'marker_reprojection_px': best['inner_error_px'], 'rim_reprojection_px': best['rim_error_px'],
              'cube_center_camera_m': best['tv'].reshape(3).tolist(),
              'cube_rotation_camera': cv.Rodrigues(best['rv'])[0].tolist(),
              'id5_only_center_camera_m': initial_tv.reshape(3).tolist(),
              'center_shift_from_id5_only_mm': (1000 * (best['tv'] - initial_tv).reshape(3)).tolist(),
              'limitation': 'Outer rim manually estimated from rounded/high-glare plastic edges; use as cross-check only.'}
    (ROOT / f'{args.output_stem}_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    cv.polylines(frame, [np.int32(rim)], True, (0, 255, 255), 2)
    cv.polylines(frame, [np.int32(best['projected'][4:])], True, (0, 0, 255), 2)
    cv.polylines(frame, [np.int32(best['projected'][:4])], True, (255, 0, 255), 2)
    (ROOT / f'{args.output_stem}_overlay.png').write_bytes(cv.imencode('.png', frame)[1].tobytes())
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
