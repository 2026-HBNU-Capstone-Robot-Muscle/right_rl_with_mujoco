"""Offline check whether assumed cube ArUco print size explains depth conflict."""

import json
import argparse
import itertools
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
    parser.add_argument('--output-file', default='sac_marker_size_sensitivity_20261009_result.json')
    args = parser.parse_args()
    with np.load(ROOT / 'camera1_720.npz') as f:
        K, dist = f['K'].copy(), f['dist'].copy()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    _, corners, ids, _ = detect(args.frame, dictionary)
    inner_img = np.asarray(next(c for c, i in zip(corners, ids) if i == 5)).reshape(4, 2).astype(np.float32)
    outer = np.asarray(args.rim, np.float32).reshape(4, 2)
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    face_n = np.asarray(cube_board.FACES[5][0], dtype=float)
    face_center = face_n * (54.5 / 2000)
    rows = []
    for marker_mm in np.arange(8.5, 12.51, .25):
        points, _ = cube_board.object_points(cube_mm=54.5, marker_mm=float(marker_mm),
                                              layout='centers', face_rot=rotations)
        inner_obj = np.asarray(points[5], np.float32)
        outer_obj = (face_center + (inner_obj - face_center) * (54.5 / marker_mm)).astype(np.float32)
        obj = np.concatenate((inner_obj, outer_obj))
        best = None
        for perm in itertools.permutations(range(4)):
            img = np.concatenate((inner_img, outer[list(perm)]))
            ok, rv, tv = cv.solvePnP(obj, img, K, dist)
            if not ok:
                continue
            proj, _ = cv.projectPoints(obj, rv, tv, K, dist)
            err = np.linalg.norm(proj.reshape(8, 2) - img, axis=1)
            row = {'marker_side_mm': round(float(marker_mm), 2),
                   'rim_assignment': list(perm),
                   'joint_error_px': float(err.mean()),
                   'marker_error_px': float(err[:4].mean()),
                   'rim_error_px': float(err[4:].mean()),
                   'center_camera_z_m': float(tv[2, 0])}
            if best is None or row['joint_error_px'] < best['joint_error_px']:
                best = row
        if best is not None:
            rows.append(best)
    report = {'status': 'offline_marker_size_sensitivity', 'hardware_authorized': False,
              'source': str(args.frame), 'hand_read_outer_corners_px': outer.tolist(),
              'assumed_marker_mm': 10.6, 'best': min(rows, key=lambda r: r['joint_error_px']),
              'rows': rows,
              'caveat': 'Rim corners are hand-read; this is not a physical sticker measurement.'}
    (ROOT / args.output_file).write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'best': report['best'], 'at_10_5': next(r for r in rows if r['marker_side_mm'] == 10.5),
                      'at_10_75': next(r for r in rows if r['marker_side_mm'] == 10.75)}, indent=2))


if __name__ == '__main__':
    main()
