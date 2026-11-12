"""Experimental one-face cube pose from ArUco marker + same-color stickers.

Pure image analysis. Never used for motor control without independent error
and occlusion validation. Assumes an UNSCRAMBLED 3x3 color grid; turning Rubik
layers invalidates the same-color sticker geometry, so this cannot serve as a
general pose tracker for a scrambled cube.
"""

import json
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'vision' / 'rubik_center'))
import cube_board

COLOR_RANGES = {
    1: ((18, 40, 65, 55),),       # yellow
    2: ((0, 12, 80, 45), (170, 179, 80, 45)),  # red
    3: ((5, 25, 80, 45),),        # orange
    4: ((90, 140, 55, 35),),      # blue
    5: ((35, 100, 55, 35),),      # green
}


def _mask(frame, marker_id):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo_h, hi_h, min_s, min_v in COLOR_RANGES[marker_id]:
        mask |= cv.inRange(hsv, np.array([lo_h, min_s, min_v]),
                           np.array([hi_h, 255, 255]))
    return mask


def fit(frame, marker_id, K, dist, rotations):
    if marker_id not in COLOR_RANGES:
        raise ValueError(f'No color mask for cube marker ID{marker_id}')
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    detector = cv.aruco.ArucoDetector(dictionary, cv.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(cv.cvtColor(frame, cv.COLOR_BGR2GRAY))
    if ids is None or marker_id not in ids:
        raise RuntimeError(f'ID{marker_id} not visible')
    marker = np.asarray(corners[list(ids.ravel()).index(marker_id)], np.float32).reshape(4, 2)
    marker_center = marker.mean(axis=0)
    marker_side_px = float(np.linalg.norm(marker[1] - marker[0]))
    marker_area = max(1., float(cv.contourArea(marker)))
    count, labels, stats, centers = cv.connectedComponentsWithStats(_mask(frame, marker_id))
    blobs = []
    for label in range(1, count):
        x, y, width, height, area = map(int, stats[label])
        if not .4 * marker_area <= area <= 20 * marker_area:
            continue
        if not .35 * marker_side_px <= width <= 4.0 * marker_side_px:
            continue
        if not .35 * marker_side_px <= height <= 4.0 * marker_side_px:
            continue
        if np.linalg.norm(centers[label] - marker_center) > 6.0 * marker_side_px:
            continue
        blobs.append(centers[label])
    if len(blobs) < 4:
        raise RuntimeError(f'ID{marker_id}: only {len(blobs)} usable color blobs')
    blobs = np.asarray(blobs, np.float32)
    points, _ = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                          layout='centers', face_rot=rotations)
    obj_marker = np.asarray(points[marker_id], np.float32)
    half = 54.5 / 2000
    pitch = 54.5 / 3000
    normal, u, v = cube_board.FACES[marker_id]
    grid = np.asarray([half * normal + x * u + y * v
                       for y in (pitch, 0, -pitch)
                       for x in (-pitch, 0, pitch)], np.float32)
    keep = [i for i in range(9) if i != 4]
    grid = grid[keep]
    count_pose, rvecs, tvecs, _ = cv.solvePnPGeneric(
        obj_marker, marker, K, dist, flags=cv.SOLVEPNP_IPPE)
    if not count_pose:
        raise RuntimeError('Marker planar pose failed')
    hypotheses = []
    for rv, tv in zip(rvecs, tvecs):
        projected, _ = cv.projectPoints(grid, rv, tv, K, dist)
        costs = np.linalg.norm(projected.reshape(-1, 2)[:, None, :] -
                               blobs[None, :, :], axis=2)
        matches = []
        used_rows, used_cols = set(), set()
        for row, col in sorted(np.ndindex(costs.shape), key=lambda pair: costs[pair]):
            if costs[row, col] > max(45., 1.5 * marker_side_px):
                break
            if row not in used_rows and col not in used_cols:
                matches.append((row, col))
                used_rows.add(row)
                used_cols.add(col)
        if len(matches) < 4:
            continue
        obj = np.concatenate((obj_marker, grid[[r for r, _ in matches]]))
        img = np.concatenate((marker, blobs[[c for _, c in matches]]))
        ok, refined_rv, refined_tv = cv.solvePnP(obj, img, K, dist,
                                                   rv.copy(), tv.copy(), True)
        if not ok:
            continue
        reproj, _ = cv.projectPoints(obj, refined_rv, refined_tv, K, dist)
        errors = np.linalg.norm(reproj.reshape(-1, 2) - img, axis=1)
        hypotheses.append({'rvec': refined_rv, 'tvec': refined_tv,
                           'matches': matches, 'mean_error_px': float(errors.mean()),
                           'marker_error_px': float(errors[:4].mean()),
                           'sticker_error_px': float(errors[4:].mean())})
    if not hypotheses:
        raise RuntimeError('No marker + same-color grid fit')
    winner = min(hypotheses, key=lambda row: (row['mean_error_px'],
                                             -len(row['matches'])))
    if (winner['mean_error_px'] > 3.0 or winner['marker_error_px'] > 3.0 or
            winner['sticker_error_px'] > 4.0):
        raise RuntimeError('Marker + sticker reprojection exceeds quality limits')
    return {'marker_id': marker_id, 'cube_center_camera_m': winner['tvec'].ravel().tolist(),
            'cube_rotation_camera': cv.Rodrigues(winner['rvec'])[0].tolist(),
            'matched_stickers': len(winner['matches']),
            'color_blobs': len(blobs),
            'mean_reprojection_px': winner['mean_error_px'],
            'marker_reprojection_px': winner['marker_error_px'],
            'sticker_reprojection_px': winner['sticker_error_px']}


def fit_image(image_path, marker_id):
    path = Path(image_path)
    frame = cv.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv.IMREAD_COLOR)
    with np.load(ROOT / 'camera1_720.npz') as archive:
        K, dist = archive['K'].copy(), archive['dist'].copy()
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text())['face_rot'])
    return fit(frame, marker_id, K, dist, rotations)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('image', type=Path)
    parser.add_argument('marker_id', type=int)
    args = parser.parse_args()
    print(json.dumps(fit_image(args.image, args.marker_id), indent=2))
