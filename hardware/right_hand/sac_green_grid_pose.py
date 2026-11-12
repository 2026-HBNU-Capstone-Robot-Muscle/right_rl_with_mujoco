"""Offline trial: use visible green sticker centers to widen ID5 pose baseline.

This is a diagnostic estimator. It never opens a device and is not wired to
policy control. Rejects glare/occlusion instead of fabricating a 6D pose.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent/'work/Leap_Test/vision/rubik_center'))
import cube_board


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('image', type=Path)
    parser.add_argument('--output-stem', default='sac_green_grid_pose')
    args = parser.parse_args()
    path = args.image if args.image.is_absolute() else ROOT/args.image
    frame = cv.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv.IMREAD_COLOR)
    with np.load(ROOT/'camera1_720.npz') as f:
        K, dist = f['K'].copy(), f['dist'].copy()
    rotations = tuple(json.loads((ROOT/'cube_face_rot_result.json').read_text())['face_rot'])
    pts, _ = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                       layout='centers', face_rot=rotations)
    obj_marker = np.asarray(pts[5], np.float32)
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    # Match the saved-capture detector, including its small-marker recovery.
    detector = MarkerDetector(dictionary, range(6), hand_id=41,
                              required_cube_count=1)
    found, ids, _ = detector.detectMarkers(cv.cvtColor(frame, cv.COLOR_BGR2GRAY))
    if ids is None or 5 not in ids:
        raise RuntimeError('ID5 not visible')
    marker = np.asarray(found[list(ids.ravel()).index(5)], np.float32).reshape(4, 2)
    marker_center = marker.mean(axis=0)
    h = 54.5/2000
    pitch = 54.5/3000
    n, u, v = cube_board.FACES[5]
    grid = np.asarray([h*n + x*u + y*v for y in (pitch, 0, -pitch)
                       for x in (-pitch, 0, pitch)], np.float32)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, np.array([35, 55, 35]), np.array([100, 255, 255]))
    count, labels, stats, centers = cv.connectedComponentsWithStats(mask)
    centroids = []
    for label in range(1, count):
        x, y, w, ht, area = map(int, stats[label])
        if not 500 <= area <= 15000 or not 20 <= w <= 120 or not 20 <= ht <= 120:
            continue
        if np.linalg.norm(centers[label]-marker_center) > 185:
            continue
        centroids.append(np.asarray(centers[label], float))
    if len(centroids) < 4:
        raise RuntimeError(f'Only {len(centroids)} usable green stickers')
    centroids = np.asarray(centroids, np.float32)
    count_pose, rvecs, tvecs, _ = cv.solvePnPGeneric(
        obj_marker, marker, K, dist, flags=cv.SOLVEPNP_IPPE)
    if not count_pose:
        raise RuntimeError('ID5 planar pose failed')
    hypotheses = []
    for rv, tv in zip(rvecs, tvecs):
        projected, _ = cv.projectPoints(grid, rv, tv, K, dist)
        projected = projected.reshape(9, 2)
        costs = np.linalg.norm(projected[:, None, :] - centroids[None, :, :], axis=2)
        # Nine grid cells at most: deterministic nearest unique assignment.
        accepted = []
        used_rows, used_cols = set(), set()
        for row, col in sorted(np.ndindex(costs.shape), key=lambda pair: costs[pair]):
            if costs[row, col] > 45:
                break
            if row not in used_rows and col not in used_cols:
                accepted.append((int(row), int(col), float(costs[row, col])))
                used_rows.add(row)
                used_cols.add(col)
        if len(accepted) < 4:
            continue
        obj = np.concatenate((obj_marker, grid[[r for r, _, _ in accepted]]))
        img = np.concatenate((marker, centroids[[c for _, c, _ in accepted]]))
        ok, refined_rv, refined_tv = cv.solvePnP(obj, img, K, dist, rv.copy(), tv.copy(), True)
        if not ok:
            continue
        image_fit, _ = cv.projectPoints(obj, refined_rv, refined_tv, K, dist)
        errors = np.linalg.norm(image_fit.reshape(-1, 2)-img, axis=1)
        hypotheses.append(dict(rvec=refined_rv, tvec=refined_tv, matches=accepted,
                               reprojection_px=float(errors.mean()),
                               marker_error_px=float(errors[:4].mean()),
                               sticker_error_px=float(errors[4:].mean())))
    if not hypotheses:
        raise RuntimeError('No ID5 + green-grid pose found')
    winner = min(hypotheses, key=lambda x: (x['reprojection_px'], -len(x['matches'])))
    report = {'status': 'offline_experimental_green_grid_fit', 'hardware_authorized': False,
              'image': str(path), 'green_blobs': len(centroids),
              'matched_stickers': len(winner['matches']),
              'mean_reprojection_px': winner['reprojection_px'],
              'marker_reprojection_px': winner['marker_error_px'],
              'sticker_reprojection_px': winner['sticker_error_px'],
              'cube_center_camera_m': winner['tvec'].ravel().tolist(),
              'cube_rotation_camera': cv.Rodrigues(winner['rvec'])[0].tolist(),
              'fit_accepted': bool(winner['reprojection_px'] <= 3
                                   and winner['marker_error_px'] <= 3
                                   and winner['sticker_error_px'] <= 4),
              'limitation': 'Glare, partial sticker blobs and imperfect sticker-center geometry can bias depth; no control use.'}
    (ROOT/f'{args.output_stem}_result.json').write_text(json.dumps(report, indent=2)+'\n')
    overlay = frame.copy()
    for row, col, _ in winner['matches']:
        p = tuple(np.round(centroids[col]).astype(int))
        cv.circle(overlay, p, 5, (255, 0, 255), 2)
        cv.putText(overlay, str(row), (p[0]+5, p[1]), cv.FONT_HERSHEY_SIMPLEX,
                   .5, (0, 0, 255), 1)
    (ROOT/f'{args.output_stem}_overlay.png').write_bytes(cv.imencode('.png', overlay)[1].tobytes())
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
