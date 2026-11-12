"""Estimate green sticker envelope from archived two-marker cube frames."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from sac_registration_from_anchor import detect


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'vision' / 'rubik_center'))
import cube_board
import track_cube


def estimate(path, board, dictionary, K, dist, rotations):
    frame, corners, ids, detector = detect(path, dictionary)
    cube = track_cube.step(frame, board, detector, dictionary, K, dist)
    if cube is None or cube.get('n_cube_markers', 0) < 2 or 5 not in ids:
        raise RuntimeError(f'Need two cube markers including ID5 in {path}: {ids}')
    marker5 = next(np.asarray(c, np.float32).reshape(4, 2)
                   for c, marker_id in zip(corners, ids) if marker_id == 5)
    p = marker5.mean(axis=0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([32, 55, 25]), np.array([90, 255, 255]))
    region = np.zeros(mask.shape, np.uint8)
    region[max(0, int(p[1]-180)):min(frame.shape[0], int(p[1]+180)),
           max(0, int(p[0]-180)):min(frame.shape[1], int(p[0]+180))] = 255
    mask &= region
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    comps = [(i, int(stats[i, cv2.CC_STAT_AREA])) for i in range(1, count)
             if stats[i, cv2.CC_STAT_AREA] >= 80]
    comps.sort(key=lambda item: item[1], reverse=True)
    selected = np.zeros_like(mask)
    for i, _ in comps[:9]:
        selected[labels == i] = 255
    ys, xs = np.nonzero(selected)
    if len(xs) < 100:
        raise RuntimeError('No green sticker group')
    hull = cv2.convexHull(np.column_stack((xs, ys)).astype(np.float32)).reshape(-1, 2)
    polygon = cv2.approxPolyDP(hull, .055 * cv2.arcLength(hull, True), True).reshape(-1, 2)
    if len(polygon) != 4:
        raise RuntimeError(f'Green outline has {len(polygon)} vertices')
    face_n, face_u0, face_v0 = cube_board.FACES[5]
    face_u, face_v = cube_board._rot_uv(face_u0, face_v0, rotations[5])
    R, t = np.asarray(cube['R_cube']), np.asarray(cube['t_cube']).reshape(3)
    rvec, _ = cv2.Rodrigues(R)
    scores = []
    for sticker_extent_mm in np.arange(46., 54.6, .25):
        s = sticker_extent_mm / 2000
        face_center = 54.5 / 2000 * face_n
        obj = np.asarray([face_center-s*face_u+s*face_v, face_center+s*face_u+s*face_v,
                          face_center+s*face_u-s*face_v, face_center-s*face_u-s*face_v], np.float32)
        projected, _ = cv2.projectPoints(obj, rvec, t, K, dist)
        projected = projected.reshape(4, 2)
        error = min(float(np.linalg.norm(projected - np.roll(base, -offset, axis=0), axis=1).mean())
                    for base in (polygon, polygon[::-1]) for offset in range(4))
        scores.append((error, sticker_extent_mm))
    best_error, best_extent = min(scores)
    return {'source': str(path), 'cube_ids': [i for i in ids if i < 40],
            'cube_reprojection_px': float(cube['reproj']),
            'green_polygon_px': polygon.tolist(),
            'best_sticker_extent_mm': float(best_extent),
            'best_outline_error_px': best_error,
            'green_component_areas_px': [area for _, area in comps[:9]]}


def main():
    with np.load(ROOT / 'camera1_720.npz') as camera:
        K, dist = camera['K'].copy(), camera['dist'].copy()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    points, ids = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                           layout='centers', face_rot=rotations)
    board = cv2.aruco.Board(points, dictionary, ids)
    folder = ROOT / 'vision_sweep_20261006_011011_994373'
    rows = [estimate(folder / name, board, dictionary, K, dist, rotations)
            for name in ('00-10s_eligible.png', '10-20s_eligible.png')]
    output = {'rows': rows, 'hardware_authorized': False,
              'caveat': 'Colored sticker segmentation and marker-board pose are approximate.'}
    (ROOT / 'sac_green_sticker_extent_from_archive_result.json').write_text(
        json.dumps(output, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
