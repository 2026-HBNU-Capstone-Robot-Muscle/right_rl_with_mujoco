"""Cross-check latest single-marker cube pose against its broad green face outline."""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from sac_registration_from_anchor import detect


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'vision' / 'rubik_center'))
import cube_board

FRAME = ROOT / 'sequence_vision_20261007_000611_625058' / 'placement_at_space.png'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frame', type=Path, default=FRAME)
    parser.add_argument('--output-stem', default='sac_cube_top_outline_latest')
    parser.add_argument('--green-hue-min', type=int, default=32)
    parser.add_argument('--recovery-json', type=Path, default=None,
                        help='Saved offline marker corners when the raw detector misses ID5')
    args = parser.parse_args()
    frame, corners, ids, _ = detect(args.frame, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    marker5 = next((np.asarray(c, np.float32).reshape(4, 2)
                    for c, marker_id in zip(corners, ids) if marker_id == 5), None)
    if marker5 is None and args.recovery_json is not None:
        recovery = json.loads(args.recovery_json.read_text(encoding='utf-8'))
        recovered = [row for row in recovery['detections']
                     if row['image'] == args.frame.name and row['id'] == 5]
        recovered.sort(key=lambda row: (row['variant'] != 'clahe', abs(row['scale'] - 1)))
        if recovered:
            marker5 = np.asarray(recovered[0]['corners_px'], np.float32).reshape(4, 2)
    if marker5 is None:
        raise RuntimeError('Cube marker ID5 not found in image or recovery JSON')
    center_px = marker5.mean(axis=0)
    x0, x1 = int(center_px[0]-190), int(center_px[0]+190)
    y0, y1 = int(center_px[1]-150), int(center_px[1]+150)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([args.green_hue_min, 55, 25]), np.array([90, 255, 255]))
    region = np.zeros(mask.shape, np.uint8)
    region[max(0, y0):min(frame.shape[0], y1), max(0, x0):min(frame.shape[1], x1)] = 255
    mask &= region
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    comps = [(i, int(stats[i, cv2.CC_STAT_AREA])) for i in range(1, count)
             if stats[i, cv2.CC_STAT_AREA] >= 100]
    comps.sort(key=lambda item: item[1], reverse=True)
    selected = np.zeros_like(mask)
    for i, _ in comps[:9]:
        selected[labels == i] = 255
    ys, xs = np.nonzero(selected)
    if len(xs) < 100:
        raise RuntimeError('No broad green face found')
    hull = cv2.convexHull(np.column_stack((xs, ys)).astype(np.float32)).reshape(-1, 2)
    polygon = None
    for tolerance in (.055, .06, .065, .07, .075, .08, .09):
        proposed = cv2.approxPolyDP(hull, tolerance * cv2.arcLength(hull, True), True).reshape(-1, 2)
        if len(proposed) == 4:
            polygon = proposed
            break
    if polygon is None:
        raise RuntimeError('Could not approximate the green face with four corners')
    overlay = frame.copy()
    cv2.polylines(overlay, [hull.astype(np.int32)], True, (0, 255, 255), 2)
    cv2.polylines(overlay, [polygon.astype(np.int32)], True, (0, 0, 255), 2)
    for index, pt in enumerate(polygon):
        cv2.putText(overlay, str(index), tuple(pt.astype(int)), cv2.FONT_HERSHEY_SIMPLEX,
                    .7, (255, 0, 255), 2)
    (ROOT / f'{args.output_stem}.png').write_bytes(cv2.imencode('.png', overlay)[1].tobytes())
    with np.load(ROOT / 'camera1_720.npz') as camera:
        K, dist = camera['K'].copy(), camera['dist'].copy()
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    board_points, _ = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                               layout='centers', face_rot=rotations)
    marker5_obj = board_points[5]
    ok, marker_rv, marker_tv = cv2.solvePnP(marker5_obj, marker5, K, dist)
    if not ok:
        raise RuntimeError('Marker pose solve failed')
    face_n, face_u0, face_v0 = cube_board.FACES[5]
    face_u, face_v = cube_board._rot_uv(face_u0, face_v0, rotations[5])
    h = 54.5 / 2000
    results = []
    extents = (48., 50., 50.5, 52., 54.5)
    for sticker_extent_mm in extents:
        s = sticker_extent_mm / 2000
        face_center = h * face_n
        obj = np.asarray([face_center-s*face_u+s*face_v, face_center+s*face_u+s*face_v,
                          face_center+s*face_u-s*face_v, face_center-s*face_u-s*face_v], np.float32)
        for reverse in (False, True):
            base = polygon[::-1] if reverse else polygon
            for offset in range(4):
                img = np.roll(base, -offset, axis=0).astype(np.float32)
                ok, rv, tv = cv2.solvePnP(obj, img, K, dist)
                if not ok:
                    continue
                marker_projected, _ = cv2.projectPoints(marker5_obj, rv, tv, K, dist)
                marker_error = float(np.linalg.norm(marker_projected.reshape(4, 2)-marker5, axis=1).mean())
                delta = tv.reshape(3) - marker_tv.reshape(3)
                R_marker = cv2.Rodrigues(marker_rv)[0]
                R_outline = cv2.Rodrigues(rv)[0]
                relative = R_marker.T @ R_outline
                angle = float(np.degrees(np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1))))
                results.append({'sticker_extent_mm': sticker_extent_mm,
                                'marker_reprojection_px': marker_error,
                                'rotation_delta_deg_vs_marker5': angle,
                                'center_delta_camera_mm': (1000*delta).tolist(),
                                'center_delta_norm_mm': float(1000*np.linalg.norm(delta)),
                                'cube_center_camera_m': tv.reshape(3).tolist()})
    best = [min((r for r in results if r['sticker_extent_mm'] == extent),
                key=lambda r: r['marker_reprojection_px']) for extent in extents]
    report = {'source': str(args.frame), 'detected_ids': ids,
              'marker5_center_px': center_px.tolist(),
              'green_component_areas_px': [area for _, area in comps[:9]],
              'green_face_polygon_px': polygon.tolist(),
              'marker5_center_camera_m': marker_tv.reshape(3).tolist(),
              'best_by_sticker_extent': best,
              'measurement_status': 'provisional_color_outline_diagnostic'}
    (ROOT / f'{args.output_stem}_result.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
