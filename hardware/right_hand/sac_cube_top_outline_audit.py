"""Offline color-outline cross-check for the stage-0 green cube face.

The color boundary is approximate, so this is a diagnostic, never a policy reset.
"""
import json
from pathlib import Path

import cv2
import numpy as np

from sac_registration_from_anchor import detect
import cube_board

ROOT = Path(__file__).resolve().parent
FRAME = ROOT / 'sequence_vision_20261006_203648_332236' / 'placement_at_space.png'


def main():
    frame = cv2.imdecode(np.frombuffer(FRAME.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError('Placement image missing')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([32, 55, 25]), np.array([90, 255, 255]))
    region = np.zeros(mask.shape, np.uint8)
    region[0:225, 390:715] = 255
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
        raise RuntimeError('No green face pixels detected')
    pts = np.column_stack((xs, ys)).astype(np.float32)
    hull = cv2.convexHull(pts).reshape(-1, 2)
    epsilon = .055 * cv2.arcLength(hull, True)
    polygon = cv2.approxPolyDP(hull, epsilon, True).reshape(-1, 2)
    overlay = frame.copy()
    cv2.polylines(overlay, [hull.astype(np.int32)], True, (0, 255, 255), 2)
    cv2.polylines(overlay, [polygon.astype(np.int32)], True, (0, 0, 255), 2)
    for index, pt in enumerate(polygon):
        cv2.putText(overlay, str(index), tuple(pt.astype(int)), cv2.FONT_HERSHEY_SIMPLEX,
                    .7, (255, 0, 255), 2)
    (ROOT / 'sac_cube_top_outline_audit.png').write_bytes(cv2.imencode('.png', overlay)[1].tobytes())
    with np.load(ROOT / 'camera1_720.npz') as camera:
        K, dist = camera['K'].copy(), camera['dist'].copy()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    _, corners, ids_detected, _ = detect(FRAME, dictionary)
    marker5 = next(np.asarray(c, np.float32).reshape(4, 2)
                   for c, marker_id in zip(corners, ids_detected) if marker_id == 5)
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    board_points, _ = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                               layout='centers', face_rot=rotations)
    marker5_obj = board_points[5]
    marker5_ok, marker5_rv, marker5_tv = cv2.solvePnP(marker5_obj, marker5, K, dist)
    if not marker5_ok:
        raise RuntimeError('ID5 pose failed')
    face_n, face_u0, face_v0 = cube_board.FACES[5]
    face_u, face_v = cube_board._rot_uv(face_u0, face_v0, rotations[5])
    h = 54.5 / 2000
    assessments = []
    for sticker_extent_mm in (48.0, 50.0, 52.0, 54.5):
        s = sticker_extent_mm / 2000
        center = h * face_n
        obj = np.asarray([center-s*face_u+s*face_v, center+s*face_u+s*face_v,
                          center+s*face_u-s*face_v, center-s*face_u-s*face_v], np.float32)
        for reverse in (False, True):
            base = polygon[::-1] if reverse else polygon
            for offset in range(4):
                img = np.roll(base, -offset, axis=0).astype(np.float32)
                ok, rv, tv = cv2.solvePnP(obj, img, K, dist)
                if not ok:
                    continue
                marker_projected, _ = cv2.projectPoints(marker5_obj, rv, tv, K, dist)
                marker_error = float(np.linalg.norm(marker_projected.reshape(4, 2)-marker5, axis=1).mean())
                image_projected, _ = cv2.projectPoints(obj, rv, tv, K, dist)
                quad_error = float(np.linalg.norm(image_projected.reshape(4, 2)-img, axis=1).mean())
                delta = tv.reshape(3) - marker5_tv.reshape(3)
                assessments.append({'sticker_extent_mm': sticker_extent_mm,
                                    'reverse': reverse, 'offset': offset,
                                    'marker_reprojection_px': marker_error,
                                    'quad_reprojection_px': quad_error,
                                    'center_delta_vs_marker5_mm': (delta*1000).tolist(),
                                    'center_delta_norm_mm': float(np.linalg.norm(delta)*1000),
                                    'cube_center_camera_m': tv.reshape(3).tolist()})
    best_by_extent = [min((a for a in assessments if a['sticker_extent_mm'] == extent),
                          key=lambda a: a['marker_reprojection_px'])
                      for extent in (48.0, 50.0, 52.0, 54.5)]
    report = {'source': str(FRAME), 'green_component_areas_px': [area for _, area in comps[:9]],
              'selected_green_pixels': len(xs), 'hull_vertices': len(hull),
              'polygon_xy_px': polygon.tolist(), 'marker5_center_camera_m': marker5_tv.reshape(3).tolist(),
              'best_outline_pose_by_assumed_extent': best_by_extent,
              'measurement_status': 'color_outline_provisional_not_hardware_authorized'}
    (ROOT / 'sac_cube_top_outline_audit_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
