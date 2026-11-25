"""Compare one-face color-grid pose with saved two-face ArUco fits."""

import json
from pathlib import Path

import cv2 as cv
import numpy as np

from sac_color_grid_pose import ROOT, fit
import sac_color_grid_pose as module

CASES = [
    ('yellow_orange', 'leap_two_face_preview_20261009_223918_666289.png', (1, 3)),
    ('green_orange_preview', 'leap_two_face_preview_20261009_225422_808088.png', (3, 5)),
    ('red_side', 'vision_check_20261004_192532_747526/eligible.png', (0, 2)),
    ('green_orange_sweep_a', 'vision_sweep_20261006_011011_994373/00-10s_eligible.png', (3, 5)),
    ('green_orange_sweep_b', 'vision_sweep_20261006_011011_994373/10-20s_eligible.png', (3, 5)),
]


def main():
    with np.load(ROOT / 'camera1_720.npz') as archive:
        K, dist = archive['K'].copy(), archive['dist'].copy()
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text())['face_rot'])
    points, _ = module.cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                                 layout='centers', face_rot=rotations)
    detector = cv.aruco.ArucoDetector(
        cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
        cv.aruco.DetectorParameters())
    rows = []
    for label, image_name, target_ids in CASES:
        frame = cv.imdecode(np.frombuffer((ROOT / image_name).read_bytes(), np.uint8),
                            cv.IMREAD_COLOR)
        found, ids, _ = detector.detectMarkers(cv.cvtColor(frame, cv.COLOR_BGR2GRAY))
        detected = {} if ids is None else {int(i): np.asarray(c, np.float32).reshape(4, 2)
                                            for i, c in zip(ids.ravel(), found)}
        available = [i for i in target_ids if i in detected]
        if len(available) != 2:
            raise RuntimeError(f'{label}: only IDs {available} detected')
        obj = np.concatenate([np.asarray(points[i], np.float32) for i in available])
        img = np.concatenate([detected[i] for i in available])
        ok, rv, tv = cv.solvePnP(obj, img, K, dist)
        if not ok:
            raise RuntimeError(f'{label}: two-face PnP failed')
        projected, _ = cv.projectPoints(obj, rv, tv, K, dist)
        reference_error = float(np.linalg.norm(projected.reshape(-1, 2) - img, axis=1).mean())
        R_ref = cv.Rodrigues(rv)[0]
        for marker_id in available:
            if marker_id not in module.COLOR_RANGES:
                continue
            row = {'case': label, 'marker_id': marker_id,
                   'reference_reprojection_px': reference_error}
            try:
                result = fit(frame, marker_id, K, dist, rotations)
                R_est = np.asarray(result['cube_rotation_camera'])
                angle = float(np.degrees(np.arccos(np.clip(
                    (np.trace(R_ref.T @ R_est) - 1) / 2, -1, 1))))
                row.update(status='ok', **result,
                           center_difference_mm=float(1000 * np.linalg.norm(
                               np.asarray(result['cube_center_camera_m']) - tv.ravel())),
                           rotation_difference_deg=angle)
            except (RuntimeError, ValueError) as exc:
                row.update(status='failed', error=str(exc))
            rows.append(row)
    latest = ROOT / 'sequence_vision_20261010_045958_610104' / 'placement_at_space.png'
    try:
        rows.append({'case': 'latest_one_face', 'status': 'ok',
                     **module.fit_image(latest, 5)})
    except (RuntimeError, ValueError) as exc:
        rows.append({'case': 'latest_one_face', 'status': 'failed', 'error': str(exc)})
    report = {'status': 'offline_color_grid_archive_benchmark',
              'hardware_authorized': False, 'results': rows,
              'caveat': 'Only three two-face reference photos; benchmark does not cover motion blur, full occlusion, or live control.'}
    (ROOT / 'sac_color_grid_archive_benchmark_result.json').write_text(
        json.dumps(report, indent=2) + '\n')
    print(json.dumps([{'case': r['case'], 'id': r.get('marker_id'),
                       'status': r['status'], 'error_mm': r.get('center_difference_mm'),
                       'error_deg': r.get('rotation_difference_deg'),
                       'reason': r.get('error')}
                      for r in rows], indent=2))


if __name__ == '__main__':
    main()
