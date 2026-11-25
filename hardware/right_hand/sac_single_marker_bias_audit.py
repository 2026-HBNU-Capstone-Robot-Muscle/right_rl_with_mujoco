"""Compare one-face cube PnP with a simultaneous two-face reference, offline."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'vision' / 'rubik_center'))
import cube_board


def solve(board, visible, ids, intrinsic, distortion):
    corners = [np.asarray(visible[i], np.float32).reshape(1, 4, 2) for i in ids]
    obj, img = board.matchImagePoints(corners, np.asarray(ids, np.int32).reshape(-1, 1))
    ok, rv, tv = cv2.solvePnP(obj, img, intrinsic, distortion,
                             flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError(f'PnP failed for {ids}')
    projected, _ = cv2.projectPoints(obj, rv, tv, intrinsic, distortion)
    reproj = float(np.linalg.norm(projected.reshape(-1, 2) - img.reshape(-1, 2), axis=1).mean())
    return cv2.Rodrigues(rv)[0], tv.reshape(3), reproj


def planar_solutions(board, visible, cube_id, intrinsic, distortion):
    corners = [np.asarray(visible[cube_id], np.float32).reshape(1, 4, 2)]
    obj, img = board.matchImagePoints(corners, np.asarray([[cube_id]], np.int32))
    _n, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        obj, img, intrinsic, distortion, flags=cv2.SOLVEPNP_IPPE)
    results = []
    for rv, tv, error in zip(rvecs, tvecs, errors):
        results.append((cv2.Rodrigues(rv)[0], tv.reshape(3), float(np.asarray(error).item())))
    return results


def report_one(path, board):
    record = json.loads(path.read_text(encoding='utf-8'))
    geometry = record.get('geometry', {})
    if not geometry.get('camera_matrix') or not geometry.get('distortion'):
        return None
    intrinsic = np.asarray(geometry['camera_matrix'], np.float64)
    distortion = np.asarray(geometry['distortion'], np.float64)
    visible = {int(item['id']): item['corners_px'] for item in record['geometry']['markers']
               if int(item['id']) < 40}
    if len(visible) < 2:
        return None
    full_ids = sorted(visible)
    full_R, full_t, full_error = solve(board, visible, full_ids, intrinsic, distortion)
    singles = {}
    for cube_id in full_ids:
        R, t, reproj = solve(board, visible, [cube_id], intrinsic, distortion)
        delta = full_R.T @ R
        rotation = np.degrees(np.arccos(np.clip((np.trace(delta)-1)/2, -1, 1)))
        planar = []
        for ippe_R, ippe_t, ippe_error in planar_solutions(board, visible, cube_id,
                                                           intrinsic, distortion):
            delta_ippe = full_R.T @ ippe_R
            planar.append({'reprojection_px': ippe_error,
                           'center_error_mm': float(np.linalg.norm(ippe_t-full_t)*1000),
                           'rotation_error_deg': float(np.degrees(np.arccos(np.clip(
                               (np.trace(delta_ippe)-1)/2, -1, 1))))})
        singles[str(cube_id)] = {
            'reprojection_px': reproj,
            'center_error_mm': float(np.linalg.norm(t-full_t)*1000),
            'center_delta_camera_mm': ((t-full_t)*1000).tolist(),
            'rotation_error_deg': float(rotation),
            'depth_m': float(t[2]),
            'ippe_solutions': planar,
        }
    return {'source': str(path.relative_to(ROOT)), 'reference_ids': full_ids,
            'reference_reprojection_px': full_error,
            'reference_depth_m': float(full_t[2]), 'single_markers': singles}


def main():
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    points, ids = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                            layout='centers', face_rot=rotations)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.Board(points, dictionary, ids)
    sources = sorted(ROOT.glob('*/eligible.json'))
    out = [item for path in sources if (item := report_one(path, board)) is not None]
    summary = {}
    improved = []
    for item in out:
        if item['reference_reprojection_px'] > 3:
            continue
        for cube_id, datum in item['single_markers'].items():
            summary.setdefault(cube_id, []).append(datum['center_error_mm'])
            improved.append({'id': cube_id, 'iterative_mm': datum['center_error_mm'],
                             'best_ippe_mm': min(x['center_error_mm'] for x in datum['ippe_solutions'])})
    summary = {cube_id: {'count': len(errors),
                         'median_center_error_mm': float(np.median(errors)),
                         'max_center_error_mm': float(np.max(errors))}
               for cube_id, errors in summary.items()}
    result = {'reference_frames': out, 'summary_valid_reprojection': summary,
              'ippe_comparison': {'count': len(improved),
                                  'median_iterative_mm': float(np.median([x['iterative_mm'] for x in improved])),
                                  'median_best_ippe_mm': float(np.median([x['best_ippe_mm'] for x in improved]))}}
    destination = ROOT / 'sac_single_marker_bias_audit_result.json'
    destination.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'reference_count': len(out), 'summary': summary,
                      'ippe_comparison': result['ippe_comparison']}, indent=2))


if __name__ == '__main__':
    main()
