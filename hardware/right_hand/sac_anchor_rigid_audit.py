"""Check the saved ID40-to-ID41 rigid transform across empty-hand frames."""
import json
from pathlib import Path

import cv2
import numpy as np

from sac_registration_from_anchor import detect, marker_pose

ROOT = Path(__file__).resolve().parent
IMAGES = ('preflight_not_ready.png', 'configure_off_not_ready.png',
          'enable_hold_not_ready.png', 'stage_0_not_ready.png',
          'anchor_empty_stage0.png')


def main():
    run = json.loads((ROOT / 'leap_anchor_capture_result.json').read_text(encoding='utf-8'))
    folder = Path(run['vision_recording']['folder'])
    with np.load(ROOT / 'camera1_720.npz') as cal:
        K, distortion = cal['K'], cal['dist']
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    rows = []
    all_views = []
    for name in IMAGES:
        frame, corners, ids, _ = detect(folder / name, dictionary)
        R_h, t_h, h_err = marker_pose(corners, ids, 40, .0282, K, distortion)
        R_a, t_a, a_err = marker_pose(corners, ids, 41, .030, K, distortion)
        anchor_corners = next(np.asarray(c).reshape(4, 2) for c, i in zip(corners, ids) if i == 41)
        anchor_edges = np.linalg.norm(anchor_corners - np.roll(anchor_corners, 1, axis=0), axis=1)
        rows.append(dict(image=name, hand_in_anchor_m=(R_a.T @ (t_h - t_a)).tolist(),
                         hand_camera_m=t_h.tolist(), anchor_camera_m=t_a.tolist(),
                         anchor_edge_px=anchor_edges.tolist(),
                         hand_R_in_anchor=(R_a.T @ R_h).tolist(),
                         hand_reprojection_px=h_err, anchor_reprojection_px=a_err))
        hand_corners = next(np.asarray(c).reshape(4, 2) for c, i in zip(corners, ids) if i == 40)
        all_views.append((R_a, t_a, hand_corners))
    reference = np.asarray(rows[0]['hand_in_anchor_m'])
    R0 = np.asarray(rows[0]['hand_R_in_anchor'])
    for row in rows:
        row['shift_from_preflight_mm'] = float(np.linalg.norm(np.asarray(row['hand_in_anchor_m']) - reference) * 1000)
        relative = R0.T @ np.asarray(row['hand_R_in_anchor'])
        row['rotation_from_preflight_deg'] = float(np.degrees(np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1))))
    s = .0282 / 2
    marker_obj = np.asarray([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    for row, (R_a, t_a, observed) in zip(rows, all_views):
        R_pred = R_a @ R0
        t_pred = t_a + R_a @ reference
        rvec, _ = cv2.Rodrigues(R_pred)
        projected, _ = cv2.projectPoints(marker_obj, rvec, t_pred, K, distortion)
        row['preflight_rigid_projection_error_px'] = float(
            np.linalg.norm(projected.reshape(4, 2) - observed, axis=1).mean())
    result = {'source_folder': str(folder), 'rows': rows}
    (ROOT / 'sac_anchor_rigid_audit_result.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
