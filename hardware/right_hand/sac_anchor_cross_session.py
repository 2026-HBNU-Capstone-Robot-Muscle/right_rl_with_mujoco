"""Compare ID40-to-ID41 transform across two saved empty-hand sessions."""
import json
from pathlib import Path

import cv2
import numpy as np

from sac_registration_from_anchor import detect, marker_pose


ROOT = Path(__file__).resolve().parent
IMAGES = {
    'old_preflight': ROOT / 'sequence_vision_20261006_203648_332236' / 'anchor_empty_preflight.png',
    'old_stage0': ROOT / 'sequence_vision_20261006_203648_332236' / 'anchor_empty_stage0.png',
    'new_preflight': ROOT / 'sequence_vision_20261007_000611_625058' / 'preflight_not_ready.png',
}


def main():
    with np.load(ROOT / 'camera1_720.npz') as cal:
        K, dist = cal['K'], cal['dist']
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    rows = []
    for label, path in IMAGES.items():
        _, corners, ids, _ = detect(path, dictionary)
        R_h, t_h, h_err = marker_pose(corners, ids, 40, .0282, K, dist)
        R_a, t_a, a_err = marker_pose(corners, ids, 41, .030, K, dist)
        rows.append({'label': label, 'detected_ids': ids,
                     'hand_in_anchor_m': (R_a.T @ (t_h - t_a)).tolist(),
                     'hand_R_in_anchor': (R_a.T @ R_h).tolist(),
                     'hand_reprojection_px': h_err, 'anchor_reprojection_px': a_err})
    reference = rows[0]
    R0 = np.asarray(reference['hand_R_in_anchor'])
    t0 = np.asarray(reference['hand_in_anchor_m'])
    for row in rows:
        R = np.asarray(row['hand_R_in_anchor'])
        row['shift_from_old_preflight_mm'] = float(1000*np.linalg.norm(np.asarray(row['hand_in_anchor_m'])-t0))
        row['rotation_from_old_preflight_deg'] = float(np.degrees(
            np.arccos(np.clip((np.trace(R0.T @ R)-1)/2, -1, 1))))
    R_stage0 = np.asarray(rows[1]['hand_R_in_anchor'])
    R_new = np.asarray(rows[2]['hand_R_in_anchor'])
    stage0_to_new = {
        'shift_mm': float(1000*np.linalg.norm(np.asarray(rows[2]['hand_in_anchor_m'])-
                                              np.asarray(rows[1]['hand_in_anchor_m']))),
        'rotation_deg': float(np.degrees(np.arccos(np.clip(
            (np.trace(R_stage0.T @ R_new)-1)/2, -1, 1))))}
    result = {'rows': rows, 'hardware_authorized': False,
              'old_stage0_vs_new_preflight': stage0_to_new,
              'limitation': 'Only empty-hand frames; later cube placement still lacks ID40.'}
    (ROOT / 'sac_anchor_cross_session_result.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
