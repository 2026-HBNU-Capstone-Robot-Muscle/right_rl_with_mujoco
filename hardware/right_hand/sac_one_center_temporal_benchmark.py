"""One-center optimistic upper bound on archived two-center still photos."""

import json

import cv2 as cv
import numpy as np

from sac_center_spin_pose import ROOT, detect, fit
from sac_one_center_temporal import choose, _rotation_distance_deg


IMAGES = [
    'leap_two_face_preview_20261009_223918_666289.png',
    'leap_two_face_preview_20261009_225422_808088.png',
    'vision_check_20261004_192532_747526/eligible.png',
    'vision_sweep_20261006_011011_994373/00-10s_eligible.png',
    'vision_sweep_20261006_011011_994373/10-20s_eligible.png',
]


def main():
    with np.load(ROOT / 'camera1_720.npz') as z:
        K, dist = z['K'].copy(), z['dist'].copy()
    rows = []
    for image_name in IMAGES:
        frame = cv.imdecode(np.frombuffer((ROOT / image_name).read_bytes(), np.uint8), cv.IMREAD_COLOR)
        centers = detect(frame)
        reference = fit(centers, K, dist)['best']
        center = np.asarray(reference['cube_center_camera_m'])
        rotation = np.asarray(reference['cube_rotation_camera'])
        for marker_id in centers:
            row = {'image': image_name, 'marker_id': marker_id,
                   'prior_type': 'same_frame_two_center_oracle'}
            try:
                result = choose(marker_id, centers[marker_id], K, dist,
                                center, rotation, .05)
                predicted = result['selected']
                center_mm = float(1000 * np.linalg.norm(
                    np.asarray(predicted['cube_center_camera_m']) - center))
                angle_deg = _rotation_distance_deg(predicted['cube_rotation_camera'], rotation)
                row.update(status='ok', center_difference_mm=center_mm,
                           rotation_difference_deg=angle_deg,
                           selected_spin=predicted['spin'],
                           selected_branch=predicted['branch'],
                           marker_reprojection_px=predicted['marker_reprojection_px'])
            except (RuntimeError, ValueError) as exc:
                row.update(status='failed', reason=str(exc))
            rows.append(row)
    report = {'status': 'offline_one_center_upper_bound',
              'motor_commands_sent': 0, 'rows': rows,
              'caveat': 'The same-frame two-center pose is used as an oracle prior. Real occlusion has no such oracle; this is only an optimistic upper bound.'}
    (ROOT / 'sac_one_center_temporal_benchmark_result.json').write_text(
        json.dumps(report, indent=2) + '\n')
    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
