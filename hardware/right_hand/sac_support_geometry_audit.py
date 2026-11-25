"""Compare photographed cube extent to approximate first-link motor axes."""
import json
import itertools
from pathlib import Path

import numpy as np

from hand_registration_geometry import candidate

ROOT = Path(__file__).resolve().parent


def quat_rotation(q):
    w, x, y, z = np.asarray(q, dtype=float)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def main():
    registration = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    hand = candidate()
    R_wm = np.asarray(hand['hand_to_world_candidate'])
    R_wc = quat_rotation(registration['cube_quat_world_candidate'])
    R_mc = R_wm.T @ R_wc
    center = np.asarray(registration['cube_center_in_hand_marker_m'])
    side = .0545
    corners = np.array([center + R_mc @ (np.asarray(signs)*side/2)
                        for signs in itertools.product((-1, 1), repeat=3)])
    axes = hand['motor_axis_landmarks_in_marker_m']
    report = {'cube_center_marker_mm': (center*1000).tolist(),
              'cube_corner_forward_range_marker_mm': [float(corners[:, 1].min()*1000),
                                                        float(corners[:, 1].max()*1000)],
              'cube_corner_lateral_range_marker_mm': [float(corners[:, 0].min()*1000),
                                                        float(corners[:, 0].max()*1000)],
              'middle_mcp_axis_forward_marker_mm': 1000*axes['9'][1],
              'cube_center_behind_middle_axis_mm': float(1000*(axes['9'][1]-center[1])),
              'model_marker_transform_verified': False,
              'hardware_authorized': False}
    (ROOT / 'sac_support_geometry_audit_result.json').write_text(
        json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
