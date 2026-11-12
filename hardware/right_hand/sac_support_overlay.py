"""Overlay approximate joint-axis landmarks on the saved stage-0 placement frame."""
import json
import argparse
from pathlib import Path

import cv2
import numpy as np

from hand_registration_geometry import candidate
from sac_registration_from_anchor import detect, marker_pose

ROOT = Path(__file__).resolve().parent
DEFAULT_FOLDER = ROOT / 'sequence_vision_20261006_203648_332236'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', type=Path, default=DEFAULT_FOLDER)
    parser.add_argument('--empty-image', default='anchor_empty_stage0.png')
    args = parser.parse_args()
    folder = args.folder if args.folder.is_absolute() else ROOT / args.folder
    with np.load(ROOT / 'camera1_720.npz') as camera:
        K, dist = camera['K'].copy(), camera['dist'].copy()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    _, empty_corners, empty_ids, _ = detect(folder / args.empty_image, dictionary)
    frame, placed_corners, placed_ids, _ = detect(folder / 'placement_at_space.png', dictionary)
    R_h0, t_h0, _ = marker_pose(empty_corners, empty_ids, 40, .0282, K, dist)
    R_a0, t_a0, _ = marker_pose(empty_corners, empty_ids, 41, .030, K, dist)
    R_a1, t_a1, _ = marker_pose(placed_corners, placed_ids, 41, .030, K, dist)
    R_h1 = R_a1 @ R_a0.T @ R_h0
    t_h1 = t_a1 + R_a1 @ R_a0.T @ (t_h0-t_a0)
    hand = candidate()
    landmarks = hand['motor_axis_landmarks_in_marker_m']
    points = np.asarray([landmarks[k] for k in ('5', '9', '13', '1')], np.float32)
    rvec, _ = cv2.Rodrigues(R_h1)
    projected, _ = cv2.projectPoints(points, rvec, t_h1, K, dist)
    projected = projected.reshape(-1, 2)
    labels = ('ID5 index MCP', 'ID9 middle MCP', 'ID13 ring MCP', 'ID1 thumb axis')
    colors = ((0, 255, 255), (0, 255, 0), (255, 0, 255), (255, 0, 0))
    for (px, py), label, color in zip(projected, labels, colors):
        center = (int(round(px)), int(round(py)))
        cv2.circle(frame, center, 8, color, 3)
        cv2.putText(frame, label, (center[0]+8, center[1]-8),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
    suffix = '_latest' if folder != DEFAULT_FOLDER else ''
    image_path = ROOT / f'sac_support_overlay{suffix}.png'
    image_path.write_bytes(cv2.imencode('.png', frame)[1].tobytes())
    report = {'source': str(folder / 'placement_at_space.png'),
              'estimated_axes_px': {label: p.tolist() for label, p in zip(labels, projected)},
              'registration': 'provisional ID40-to-ID41 and marker-to-model transform',
              'hardware_authorized': False}
    (ROOT / f'sac_support_overlay{suffix}_result.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
