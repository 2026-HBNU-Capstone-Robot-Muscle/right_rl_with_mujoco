"""Offline board-pose quality check for motor-free two-face preview."""

import json
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

from sac_registration_from_anchor import detect


ROOT = Path(__file__).resolve().parent
FRAME = ROOT / 'leap_two_face_preview_20261009_223918_666289.png'
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/vision/rubik_center'))
import cube_board
import track_cube


def main():
    with np.load(ROOT / 'camera1_720.npz') as f:
        K, dist = f['K'].copy(), f['dist'].copy()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    frame, _, ids, detector = detect(FRAME, dictionary)
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    points, board_ids = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                                  layout='centers', face_rot=rotations)
    board = cv.aruco.Board(points, dictionary, board_ids)
    pose = track_cube.step(frame, board, detector, dictionary, K, dist)
    report = {'image': str(FRAME), 'detected_ids': ids,
              'cube_marker_count': None if pose is None else pose.get('n_cube_markers'),
              'cube_reprojection_px': None if pose is None else float(pose.get('reproj', float('nan'))),
              'passes_powered_anchor_gate': bool(pose is not None and pose.get('n_cube_markers', 0) >= 2
                                                 and np.isfinite(pose.get('reproj', float('nan')))
                                                 and pose['reproj'] <= 3)}
    (ROOT / 'sac_two_face_preview_quality_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
