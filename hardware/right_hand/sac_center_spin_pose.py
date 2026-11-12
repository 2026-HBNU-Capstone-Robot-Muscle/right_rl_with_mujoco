"""Offline pose fit using only Rubik cube CENTER markers after face turns.

Center pieces stay at the same face centers after a scramble, but their printed
ArUco patterns can rotate in 90-degree steps.  Enumerating those orientations
avoids treating an old face-rotation calibration as permanent.  This module
does not supply single-marker or fully occluded poses to a motor controller.
"""

import itertools
import json
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'vision' / 'rubik_center'))
import cube_board


def detect(frame):
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    detector = cv.aruco.ArucoDetector(dictionary, cv.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(cv.cvtColor(frame, cv.COLOR_BGR2GRAY))
    return ({} if ids is None else
            {int(i): np.asarray(c, np.float32).reshape(4, 2)
             for i, c in zip(ids.ravel(), corners) if 0 <= int(i) <= 5})


def fit(detected, K, dist, *, max_reprojection_px=3.0, prior_rotation=None):
    visible = tuple(sorted(i for i in detected if 0 <= i <= 5))
    if len(visible) < 2:
        raise ValueError('Two center markers on different faces required for an observed pose')
    # All candidates use the same ID-specific face centers, so face scrambling
    # affects only the 90-degree in-plane marker spin.
    measured = np.concatenate([detected[i] for i in visible])
    candidates = []
    for spins in itertools.product(range(4), repeat=len(visible)):
        rotations = [0] * 6
        for marker_id, spin in zip(visible, spins):
            rotations[marker_id] = spin
        points, _ = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                             layout='centers', face_rot=rotations)
        object_corners = np.concatenate([points[i] for i in visible])
        ok, rv, tv = cv.solvePnP(object_corners, measured, K, dist,
                                  flags=cv.SOLVEPNP_ITERATIVE)
        if not ok or float(tv[2]) <= 0:
            continue
        projected, _ = cv.projectPoints(object_corners, rv, tv, K, dist)
        errors = np.linalg.norm(projected.reshape(-1, 2) - measured, axis=1)
        mean_error = float(errors.mean())
        R = cv.Rodrigues(rv)[0]
        prior_angle = 0.0
        if prior_rotation is not None:
            prior_angle = float(np.degrees(np.arccos(np.clip(
                (np.trace(np.asarray(prior_rotation).T @ R) - 1) / 2, -1, 1))))
        candidates.append(dict(spins=dict(zip(visible, spins)),
                               mean_reprojection_px=mean_error,
                               max_corner_reprojection_px=float(errors.max()),
                               prior_rotation_difference_deg=prior_angle,
                               cube_center_camera_m=tv.ravel().tolist(),
                               cube_rotation_camera=R.tolist()))
    if not candidates:
        raise RuntimeError('No positive-depth center-marker pose')
    candidates.sort(key=lambda row: (row['mean_reprojection_px'],
                                     row['prior_rotation_difference_deg']))
    best = candidates[0]
    if best['mean_reprojection_px'] > max_reprojection_px:
        raise RuntimeError(f"Center-marker reprojection {best['mean_reprojection_px']:.2f}px > "
                           f'{max_reprojection_px:g}px')
    return dict(visible_ids=visible, candidate_count=len(candidates),
                best=best, runner_up_reprojection_px=(
                    candidates[1]['mean_reprojection_px'] if len(candidates) > 1 else None),
                ambiguity_gap_px=(candidates[1]['mean_reprojection_px'] -
                                  best['mean_reprojection_px'] if len(candidates) > 1 else None))


def fit_image(path):
    path = Path(path)
    frame = cv.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f'Cannot decode {path}')
    with np.load(ROOT / 'camera1_720.npz') as z:
        K, dist = z['K'].copy(), z['dist'].copy()
    return fit(detect(frame), K, dist)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('image')
    args = parser.parse_args()
    print(json.dumps(fit_image(args.image), indent=2))
