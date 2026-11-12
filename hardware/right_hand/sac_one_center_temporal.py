"""Offline-only one-center pose hypotheses with a recent two-center prior.

The prior is a necessary disambiguation aid, not evidence that the cube was
observed through an occlusion.  The caller must enforce a short age bound and
must not treat this as a live motor-control signal without sequence validation.
"""

import itertools

import cv2 as cv
import numpy as np

from sac_center_spin_pose import cube_board


def _rotation_distance_deg(a, b):
    return float(np.degrees(np.arccos(np.clip(
        (np.trace(np.asarray(a).T @ np.asarray(b)) - 1) / 2, -1, 1))))


def hypotheses(marker_id, corners_px, K, dist):
    if not 0 <= marker_id <= 5:
        raise ValueError('Use only center marker IDs 0..5')
    corners_px = np.asarray(corners_px, np.float32).reshape(4, 2)
    rows = []
    for spin in range(4):
        rotations = [0] * 6
        rotations[marker_id] = spin
        points, _ = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                             layout='centers', face_rot=rotations)
        obj = points[marker_id]
        count, rvecs, tvecs, _ = cv.solvePnPGeneric(
            obj, corners_px, K, dist, flags=cv.SOLVEPNP_IPPE)
        if not count:
            continue
        for branch, (rv, tv) in enumerate(zip(rvecs, tvecs)):
            if float(tv[2]) <= 0:
                continue
            projection, _ = cv.projectPoints(obj, rv, tv, K, dist)
            reproj = float(np.linalg.norm(projection.reshape(4, 2) - corners_px, axis=1).mean())
            rows.append(dict(spin=spin, branch=branch, cube_center_camera_m=tv.ravel().tolist(),
                             cube_rotation_camera=cv.Rodrigues(rv)[0].tolist(),
                             marker_reprojection_px=reproj))
    return rows


def choose(marker_id, corners_px, K, dist, prior_center_m, prior_rotation,
           prior_age_s, *, max_prior_age_s=.15, max_position_change_mm=12,
           max_rotation_change_deg=25, max_marker_reprojection_px=2.5):
    if prior_age_s < 0 or prior_age_s > max_prior_age_s:
        raise ValueError('Prior too old for one-center pose selection')
    prior_center_m = np.asarray(prior_center_m, np.float64)
    prior_rotation = np.asarray(prior_rotation, np.float64)
    rows = hypotheses(marker_id, corners_px, K, dist)
    eligible = []
    for row in rows:
        center = np.asarray(row['cube_center_camera_m'])
        R = np.asarray(row['cube_rotation_camera'])
        center_change = float(1000 * np.linalg.norm(center - prior_center_m))
        rotation_change = _rotation_distance_deg(R, prior_rotation)
        row.update(position_change_from_prior_mm=center_change,
                   rotation_change_from_prior_deg=rotation_change)
        if (center_change <= max_position_change_mm and
                rotation_change <= max_rotation_change_deg and
                row['marker_reprojection_px'] <= max_marker_reprojection_px):
            eligible.append(row)
    if not eligible:
        raise RuntimeError('One-center image does not agree with recent prior')
    # Rank by normalized physical displacement; a low 4-corner reprojection
    # alone cannot distinguish planar marker-pose branches.
    eligible.sort(key=lambda row: (row['position_change_from_prior_mm'] / max_position_change_mm) ** 2
                  + (row['rotation_change_from_prior_deg'] / max_rotation_change_deg) ** 2
                  + (row['marker_reprojection_px'] / max_marker_reprojection_px) ** 2)
    return dict(marker_id=marker_id, prior_age_s=prior_age_s,
                candidate_count=len(rows), eligible_count=len(eligible),
                selected=eligible[0])
