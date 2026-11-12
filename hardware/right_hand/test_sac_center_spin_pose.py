"""Center markers must remain usable when a Rubik face has turned."""

import json

import cv2 as cv
import numpy as np

from sac_center_spin_pose import ROOT, cube_board, fit, fit_image


def main():
    with np.load(ROOT / 'camera1_720.npz') as z:
        K, dist = z['K'].copy(), z['dist'].copy()
    # Different face turns change the center-marker spins independently.
    spins = [0, 0, 0, 2, 0, 3]
    points, _ = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                         layout='centers', face_rot=spins)
    rv = np.array([.2, -.3, .1])
    tv = np.array([.01, -.015, .33])
    detected = {}
    for marker_id in (3, 5):
        projected, _ = cv.projectPoints(points[marker_id], rv, tv, K, dist)
        detected[marker_id] = projected.reshape(4, 2).astype(np.float32)
    result = fit(detected, K, dist)
    assert result['best']['spins'] == {3: 2, 5: 3}, result
    assert result['best']['mean_reprojection_px'] < .001, result
    archived = fit_image(ROOT / 'leap_two_face_preview_20261009_225422_808088.png')
    assert archived['visible_ids'] == (3, 5), archived
    assert archived['best']['mean_reprojection_px'] < 3, archived
    print(json.dumps(dict(synthetic_spins=result['best']['spins'],
                          synthetic_reprojection_px=result['best']['mean_reprojection_px'],
                          archive_spins=archived['best']['spins'],
                          archive_reprojection_px=archived['best']['mean_reprojection_px'],
                          archive_gap_px=archived['ambiguity_gap_px']), indent=2))


if __name__ == '__main__':
    main()
