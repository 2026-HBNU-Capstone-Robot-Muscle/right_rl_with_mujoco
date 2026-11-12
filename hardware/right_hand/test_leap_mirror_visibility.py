"""Motor-free mirrored ArUco detector check."""

import cv2 as cv
import numpy as np

from leap_mirror_visibility import detector, visible_ids


def main():
    frame = np.full((720, 1280, 3), 255, np.uint8)
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    direct = cv.aruco.generateImageMarker(dictionary, 5, 100)
    mirror = cv.aruco.generateImageMarker(dictionary, 3, 100)
    frame[100:200, 100:200] = cv.cvtColor(direct, cv.COLOR_GRAY2BGR)
    frame[300:400, 700:800] = cv.cvtColor(cv.flip(mirror, 1), cv.COLOR_GRAY2BGR)
    seen_direct, seen_reflection = visible_ids(frame, (600, 250, 300, 220), detector())
    assert seen_direct == [5], seen_direct
    assert seen_reflection == [3], seen_reflection
    print('One-webcam direct+mirror marker separation: PASS')


if __name__ == '__main__':
    main()
