"""Read-only cube marker ID inventory for representative archived images."""

from pathlib import Path

import cv2 as cv
import numpy as np

ROOT = Path(__file__).resolve().parent
detector = cv.aruco.ArucoDetector(
    cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
    cv.aruco.DetectorParameters())
for path in sorted((ROOT / 'vision_sweep_20261006_011011_994373').glob('*eligible.png')):
    image = cv.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv.IMREAD_COLOR)
    _, ids, _ = detector.detectMarkers(cv.cvtColor(image, cv.COLOR_BGR2GRAY))
    print(path.name, [] if ids is None else ids.ravel().tolist())
