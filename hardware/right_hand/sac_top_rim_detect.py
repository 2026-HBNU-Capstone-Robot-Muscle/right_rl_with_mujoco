"""Camera-only diagnostic for the exposed green top face around cube marker 5.

The returned quadrilateral bounds the green stickers, not the physical plastic
rim. It must be calibrated before use as a metric cube pose measurement.
"""

import argparse
import json
from pathlib import Path

import cv2 as cv
import numpy as np


ROOT = Path(__file__).resolve().parent


def detect(image):
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    detector = cv.aruco.ArucoDetector(dictionary, cv.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(cv.cvtColor(image, cv.COLOR_BGR2GRAY))
    if ids is None or 5 not in ids:
        raise RuntimeError('Top marker ID5 not detected')
    marker = np.asarray(corners[list(ids.ravel()).index(5)]).reshape(4, 2)
    center = marker.mean(axis=0)
    side_px = np.linalg.norm(marker - np.roll(marker, 1, axis=0), axis=1).mean()
    # The 54.5-mm top face spans roughly five marker widths. Exclude remote
    # green objects by limiting the search to the marker's neighborhood.
    radius = max(120, int(3.6 * side_px))
    yy, xx = np.indices(image.shape[:2])
    vicinity = (xx-center[0])**2 + (yy-center[1])**2 < radius**2
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    green = cv.inRange(hsv, np.array([35, 55, 35]), np.array([100, 255, 255]))
    green[~vicinity] = 0
    count, labels, stats, centroids = cv.connectedComponentsWithStats(green)
    selected = np.zeros_like(green)
    components = []
    for label in range(1, count):
        x, y, w, h, area = map(int, stats[label])
        if not 200 <= area <= 18000 or w > radius or h > radius:
            continue
        position = centroids[label]
        if np.linalg.norm(position-center) > radius * .78:
            continue
        selected[labels == label] = 255
        components.append(dict(area=area, center_px=position.tolist()))
    contours, _ = cv.findContours(selected, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if len(contours) < 4:
        raise RuntimeError(f'Only {len(contours)} green components; top grid incomplete')
    hull = cv.convexHull(np.concatenate(contours))
    perimeter = cv.arcLength(hull, True)
    polygon = None
    for factor in np.arange(.005, .061, .0025):
        candidate = cv.approxPolyDP(hull, factor*perimeter, True).reshape(-1, 2)
        if len(candidate) == 4:
            polygon = candidate.astype(float)
            break
    if polygon is None:
        raise RuntimeError('Green top hull did not yield four corners')
    # Ordered TL, TR, BR, BL for comparison with hand-read rim corners.
    sums = polygon.sum(axis=1)
    differences = polygon[:, 0] - polygon[:, 1]
    quad = polygon[[np.argmin(sums), np.argmax(differences),
                    np.argmax(sums), np.argmin(differences)]]
    if len({tuple(p) for p in quad}) != 4:
        raise RuntimeError('Top corners were ambiguous')
    return quad, marker, components, selected


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('image', type=Path)
    parser.add_argument('--output-stem', default='sac_top_rim_detect')
    args = parser.parse_args()
    path = args.image if args.image.is_absolute() else ROOT / args.image
    frame = cv.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv.IMREAD_COLOR)
    quad, marker, components, mask = detect(frame)
    report = dict(status='offline_sticker_boundary_diagnostic', hardware_authorized=False,
                  image=str(path), marker_center_px=marker.mean(axis=0).tolist(),
                  green_components=len(components), sticker_quad_px=quad.tolist(),
                  caveat='Sticker boundary is smaller than cube plastic rim; not a metric pose yet.')
    (ROOT / f'{args.output_stem}_result.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    overlay = frame.copy()
    cv.polylines(overlay, [quad.astype(np.int32)], True, (255, 0, 255), 2)
    cv.polylines(overlay, [marker.astype(np.int32)], True, (0, 255, 255), 2)
    (ROOT / f'{args.output_stem}_overlay.png').write_bytes(cv.imencode('.png', overlay)[1].tobytes())
    (ROOT / f'{args.output_stem}_mask.png').write_bytes(cv.imencode('.png', mask)[1].tobytes())
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
