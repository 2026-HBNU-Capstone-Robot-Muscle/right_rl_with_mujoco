"""Offline two-photo overlay; diagnostic only, not a 3D hand registration."""

import json
from pathlib import Path

import cv2 as cv
import numpy as np


ROOT = Path(__file__).resolve().parent
WITH = ROOT / "leap_palm_photo_20261008_200826_400273.png"
EMPTY = ROOT / "leap_palm_photo_20261008_201116_472718.png"


def main():
    with_cube = cv.imdecode(np.fromfile(WITH, dtype=np.uint8), cv.IMREAD_COLOR)
    empty = cv.imdecode(np.fromfile(EMPTY, dtype=np.uint8), cv.IMREAD_COLOR)
    if with_cube is None or empty is None:
        raise FileNotFoundError("Photo pair missing")
    mask = np.zeros(with_cube.shape[:2], np.uint8)
    # Match only rigid hand parts visible in both images, excluding the cube.
    cv.rectangle(mask, (220, 0), (940, 680), 255, -1)
    cv.rectangle(mask, (520, 255), (815, 560), 0, -1)
    sift = cv.SIFT_create(nfeatures=2500)
    p1, d1 = sift.detectAndCompute(cv.cvtColor(with_cube, cv.COLOR_BGR2GRAY), mask)
    p2, d2 = sift.detectAndCompute(cv.cvtColor(empty, cv.COLOR_BGR2GRAY), None)
    pairs = cv.FlannBasedMatcher(dict(algorithm=1, trees=5), dict(checks=100)).knnMatch(d1, d2, k=2)
    good = [a for a, b in pairs if a.distance < 0.72 * b.distance]
    src = np.float32([p1[m.queryIdx].pt for m in good])
    dst = np.float32([p2[m.trainIdx].pt for m in good])
    H, inlier = cv.findHomography(src, dst, cv.RANSAC, 4.0)
    if H is None:
        raise RuntimeError("No 2D alignment")
    support = np.float32([[[535, 525], [780, 555], [755, 595], [500, 565]]])
    mapped = cv.perspectiveTransform(support, H)[0]
    view = empty.copy()
    cv.polylines(view, [np.int32(mapped)], True, (0, 0, 255), 3)
    cv.putText(view, "Approx projected cube lower edge; 2D only", (30, 690),
               cv.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
    ok, encoded = cv.imencode(".png", view)
    if not ok:
        raise RuntimeError("Overlay encoding failed")
    (ROOT / "sac_palm_photo_pair_overlay.png").write_bytes(encoded.tobytes())
    report = {"matches": len(good), "inliers": int(inlier.sum()),
              "inlier_fraction": float(inlier.mean()),
              "homography": H.tolist(), "projected_lower_edge": mapped.tolist(),
              "note": "Hand-part image match only; no cube underside or contact height measured."}
    (ROOT / "sac_palm_photo_pair_result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
