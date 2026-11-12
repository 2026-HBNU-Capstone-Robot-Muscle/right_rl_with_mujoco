"""Compare image-feature and rigid-ID41 predictions of hidden ID40 (offline)."""

import json
from pathlib import Path

import cv2 as cv
import numpy as np

from sac_registration_from_anchor import detect, marker_pose


ROOT = Path(__file__).resolve().parent
OLD = ROOT / "sequence_vision_20261006_203648_332236" / "anchor_empty_stage0.png"
NEW = ROOT / "sequence_vision_20261008_191152_096085" / "placement_at_space.png"


def load(path):
    return cv.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv.IMREAD_COLOR)


def main():
    old = load(OLD)
    new = load(NEW)
    detector = cv.SIFT_create(nfeatures=3000)
    old_mask = np.zeros(old.shape[:2], np.uint8)
    new_mask = np.zeros(new.shape[:2], np.uint8)
    cv.rectangle(old_mask, (250, 0), (900, 630), 255, -1)
    cv.rectangle(new_mask, (200, 0), (930, 680), 255, -1)
    cv.rectangle(new_mask, (405, 70), (725, 425), 0, -1)
    k0, d0 = detector.detectAndCompute(cv.cvtColor(old, cv.COLOR_BGR2GRAY), old_mask)
    k1, d1 = detector.detectAndCompute(cv.cvtColor(new, cv.COLOR_BGR2GRAY), new_mask)
    pairs = cv.FlannBasedMatcher(dict(algorithm=1, trees=5), dict(checks=100)).knnMatch(d0, d1, k=2)
    matches = [a for a, b in pairs if a.distance < .72 * b.distance]
    src = np.float32([k0[m.queryIdx].pt for m in matches])
    dst = np.float32([k1[m.trainIdx].pt for m in matches])
    if len(matches) < 4:
        raise RuntimeError(f"Too few feature matches: {len(matches)}")
    H, inliers = cv.findHomography(src, dst, cv.RANSAC, 4.0)
    if H is None:
        raise RuntimeError("No image homography")
    with np.load(ROOT / "camera1_720.npz") as f:
        K, dist = f["K"].copy(), f["dist"].copy()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    _, old_corners, old_ids, _ = detect(OLD, dictionary)
    R_h0, t_h0, _ = marker_pose(old_corners, old_ids, 40, .0282, K, dist)
    R_a0, t_a0, _ = marker_pose(old_corners, old_ids, 41, .030, K, dist)
    _, new_corners, new_ids, _ = detect(NEW, dictionary)
    R_a1, t_a1, _ = marker_pose(new_corners, new_ids, 41, .030, K, dist)
    old40 = np.asarray(old_corners[old_ids.index(40)], np.float32).reshape(4, 2)
    image40 = cv.perspectiveTransform(old40[None], H)[0]
    half = .0282 / 2
    marker40_obj = np.asarray([[-half, half, 0], [half, half, 0],
                               [half, -half, 0], [-half, -half, 0]], np.float32)
    R_new_hand = R_a1 @ R_a0.T @ R_h0
    t_new_hand = t_a1 + R_a1 @ R_a0.T @ (t_h0 - t_a0)
    bridge_camera = (R_new_hand @ marker40_obj.T).T + t_new_hand
    rigid40 = cv.projectPoints(bridge_camera, np.zeros(3), np.zeros(3), K, dist)[0].reshape(4, 2)
    view = new.copy()
    cv.polylines(view, [np.int32(image40)], True, (0, 255, 0), 3)
    cv.polylines(view, [np.int32(rigid40)], True, (0, 0, 255), 3)
    cv.putText(view, "GREEN image features | RED old ID41 bridge", (20, 700),
               cv.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2)
    (ROOT / "sac_cross_session_image_audit_overlay.png").write_bytes(cv.imencode(".png", view)[1].tobytes())
    report = {"matches": len(matches), "inliers": int(inliers.sum()),
              "inlier_fraction": float(inliers.mean()),
              "image_predicted_id40_px": image40.tolist(),
              "rigid_bridge_predicted_id40_px": rigid40.tolist(),
              "mean_corner_disagreement_px": float(np.mean(np.linalg.norm(image40 - rigid40, axis=1))),
              "caveat": "2D feature homography across cameras/views is only qualitative; palm may not be planar and cube masks many matches."}
    (ROOT / "sac_cross_session_image_audit_result.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if not k.endswith("px")}, indent=2))
    print(f"Mean corner disagreement: {report['mean_corner_disagreement_px']:.1f}px")


if __name__ == "__main__":
    main()
