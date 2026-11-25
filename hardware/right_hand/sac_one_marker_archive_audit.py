"""Offline comparison of one-face and multi-face cube PnP on saved photos.

This script only reads archived PNGs; it never opens a camera or motor port.
The multi-face fit is a reference for consistency, not ground truth.
"""

import json
import hashlib
import sys
from pathlib import Path

import cv2 as cv
import numpy as np


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "work/Leap_Test/vision/rubik_center"))
import cube_board


def fit(ids, corners, object_points, K, distortion):
    obj = np.concatenate([object_points[i] for i in ids]).astype(np.float32)
    img = np.concatenate([corners[i] for i in ids]).astype(np.float32)
    ok, rvec, tvec = cv.solvePnP(obj, img, K, distortion,
                                  flags=cv.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    projection, _ = cv.projectPoints(obj, rvec, tvec, K, distortion)
    error = float(np.linalg.norm(projection.reshape(-1, 2) - img, axis=1).mean())
    return cv.Rodrigues(rvec)[0], tvec.ravel(), error


def main():
    with np.load(ROOT / "camera1_720.npz") as calibration:
        K = calibration["K"].copy()
        distortion = calibration["dist"].copy()
    rotations = json.loads((ROOT / "cube_face_rot_result.json").read_text(encoding="utf-8"))["face_rot"]
    points, _ = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                          layout="centers", face_rot=tuple(rotations))
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    detector = cv.aruco.ArucoDetector(dictionary, cv.aruco.DetectorParameters())
    photos = sorted(set(ROOT.glob("vision_sweep_20261006_011011_994373/*.png")) |
                    set(ROOT.glob("vision_check_20261004_*/*.png")) |
                    set(ROOT.glob("sequence_vision_20261009_*/*.png")) |
                    set(ROOT.glob("leap_two_face_preview_*.png")))
    rows = []
    seen_images = set()
    for photo in photos:
        data = photo.read_bytes()
        digest = hashlib.sha256(data).digest()
        if digest in seen_images:
            continue
        seen_images.add(digest)
        # OpenCV's Windows filename decoder can reject Korean user-profile paths.
        frame = cv.imdecode(np.frombuffer(data, dtype=np.uint8), cv.IMREAD_COLOR)
        if frame is None:
            continue
        detected, ids, _ = detector.detectMarkers(cv.cvtColor(frame, cv.COLOR_BGR2GRAY))
        if ids is None:
            continue
        cube = {int(i): np.asarray(c, np.float32).reshape(4, 2)
                for c, i in zip(detected, ids.ravel()) if int(i) in range(6)}
        if len(cube) < 2:
            continue
        full = fit(sorted(cube), cube, points, K, distortion)
        if full is None or not np.isfinite(full[2]) or full[2] > 3:
            continue
        R_full, t_full, full_error = full
        for marker_id in sorted(cube):
            single = fit([marker_id], cube, points, K, distortion)
            if single is None:
                continue
            R_single, t_single, single_error = single
            rotation_gap = float(np.degrees(np.arccos(np.clip(
                (np.trace(R_single.T @ R_full) - 1) / 2, -1, 1))))
            rows.append({"image": str(photo.relative_to(ROOT)), "marker_id": marker_id,
                         "reference_ids": sorted(cube),
                         "multi_face_center_camera_m": t_full.tolist(),
                         "multi_face_reprojection_px": full_error,
                         "single_face_reprojection_px": single_error,
                         "center_difference_mm": float(np.linalg.norm(t_single-t_full)*1000),
                         "rotation_difference_deg": rotation_gap})
    def summary(values):
        return {"count": len(values), "median": float(np.median(values)),
                "p90": float(np.percentile(values, 90)), "max": float(np.max(values))} if values else None
    output = {"status": "offline_saved_photo_diagnostic", "hardware_authorized": False,
              "comparison": "one marker against two-or-more-marker fit on the same saved frame",
              "pairs": len(rows),
              "center_difference_mm": summary([r["center_difference_mm"] for r in rows]),
              "rotation_difference_deg": summary([r["rotation_difference_deg"] for r in rows]),
              "id5_center_difference_mm": summary([r["center_difference_mm"] for r in rows
                                                    if r["marker_id"] == 5]),
              "rows": rows,
              "caveat": "Multi-face PnP is a consistency reference, not a physical ground-truth pose; archived photos sample only a few views."}
    (ROOT / "sac_one_marker_archive_audit_result.json").write_text(
        json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in output.items() if k != "rows"}, indent=2))


if __name__ == "__main__":
    main()
