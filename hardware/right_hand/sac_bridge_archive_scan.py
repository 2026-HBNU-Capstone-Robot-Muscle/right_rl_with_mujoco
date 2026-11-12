"""Find saved empty frames with both ID40 and ID41 before asking for a new photo."""

import json
from pathlib import Path

import cv2 as cv
import numpy as np

from sac_registration_from_anchor import detect, marker_pose


ROOT = Path(__file__).resolve().parent
NAMES = ("anchor_empty_stage0.png", "anchor_empty_preflight.png",
         "preflight_not_ready.png", "configure_off_not_ready.png",
         "enable_hold_not_ready.png", "stage_0_not_ready.png")


def main():
    with np.load(ROOT / "camera1_720.npz") as f:
        K, dist = f["K"].copy(), f["dist"].copy()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    paths = []
    for date in ("20261006", "20261007", "20261008", "20261009", "20261010"):
        for folder in ROOT.glob(f"sequence_vision_{date}_*"):
            paths.extend(folder / name for name in NAMES if (folder / name).exists())
    paths.append(ROOT / "leap_palm_photo_20261008_201116_472718.png")
    rows = []
    for path in paths:
        try:
            _, corners, ids, _ = detect(path, dictionary)
            if 40 not in ids or 41 not in ids:
                continue
            R_h, t_h, err_h = marker_pose(corners, ids, 40, .0282, K, dist)
            R_a, t_a, err_a = marker_pose(corners, ids, 41, .030, K, dist)
            rows.append({"image": str(path.relative_to(ROOT)),
                         "hand_center_in_anchor_m": (R_a.T @ (t_h - t_a)).tolist(),
                         "hand_rotation_in_anchor": (R_a.T @ R_h).tolist(),
                         "reprojection_px": [err_h, err_a]})
        except Exception as exc:
            rows.append({"image": str(path.relative_to(ROOT)), "error": str(exc)})
    ref = next((r for r in rows if r["image"].startswith(
        "sequence_vision_20261006_203648_332236\\anchor_empty_stage0")), None)
    if ref is None:
        ref = next((r for r in rows if "hand_center_in_anchor_m" in r), None)
    if ref is not None:
        p0 = np.asarray(ref["hand_center_in_anchor_m"])
        R0 = np.asarray(ref["hand_rotation_in_anchor"])
        for row in rows:
            if "hand_center_in_anchor_m" not in row:
                continue
            p = np.asarray(row["hand_center_in_anchor_m"])
            R = np.asarray(row["hand_rotation_in_anchor"])
            row["shift_from_reference_mm"] = float(1000 * np.linalg.norm(p - p0))
            row["angle_from_reference_deg"] = float(np.degrees(np.arccos(
                np.clip((np.trace(R0.T @ R) - 1) / 2, -1, 1))))
    result = {"source_count": len(paths), "both_detected": sum("hand_center_in_anchor_m" in r for r in rows),
              "reference_image": None if ref is None else ref["image"], "rows": rows,
              "hardware_authorized": False}
    (ROOT / "sac_bridge_archive_scan_result.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"source_count": result["source_count"], "both_detected": result["both_detected"],
                      "reference_image": result["reference_image"],
                      "found": [{k: v for k, v in row.items() if k in
                                  ("image", "shift_from_reference_mm", "angle_from_reference_deg")}
                                for row in rows if "hand_center_in_anchor_m" in row]}, indent=2))


if __name__ == "__main__":
    main()
