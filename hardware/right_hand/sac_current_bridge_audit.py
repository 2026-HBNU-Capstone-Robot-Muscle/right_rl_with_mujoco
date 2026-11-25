"""Offline comparison of newly captured rigid ID40/ID41 bridge to archived views."""

import json
from pathlib import Path

import cv2 as cv
import numpy as np

from sac_registration_from_anchor import detect, marker_pose


ROOT = Path(__file__).resolve().parent
IMAGES = {
    "oct6": ROOT / "sequence_vision_20261006_203648_332236" / "anchor_empty_stage0.png",
    "oct7": ROOT / "sequence_vision_20261007_011324_187249" / "stage_0_not_ready.png",
    "oct9_first": ROOT / "leap_marker_bridge_20261009_202219_278541.png",
    "current": ROOT / "leap_marker_bridge_20261009_210136_395232.png",
}


def main():
    with np.load(ROOT / "camera1_720.npz") as f:
        K, dist = f["K"].copy(), f["dist"].copy()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    poses = {}
    for key, path in IMAGES.items():
        _, corners, ids, _ = detect(path, dictionary)
        Rh, th, eh = marker_pose(corners, ids, 40, .0282, K, dist)
        Ra, ta, ea = marker_pose(corners, ids, 41, .030, K, dist)
        poses[key] = {"image": str(path), "position_m": (Ra.T @ (th - ta)).tolist(),
                      "rotation": (Ra.T @ Rh).tolist(), "reprojection_px": [eh, ea]}
    cur = poses["current"]
    for key in ("oct6", "oct7", "oct9_first"):
        old = poses[key]
        d = np.asarray(cur["position_m"]) - old["position_m"]
        R = np.asarray(old["rotation"]).T @ np.asarray(cur["rotation"])
        old["current_delta_mm"] = (1000 * d).tolist()
        old["current_translation_distance_mm"] = float(1000 * np.linalg.norm(d))
        old["current_rotation_distance_deg"] = float(np.degrees(np.arccos(
            np.clip((np.trace(R) - 1) / 2, -1, 1))))
    (ROOT / "sac_current_bridge_audit_result.json").write_text(
        json.dumps(poses, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(poses, indent=2))


if __name__ == "__main__":
    main()
