"""Convert archived cuboid-rim PnP poses through provisional ID40/ID41 bridge."""

import json
import sys
import argparse
from pathlib import Path

import cv2 as cv
import numpy as np

from hand_registration_geometry import candidate
from sac_registration_from_anchor import detect, marker_pose


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "work" / "Leap_Test" / "vision" / "rubik_center"))
import track_cube


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge-image", type=Path,
                        default=ROOT / "sequence_vision_20261006_203648_332236" / "anchor_empty_stage0.png")
    parser.add_argument("--output-file", default="sac_outer_face_registration_191152_result.json")
    args = parser.parse_args()
    old_image = args.bridge_image if args.bridge_image.is_absolute() else ROOT / args.bridge_image
    with np.load(ROOT / "camera1_720.npz") as f:
        K, dist = f["K"].copy(), f["dist"].copy()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    new_image = ROOT / "sequence_vision_20261008_191152_096085" / "placement_at_space.png"
    _, old_corners, old_ids, _ = detect(old_image, dictionary)
    R_h0, t_h0, _ = marker_pose(old_corners, old_ids, 40, .0282, K, dist)
    R_a0, t_a0, _ = marker_pose(old_corners, old_ids, 41, .030, K, dist)
    _, new_corners, new_ids, _ = detect(new_image, dictionary)
    R_a1, t_a1, _ = marker_pose(new_corners, new_ids, 41, .030, K, dist)
    R_anchor_hand = R_a0.T @ R_h0
    t_anchor_hand = R_a0.T @ (t_h0 - t_a0)
    reg = candidate()
    R_world_hand = np.asarray(reg["hand_to_world_candidate"])
    grasp_in_hand = np.asarray(reg["grasp_site_in_marker_m_candidate"])
    source = json.loads((ROOT / "sac_cube_outer_face_191152_result.json").read_text(encoding="utf-8"))
    old = json.loads((ROOT / "sac_cross_session_191152_registration_result.json").read_text(encoding="utf-8"))
    rows = []
    for item in source["cases"]:
        R_cube_cam = np.asarray(item["rotation_cube_to_camera"])
        t_cube_cam = np.asarray(item["center_camera_m"])
        cube_anchor = R_a1.T @ (t_cube_cam - t_a1)
        R_cube_anchor = R_a1.T @ R_cube_cam
        cube_hand = R_anchor_hand.T @ (cube_anchor - t_anchor_hand)
        R_cube_hand = R_anchor_hand.T @ R_cube_anchor
        cube_world = np.array([.11, 0, .03]) + R_world_hand @ (cube_hand - grasp_in_hand)
        cube_quat = track_cube.mat_to_quat(R_world_hand @ R_cube_hand)
        rows.append({
            "rim_pixel_scale": item["rim_pixel_scale"],
            "cube_center_in_hand_marker_m": cube_hand.tolist(),
            "cube_center_in_sim_world_candidate_m": cube_world.tolist(),
            "cube_quat_world_candidate": np.asarray(cube_quat).tolist(),
            "shift_from_single_marker_sim_mm": float(1000 * np.linalg.norm(
                cube_world - np.asarray(old["cube_center_in_sim_world_candidate_m"]))),
            "rim_reprojection_px": item["rim_reprojection_px"],
            "marker_reprojection_px": item["marker_reprojection_px"],
        })
    result = {"status": "offline_outer_face_registration_diagnostic", "hardware_authorized": False,
              "source": str(new_image), "bridge_image": str(old_image), "cases": rows,
              "caveat": "Rim hand-read and ID40/ID41 bridge crosses sessions; not a verified physical reset."}
    (ROOT / args.output_file).write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"cases": [{k: v for k, v in r.items() if k not in
                                   ("cube_quat_world_candidate", "cube_center_in_hand_marker_m")}
                                  for r in rows]}, indent=2))


if __name__ == "__main__":
    main()
