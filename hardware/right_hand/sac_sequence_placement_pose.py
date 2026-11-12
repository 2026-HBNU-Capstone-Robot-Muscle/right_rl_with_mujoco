"""Estimate stage-0 cube pose from a placement frame and later visible ID40.

The camera must have stayed fixed across both frames; a one-marker cube pose
can be ambiguous, so this remains an offline diagnostic rather than a label.
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "work" / "Leap_Test" / "vision" / "rubik_center"))
import cube_board
import track_cube
from marker_detector import MarkerDetector

from hand_registration_geometry import candidate


def load_image(path):
    return cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)


def main():
    with np.load(ROOT / "camera1_720.npz") as calibration:
        K, distortion = calibration["K"].copy(), calibration["dist"].copy()
    face_rot = tuple(json.loads((ROOT / "cube_face_rot_result.json").read_text(encoding="utf-8"))["face_rot"])
    points, ids = cube_board.object_points(cube_mm=54.5, marker_mm=10.6, layout="centers", face_rot=face_rot)
    dictionary, _, _ = track_cube.build(marker_mm=10.6, grid=1)
    board = cv2.aruco.Board(points, dictionary, ids)
    registration = candidate()
    track_cube.PALM_IN_MARKER = np.asarray(registration["grasp_site_in_marker_m_candidate"])
    track_cube.HAND_TO_WORLD = np.asarray(registration["hand_to_world_candidate"])
    result = []
    for folder_name in ("sequence_vision_20261006_042959_779594", "sequence_vision_20261006_043709_528624"):
        folder = ROOT / folder_name
        ref_path = folder / "stage_4_eligible.png"
        placement_path = folder / "placement_at_space.png"
        if not placement_path.exists():
            placement_path = folder / "stage_1_not_ready.png"
        ref_detector = MarkerDetector(dictionary, ids)
        placement_detector = MarkerDetector(dictionary, ids)
        ref = track_cube.step(load_image(ref_path), board, ref_detector, dictionary, K, distortion)
        placement = track_cube.step(load_image(placement_path), board, placement_detector, dictionary, K, distortion)
        session = json.loads((folder / "result.json").read_text(encoding="utf-8"))
        placed_at = session.get("placement_image_captured_at_unix_s")
        nearest_hand_frame = None
        if placed_at is not None:
            hand_frames = [f for f in session["frames"]
                           if f["captured_at_unix_s"] <= placed_at
                           and f.get("pose") and "R_hand" in f["pose"]]
            if hand_frames:
                nearest_hand_frame = max(hand_frames, key=lambda f: f["captured_at_unix_s"])
                ref = nearest_hand_frame["pose"]
        row = {
            "folder": folder_name,
            "hand_reference": ref_path.name,
            "placement_image": placement_path.name,
            "hand_ids": ref_detector.stats.get("ids", []),
            "placement_ids": placement_detector.stats.get("ids", []),
            "placement_cube_markers": None if placement is None else placement.get("n_cube_markers"),
            "placement_reprojection_px": None if placement is None else placement.get("reproj"),
            "nearest_hand_phase": None if nearest_hand_frame is None else nearest_hand_frame["phase"],
            "hand_reference_age_s": None if nearest_hand_frame is None else placed_at - nearest_hand_frame["captured_at_unix_s"],
        }
        if ref and placement and "R_hand" in ref and "t_cube" in placement:
            cube_in_marker = np.asarray(ref["R_hand"]).T @ (
                np.asarray(placement["t_cube"]) - np.asarray(ref["t_hand"])
            )
            R_world_marker = np.asarray(registration["hand_to_world_candidate"])
            grasp_in_marker = np.asarray(registration["grasp_site_in_marker_m_candidate"])
            cube_in_world = np.array([0.11, 0.0, 0.03]) + R_world_marker @ (cube_in_marker - grasp_in_marker)
            row["cube_center_in_marker_m"] = cube_in_marker.tolist()
            row["cube_center_in_sim_world_candidate_m"] = cube_in_world.tolist()
            row["cube_quat_world_candidate"] = track_cube.mat_to_quat(
                R_world_marker @ np.asarray(ref["R_hand"]).T @ np.asarray(placement["R_cube"])
            ).tolist()
        result.append(row)
    (ROOT / "sac_sequence_placement_pose_result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
