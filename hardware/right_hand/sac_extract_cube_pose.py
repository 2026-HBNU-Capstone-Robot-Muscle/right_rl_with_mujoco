"""Recover hand-relative cube poses from saved frames; never opens a camera/motor."""

import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent / "work" / "Leap_Test"
sys.path.insert(0, str(PROJECT / "vision" / "rubik_center"))
import cube_board
import track_cube
from hand_registration_geometry import candidate
from marker_detector import MarkerDetector


def main():
    registration = candidate()
    track_cube.PALM_IN_MARKER = np.asarray(registration["grasp_site_in_marker_m_candidate"])
    track_cube.HAND_TO_WORLD = np.asarray(registration["hand_to_world_candidate"])
    with np.load(ROOT / "camera1_720.npz") as calibration:
        intrinsic = calibration["K"].copy()
        distortion = calibration["dist"].copy()
    rotations = json.loads((ROOT / "cube_face_rot_result.json").read_text(encoding="utf-8"))["face_rot"]
    points, ids = cube_board.object_points(
        cube_mm=54.5, marker_mm=10.6, layout="centers", face_rot=tuple(rotations)
    )
    dictionary, _, _ = track_cube.build(marker_mm=10.6, grid=1)
    board = cv2.aruco.Board(points, dictionary, ids)
    detector = MarkerDetector(dictionary, ids)
    folder = ROOT / "vision_sweep_20261006_011011_994373"
    rows = []
    for path in sorted(folder.glob("*s_eligible.png")):
        image = cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
        pose = track_cube.step(image, board, detector, dictionary, intrinsic, distortion)
        rows.append({
            "image": path.name,
            "ids": detector.stats.get("ids", []),
            "markers": None if pose is None else pose.get("n_cube_markers"),
            "palm_minus_cube_m": None if pose is None else np.asarray(
                pose.get("palm_minus_cube", []), dtype=float
            ).tolist(),
            "reprojection_px": None if pose is None else pose.get("reproj"),
            "cube_quat_world": None if pose is None else np.asarray(
                pose.get("cube_quat_world", []), dtype=float
            ).tolist(),
            "face_up": None if pose is None else pose.get("face_up"),
            "cube_minus_hand_marker_m": (
                None if pose is None or "R_hand" not in pose or "t_cube" not in pose
                else (np.asarray(pose["R_hand"]).T @ (
                    np.asarray(pose["t_cube"]) - np.asarray(pose["t_hand"])
                )).tolist()
            ),
        })
    output = ROOT / "sac_success_cube_poses.json"
    output.write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
