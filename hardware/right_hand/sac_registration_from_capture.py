"""Extract one simultaneous ID40 + two-face cube pose from a capture-only run."""

import argparse
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", type=Path, help="Existing sequence_vision folder for offline validation")
    parser.add_argument("--image", type=Path, help="Existing image for offline parser validation")
    args = parser.parse_args()
    if args.folder is None and args.image is None:
        run = json.loads((ROOT / "leap_registration_capture_result.json").read_text(encoding="utf-8"))
        if run.get("status") != "placement_capture_complete" or not run.get("all_torque_off_verified"):
            raise RuntimeError("Capture was not completed with verified torque OFF")
        folder = Path(run["vision_recording"]["folder"])
    else:
        folder = args.folder if args.folder is not None else args.image.parent
    path = args.image if args.image is not None else folder / "placement_at_space.png"
    frame = cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
    with np.load(ROOT / "camera1_720.npz") as calibration:
        K, distortion = calibration["K"].copy(), calibration["dist"].copy()
    rotations = tuple(json.loads((ROOT / "cube_face_rot_result.json").read_text(encoding="utf-8"))["face_rot"])
    points, ids = cube_board.object_points(cube_mm=54.5, marker_mm=10.6, layout="centers", face_rot=rotations)
    dictionary, _, _ = track_cube.build(marker_mm=10.6, grid=1)
    board = cv2.aruco.Board(points, dictionary, ids)
    detector = MarkerDetector(dictionary, ids)
    registration = candidate()
    track_cube.PALM_IN_MARKER = np.asarray(registration["grasp_site_in_marker_m_candidate"])
    track_cube.HAND_TO_WORLD = np.asarray(registration["hand_to_world_candidate"])
    pose = track_cube.step(frame, board, detector, dictionary, K, distortion)
    found = detector.stats.get("ids", [])
    if pose is None or 40 not in found or pose.get("n_cube_markers", 0) < 2 or "t_cube" not in pose or "t_hand" not in pose:
        raise RuntimeError(f"Same-frame calibration requires ID40 and two cube IDs; found {found}")
    cube_in_marker = np.asarray(pose["R_hand"]).T @ (np.asarray(pose["t_cube"]) - np.asarray(pose["t_hand"]))
    cube_in_world = np.array([0.11, 0.0, 0.03]) - np.asarray(pose["palm_minus_cube"])
    report = {
        "source_image": str(path),
        "detected_ids": found,
        "cube_markers": pose["n_cube_markers"],
        "reprojection_px": float(pose["reproj"]),
        "cube_center_in_marker_m": cube_in_marker.tolist(),
        "cube_center_in_sim_world_candidate_m": cube_in_world.tolist(),
        "cube_quat_world_candidate": np.asarray(pose["cube_quat_world"]).tolist(),
        "registration_status": registration["status"],
        "note": "Same-frame camera measurement; marker-to-MuJoCo mounting transform remains a candidate.",
    }
    if args.folder is None and args.image is None:
        (ROOT / "sac_registration_capture_pose.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
