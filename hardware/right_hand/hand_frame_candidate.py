"""Measure cube pose relative to the hand marker, without motor access.

An arbitrary cube placement cannot identify the marker-to-robot transform.
This script only saves relative observations, never robot calibration values.
"""

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from camera_calibrate import open_camera


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "work" / "Leap_Test" / "vision" / "rubik_center"))
import cube_board as cb  # noqa: E402
import track_cube as tc  # noqa: E402


def nearest_rotation(matrices):
    u, _, vt = np.linalg.svd(np.mean(matrices, axis=0))
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    return result


def summarize(rotations, translations, reprojections, total):
    report = {
        "status": "no_usable_pose" if not rotations else "relative_pose_only",
        "robot_frame_calibrated": False,
        "camera": 1, "frames": total, "usable_two_marker_frames": len(rotations),
        "note": "Robot reference-point location and axes require independent physical registration.",
    }
    if not rotations:
        return report
    R_h_cube = nearest_rotation(np.asarray(rotations))
    t_h_cube = np.median(np.asarray(translations), axis=0)
    position_deviation_mm = np.linalg.norm(np.asarray(translations) - t_h_cube, axis=1) * 1000
    rotation_deviation_deg = np.degrees(np.arccos(np.clip(
        (np.einsum("ij,nij->n", R_h_cube, np.asarray(rotations)) - 1) / 2, -1, 1)))
    report.update({
        "cube_to_hand_marker_rotation": R_h_cube.tolist(),
        "cube_center_in_hand_marker_m": t_h_cube.tolist(),
        "relative_position_p95_mm": round(float(np.percentile(position_deviation_mm, 95)), 2),
        "relative_rotation_p95_deg": round(float(np.percentile(rotation_deviation_deg, 95)), 2),
        "cube_reprojection_mean_px": round(float(np.mean(reprojections)), 2),
    })
    return report


def main():
    calibration = np.load(ROOT / "camera1.npz")
    face_rot = tuple(json.loads((ROOT / "cube_face_rot_result.json").read_text())["face_rot"])
    points, ids = cb.object_points(cube_mm=54.5, marker_mm=10.6,
                                    layout="centers", face_rot=face_rot)
    dictionary, _, detector = tc.build(marker_mm=10.6, grid=1)
    board = cv2.aruco.Board(points, dictionary, ids)
    cap, first = open_camera(1)
    if first.shape[:2] != (int(calibration["height"]), int(calibration["width"])):
        cap.release()
        raise SystemExit("Camera resolution differs from camera1.npz")
    print("Relative-pose diagnostics only; this does NOT calibrate the robot frame.")
    print("Any fixed cube orientation is OK. Show ID 40 and at least 2 cube markers.")
    print("Keep both objects still. SPACE starts 10 seconds; Q quits.")
    rotations, translations, reprojections = [], [], []
    total = 0
    started = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            result = tc.step(frame, board, detector, dictionary,
                             calibration["K"], calibration["dist"])
            good = (result is not None and "R_hand" in result and
                    "R_cube" in result and result["n_cube_markers"] >= 2)
            if started is not None:
                total += 1
                if good:
                    R_hand = result["R_hand"]
                    rotations.append(R_hand.T @ result["R_cube"])
                    translations.append(R_hand.T @ (result["t_cube"] - result["t_hand"]))
                    reprojections.append(float(result["reproj"]))
                remaining = max(0.0, 10.0 - (time.monotonic() - started))
                status = f"{remaining:.1f}s  2+ cube + hand {'YES' if good else 'NO'}  {len(rotations)}/{total}"
            else:
                status = f"2+ cube + hand {'YES' if good else 'NO'}  SPACE start"
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0) if good else (0, 0, 255), 1)
            cv2.imshow("LEAP hand-frame candidate - no motors", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == 32 and started is None:
                started = time.monotonic()
                print("Measuring 10 seconds...", flush=True)
            if started is not None and time.monotonic() - started >= 10.0:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    samples_file = ROOT / "hand_frame_observations.json"
    samples_file.write_text(json.dumps([
        {"cube_to_hand_marker_rotation": r.tolist(),
         "cube_center_in_hand_marker_m": t.tolist(), "reprojection_px": e}
        for r, t, e in zip(rotations, translations, reprojections)
    ], indent=2), encoding="utf-8")
    report = summarize(rotations, translations, reprojections, total)
    output = ROOT / "hand_frame_candidate_result.json"
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"Diagnostics saved (NOT robot calibration): {output}")
    print(f"Usable frames: {len(rotations)}/{total}; position p95: {report.get('relative_position_p95_mm')} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
