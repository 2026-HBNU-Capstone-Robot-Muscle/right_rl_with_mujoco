"""Measure per-face ArUco rotations on the marked cube; no hand motor access."""

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from camera_calibrate import open_camera


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "work" / "Leap_Test" / "vision" / "rubik_center"))
import calibrate_face_rot  # noqa: E402


def main():
    camera_file = ROOT / "camera1.npz"
    if not camera_file.exists():
        raise SystemExit("camera1.npz is missing; calibrate webcam 1 first")
    z = np.load(camera_file)
    cap, frame = open_camera(1)
    if frame.shape[1] != int(z["width"]) or frame.shape[0] != int(z["height"]):
        cap.release()
        raise SystemExit("Camera resolution differs from camera1.npz")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    samples, covered = [], set()
    last_good = None
    last_capture_time = -1.0
    print("Show 2 or 3 cube faces at once. Change cube angle between captures.")
    print("SPACE captures when >=2 cube IDs are visible. Collect >=20 sharp views covering IDs 0-5; Q solves.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            cube = [] if ids is None else [
                (int(i), c.reshape(4, 2).copy())
                for i, c in zip(ids.ravel(), corners) if 0 <= int(i) <= 5
            ]
            visible = {i for i, _ in cube}
            now = time.monotonic()
            if len(visible) >= 2:
                last_good = (now, cube)
            ready = last_good is not None and now - last_good[0] <= 0.3
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            status = f"{'READY' if ready else 'WAIT'}  views {len(samples)}  now {sorted(visible)}  covered {sorted(covered)}"
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0) if ready else (0, 0, 255), 1)
            cv2.putText(frame, "SPACE capture  Q solve", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("LEAP cube face orientation - webcam 1", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == 32:
                if not ready or last_good[0] <= last_capture_time:
                    print("Wait for a new 2-marker view", flush=True)
                    continue
                captured = last_good[1]
                captured_ids = {i for i, _ in captured}
                samples.append((np.array([i for i, _ in captured], np.int32),
                                np.array([c for _, c in captured], np.float32)))
                covered.update(captured_ids)
                last_capture_time = last_good[0]
                print(f"Captured {len(samples)}: {sorted(captured_ids)}; covered {sorted(covered)}", flush=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if len(samples) < 20 or covered != set(range(6)):
        print(f"Not enough coverage: {len(samples)} views, IDs {sorted(covered)}; nothing saved")
        return 1
    observations_file = ROOT / "cube_face_rot_observations.json"
    observations_file.write_text(json.dumps([
        {"ids": ids.tolist(), "corners": corners.tolist()}
        for ids, corners in samples
    ]))
    print(f"Numerical observations saved: {observations_file}", flush=True)
    print("Solving 4096 possible face rotations; this may take a while...", flush=True)
    rotations, error = calibrate_face_rot.solve(
        samples, z["K"], z["dist"], marker_mm=10.6, cube_mm=54.5)
    output = ROOT / "cube_face_rot_result.json"
    output.write_text(json.dumps({"face_rot": list(rotations), "reprojection_px": error,
                                  "views": len(samples), "camera": 1}, indent=2))
    print(f"Saved: {output}")
    print(f"Use --face_rot {','.join(str(x) for x in rotations)} after reviewing the error")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
