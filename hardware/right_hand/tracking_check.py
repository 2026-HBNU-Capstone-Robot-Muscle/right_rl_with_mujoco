"""Measure simultaneous cube/hand marker visibility; never accesses motors."""

import json
import time
from pathlib import Path

import cv2

from camera_calibrate import open_camera


ROOT = Path(__file__).resolve().parent
DURATION = 10.0


def main():
    cap, _ = open_camera(1)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    counts = {"frames": 0, "hand": 0, "cube": 0, "both": 0, "two_cube": 0}
    per_id = {str(i): 0 for i in range(6)}
    started = None
    print("Hold the marked cube still near the stationary hand; show 2 cube faces and ID 40.")
    print("SPACE starts a 10-second visibility measurement; Q exits.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            visible = set() if ids is None else {int(x) for x in ids.ravel()}
            cube = visible.intersection(range(6))
            hand = 40 in visible
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            if started is not None:
                counts["frames"] += 1
                counts["hand"] += int(hand)
                counts["cube"] += int(bool(cube))
                counts["both"] += int(hand and bool(cube))
                counts["two_cube"] += int(hand and len(cube) >= 2)
                for marker_id in cube:
                    per_id[str(marker_id)] += 1
                remaining = max(0.0, DURATION - (time.monotonic() - started))
                status = f"{remaining:.1f}s  cube {sorted(cube)}  hand {'YES' if hand else 'NO'}"
            else:
                status = f"cube {sorted(cube)}  hand {'YES' if hand else 'NO'}  SPACE start"
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 0) if hand and cube else (0, 0, 255), 2)
            cv2.imshow("LEAP hand + cube visibility - webcam 1", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == 32 and started is None:
                started = time.monotonic()
                print("Measuring for 10 seconds...", flush=True)
            if started is not None and time.monotonic() - started >= DURATION:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if counts["frames"]:
        result = {"camera": 1, "duration_s": DURATION, "counts": counts,
                  "per_id": per_id,
                  "percent": {key: round(100 * counts[key] / counts["frames"], 1)
                              for key in ("hand", "cube", "both", "two_cube")}}
        output = ROOT / "tracking_check_result.json"
        output.write_text(json.dumps(result, indent=2))
        print(f"Saved: {output}")
        print("Detection percentages:", result["percent"])
        print("Cube ID frame counts:", per_id)


if __name__ == "__main__":
    main()
