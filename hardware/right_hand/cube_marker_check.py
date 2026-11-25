"""Check printed cube markers with external webcam 1; never accesses hand motors."""

from pathlib import Path

import cv2

from camera_calibrate import open_camera


def main():
    cap, frame = open_camera(1)
    print(f"External webcam 1: {frame.shape[1]}x{frame.shape[0]}")
    print("Turn the cube to show each center marker. S saves the current raw frame; Q quits.")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    seen = set()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            raw = frame.copy()
            corners, ids, rejected = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            visible = set() if ids is None else set(int(x) for x in ids.ravel())
            new = visible.intersection(range(6)) - seen
            if new:
                seen.update(new)
                print(f"New IDs: {sorted(new)}; seen: {sorted(seen)}", flush=True)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            status = f"Seen {sorted(seen)}  Visible {sorted(visible)}  S save  Q quit"
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 1)
            cv2.imshow("LEAP cube marker check - webcam 1", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                output = Path(__file__).resolve().parent / "id4_webcam.png"
                success, encoded = cv2.imencode(".png", raw)
                if success:
                    output.write_bytes(encoded.tobytes())
                    print(f"Saved {output}; visible IDs {sorted(visible)}; rejected candidates {len(rejected)}", flush=True)
            if key in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    print(f"Cube IDs seen: {sorted(seen)} / [0, 1, 2, 3, 4, 5]")
    if seen != set(range(6)):
        print("Some markers were not detected; check lighting/focus/white border.")


if __name__ == "__main__":
    main()
