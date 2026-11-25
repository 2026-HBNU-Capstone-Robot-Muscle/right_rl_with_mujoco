"""Save one webcam photo of the stationary LEAP hand; no motor access."""

from pathlib import Path

import cv2

from camera_calibrate import open_camera


def main():
    cap, _ = open_camera(1)
    output = Path(__file__).resolve().parent / "hand_photo.png"
    print("Show the full stationary hand and its palm mount. S saves a photo; Q quits.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            raw = frame.copy()
            cv2.putText(frame, "S save hand photo  Q quit", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("LEAP hand photo - webcam 1", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                success, encoded = cv2.imencode(".png", raw)
                if success:
                    output.write_bytes(encoded.tobytes())
                    print(f"Saved: {output}", flush=True)
            if key in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
