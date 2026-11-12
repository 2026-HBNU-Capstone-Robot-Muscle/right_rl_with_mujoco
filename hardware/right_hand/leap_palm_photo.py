"""Capture one raw overhead photo from the named Streaming Webcam; no motor access."""

import datetime
import json
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def main():
    import cv2 as cv
    from camera_stream import CameraStream

    report = {"motor_port_opened": False, "motor_commands_sent": 0}
    stream = None
    title = "LEAP palm photo - S saves, Q quits - NO MOTORS"
    try:
        stream = CameraStream(report)
        print("MOTOR-FREE PHOTO. Show ID40, the palm contact spot, and a ruler in one frame.", flush=True)
        print("Press S in the CAMERA window to save one photo; Q or Esc quits.", flush=True)
        cv.namedWindow(title, cv.WINDOW_NORMAL)
        cv.resizeWindow(title, 1100, 620)
        last_frame = time.monotonic()
        latest = None
        latest_stamp = None
        while True:
            packet = stream.read_packet()
            if packet is not None:
                latest_stamp, latest = packet
                if time.monotonic() - latest_stamp <= 0.2:
                    last_frame = time.monotonic()
                    preview = latest.copy()
                    cv.putText(preview, "ID40 + palm contact spot + ruler | S SAVE | Q QUIT",
                               (12, 32), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv.imshow(title, preview)
            if time.monotonic() - last_frame > 3:
                raise RuntimeError("No fresh webcam frame for 3s")
            key = cv.waitKey(20) & 255
            if key in (ord("q"), ord("Q"), 27):
                print("Photo cancelled; no image saved.", flush=True)
                return 0
            if key in (ord("s"), ord("S")):
                if latest is None or time.monotonic() - latest_stamp > 0.2:
                    print("Wait for a fresh camera frame, then press S again.", flush=True)
                    continue
                tag = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                output = ROOT / f"leap_palm_photo_{tag}.png"
                ok, encoded = cv.imencode(".png", latest)
                if not ok:
                    raise RuntimeError("PNG encoding failed")
                output.write_bytes(encoded.tobytes())
                report.update(saved_image=str(output), captured_at=datetime.datetime.now().astimezone().isoformat())
                (ROOT / "leap_palm_photo_result.json").write_text(
                    json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
                print(f"Saved: {output}", flush=True)
                return 0
    finally:
        if stream is not None:
            stream.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
