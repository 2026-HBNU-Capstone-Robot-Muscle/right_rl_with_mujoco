"""Camera-only photo requiring both fixed hand markers ID40 and ID41."""

import datetime
import json
import time
from pathlib import Path

import cv2 as cv

from camera_stream import CameraStream


ROOT = Path(__file__).resolve().parent


def main():
    report = {"motor_port_opened": False, "motor_commands_sent": 0,
              "required_markers": [40, 41]}
    stream = None
    title = "LEAP marker bridge | IDs 40+41 | S saves | Q quits | NO MOTORS"
    detector = cv.aruco.ArucoDetector(
        cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
        cv.aruco.DetectorParameters())
    try:
        stream = CameraStream(report)
        print("MOTOR-FREE. Remove cube. Aim webcam until BOTH palm ID40 and mount ID41 appear.", flush=True)
        print("Press S in CAMERA window when READY; Q/Esc quits. No hand motion.", flush=True)
        cv.namedWindow(title, cv.WINDOW_NORMAL)
        cv.resizeWindow(title, 1100, 620)
        latest = None
        latest_stamp = None
        latest_ids = []
        latest_corners = []
        ready_streak = 0
        last_frame = time.monotonic()
        last_prompt = 0.0
        while True:
            packet = stream.read_packet()
            if packet is not None:
                latest_stamp, latest = packet
                if time.monotonic() - latest_stamp <= .2:
                    last_frame = time.monotonic()
                    corners, ids, _ = detector.detectMarkers(cv.cvtColor(latest, cv.COLOR_BGR2GRAY))
                    latest_ids = [] if ids is None else [int(i) for i in ids.ravel()]
                    latest_corners = [] if ids is None else [c.reshape(4, 2).tolist() for c in corners]
                    ready_streak = ready_streak + 1 if {40, 41}.issubset(latest_ids) else 0
                    ready = ready_streak >= 3
                    preview = latest.copy()
                    if ids is not None:
                        cv.aruco.drawDetectedMarkers(preview, corners, ids)
                    label = ("READY: S saves" if ready else "Show BOTH IDs 40 and 41")
                    cv.putText(preview, f"{label} | found {latest_ids} | Q quits",
                               (12, 32), cv.FONT_HERSHEY_SIMPLEX, .7,
                               (0, 255, 0) if ready else (0, 255, 255), 2)
                    cv.imshow(title, preview)
                    if time.monotonic() - last_prompt > 2:
                        print(f"Detected {latest_ids}; {'READY' if ready else 'waiting for 40+41'}", flush=True)
                        last_prompt = time.monotonic()
            if time.monotonic() - last_frame > 3:
                raise RuntimeError("No fresh webcam frame for 3s")
            key = cv.waitKey(20) & 255
            if key in (ord("q"), ord("Q"), 27):
                print("Cancelled; no image saved.", flush=True)
                return 0
            if key in (ord("s"), ord("S")):
                if latest is None or time.monotonic() - latest_stamp > .2 or ready_streak < 3:
                    print(f"Not saved: need a fresh READY frame with 40+41; found {latest_ids}", flush=True)
                    continue
                tag = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                path = ROOT / f"leap_marker_bridge_{tag}.png"
                ok, encoded = cv.imencode(".png", latest)
                if not ok:
                    raise RuntimeError("PNG encoding failed")
                path.write_bytes(encoded.tobytes())
                report.update(saved_image=str(path), detected_ids=latest_ids,
                              corners_px={str(i): c for i, c in zip(latest_ids, latest_corners)},
                              captured_at=datetime.datetime.now().astimezone().isoformat())
                (ROOT / "leap_marker_bridge_result.json").write_text(
                    json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
                print(f"Saved: {path}", flush=True)
                return 0
    finally:
        if stream is not None:
            stream.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
