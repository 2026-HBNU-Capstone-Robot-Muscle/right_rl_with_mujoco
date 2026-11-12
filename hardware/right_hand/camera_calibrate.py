"""Interactive chessboard calibration in a normal Windows user session.

The printed 50 mm ruler measured about 48 mm, so each nominal 25 mm
chessboard square is treated as 24 mm. No hand motors are accessed.
"""

from pathlib import Path
import argparse

import cv2
import numpy as np
import time


COLS, ROWS = 9, 6
SQUARE_MM = 24.0
TARGET_SAMPLES = 20


def open_camera(index):
    # The earlier probe confirmed 640x480. Try that first, then HD.
    for width, height in ((640, 480), (1280, 720)):
        for backend in (cv2.CAP_DSHOW, cv2.CAP_MSMF):
            cap = cv2.VideoCapture(index, backend)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            for _ in range(10):
                ok, frame = cap.read()
                if ok and frame is not None:
                    return cap, frame
                time.sleep(0.1)
            cap.release()
    raise RuntimeError(
        f"Camera {index} could not provide a frame. Close Camera/OBS/Zoom/Teams "
        "and retry; if it still fails, rerun camera_probe.cmd."
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hd', action='store_true', help='Require 1280x720; save a separate calibration')
    parser.add_argument('--camera', type=int, choices=range(4))
    args = parser.parse_args()
    print("Camera indices can change when OBS virtual camera or USB devices reconnect.")
    print("Choose 0-3, then COVER THE PHYSICAL WEBCAM LENS to confirm the preview.")
    index = args.camera if args.camera is not None else int(input("Camera index [0-3]: ").strip())
    if index not in range(4):
        raise ValueError("Choose an index from 0 to 3")

    if args.hd:
        from camera_720_check import open_hd
        cap, _, _ = open_hd(cv2, index)
        width, height = 1280, 720
    else:
        cap, frame = open_camera(index)
        width, height = frame.shape[1], frame.shape[0]
    print(f"Camera {index}: {width}x{height}; square size {SQUARE_MM:.1f} mm")
    print("Move/tilt the board around the image. SPACE: capture detected corners; Q: finish.")

    obj = np.zeros((ROWS * COLS, 3), np.float32)
    obj[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * (SQUARE_MM / 1000.0)
    object_points, image_points = [], []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("Frame read failed; retrying")
                continue
            if frame.shape[:2] != (height, width):
                raise RuntimeError("Camera resolution changed during calibration")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, (COLS, ROWS))
            display = frame.copy()
            if found:
                cv2.drawChessboardCorners(display, (COLS, ROWS), corners, found)
            status = f"{len(image_points)}/{TARGET_SAMPLES}  board: {'YES' if found else 'NO'}  SPACE capture  Q finish"
            cv2.putText(display, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 255, 0) if found else (0, 0, 255), 2)
            cv2.imshow("LEAP camera calibration", display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == 32 and found:
                refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                object_points.append(obj.copy())
                image_points.append(refined.copy())
                print(f"Captured {len(image_points)}/{TARGET_SAMPLES}")
                if len(image_points) >= TARGET_SAMPLES:
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if len(image_points) < 8:
        print(f"Only {len(image_points)} valid views; no calibration saved (need >=8).")
        return 1
    rms, K, dist, _, _ = cv2.calibrateCamera(
        object_points, image_points, (width, height), None, None)
    suffix = '_720' if args.hd else ''
    output = Path(__file__).resolve().parent / f"camera{index}{suffix}.npz"
    np.savez(output, K=K, dist=dist, rms=rms, n=len(image_points),
             width=width, height=height, square_mm=SQUARE_MM)
    print(f"Saved: {output}")
    print(f"Samples: {len(image_points)}, RMS: {rms:.3f} px")
    if rms > 1.0:
        print("WARNING: high error; more diverse views are needed before use.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
