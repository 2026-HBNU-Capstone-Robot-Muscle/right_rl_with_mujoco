"""Read one frame from each Windows camera backend; do not save images."""

import sys
import time

import cv2


def main():
    print(f"OpenCV {cv2.__version__}", flush=True)
    found = False
    for backend_name, backend in (("DirectShow", cv2.CAP_DSHOW),
                                  ("MediaFoundation", cv2.CAP_MSMF)):
        for index in range(4):
            cap = cv2.VideoCapture(index, backend)
            try:
                if not cap.isOpened():
                    print(f"{backend_name} camera {index}: cannot open", flush=True)
                    continue
                frame = None
                for _ in range(5):
                    ok, candidate = cap.read()
                    if ok and candidate is not None:
                        frame = candidate
                        break
                    time.sleep(0.2)
                if frame is None:
                    print(f"{backend_name} camera {index}: open, no frame", flush=True)
                else:
                    height, width = frame.shape[:2]
                    print(f"{backend_name} camera {index}: {width}x{height} frame OK", flush=True)
                    found = True
            finally:
                cap.release()
    return 0 if found else 1


if __name__ == "__main__":
    sys.exit(main())
