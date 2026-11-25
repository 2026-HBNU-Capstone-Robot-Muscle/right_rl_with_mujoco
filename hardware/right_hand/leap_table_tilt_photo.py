"""Motor-free webcam capture of a flat tabletop chessboard beside hand ID40."""

from collections import deque
import datetime as dt
import json
import time
from pathlib import Path

import cv2 as cv

from camera_stream import CameraStream

ROOT = Path(__file__).resolve().parent
PATTERN = (9, 6)  # internal corners of camera_chessboard_A4_25mm.pdf


def main():
    report = {'motor_port_opened': False, 'motor_commands_sent': 0,
              'required': 'flat 9x6 internal-corner board and hand ID40 in one frame'}
    stream = None
    title = 'LEAP table tilt | flat board + ID40 | S saves | Q quits | NO MOTORS'
    detector = cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
                                      cv.aruco.DetectorParameters())
    recent = deque(maxlen=5)
    streak = 0
    last_frame = time.monotonic()
    last_prompt = 0.
    try:
        stream = CameraStream(report)
        print('NO MOTOR PORT. Remove cube; keep motor power OFF.', flush=True)
        print('Lay the printed 9x6 chessboard FLAT on a rigid HORIZONTAL table beside the hand.', flush=True)
        print('Show the whole board and ID40 together; ID41 is helpful. S saves five frames; Q exits.', flush=True)
        cv.namedWindow(title, cv.WINDOW_NORMAL)
        cv.resizeWindow(title, 1100, 620)
        while True:
            packet = stream.read_packet()
            if packet is not None:
                stamp, frame = packet
                if time.monotonic()-stamp <= .2:
                    last_frame = time.monotonic()
                    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                    found, board_corners = cv.findChessboardCornersSB(
                        gray, PATTERN, flags=cv.CALIB_CB_NORMALIZE_IMAGE)
                    aruco_corners, ids, _ = detector.detectMarkers(gray)
                    detected = [] if ids is None else [int(i) for i in ids.ravel()]
                    valid = bool(found and 40 in detected)
                    streak = streak+1 if valid else 0
                    if valid:
                        recent.append(frame.copy())
                    else:
                        recent.clear()
                    preview = frame.copy()
                    if found:
                        cv.drawChessboardCorners(preview, PATTERN, board_corners, found)
                    if ids is not None:
                        cv.aruco.drawDetectedMarkers(preview, aruco_corners, ids)
                    label = 'READY: S saves' if streak >= 3 else 'Show BOARD + ID40'
                    cv.putText(preview, f'{label} | IDs {detected} | Q quits', (12,32),
                               cv.FONT_HERSHEY_SIMPLEX,.65,(0,255,0) if streak>=3 else (0,255,255),2)
                    cv.imshow(title, preview)
                    if time.monotonic()-last_prompt > 2:
                        print(f'Board={bool(found)} IDs={detected}; {label}',flush=True)
                        last_prompt=time.monotonic()
            if time.monotonic()-last_frame > 3:
                raise RuntimeError('No fresh webcam frame for 3s')
            key = cv.waitKey(20)&255
            if key in (ord('q'),ord('Q'),27):
                print('Cancelled; no image saved.',flush=True)
                return 0
            if key in (ord('s'),ord('S')):
                if streak<3 or len(recent)<3:
                    print('Not saved: need fresh BOARD + ID40 together.',flush=True)
                    continue
                folder = ROOT/f'table_tilt_{dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")}'
                folder.mkdir()
                for index, image in enumerate(recent):
                    ok, encoded = cv.imencode('.png', image)
                    if not ok:
                        raise RuntimeError('PNG encoding failed')
                    (folder/f'frame_{index}.png').write_bytes(encoded.tobytes())
                report.update(status='captured',folder=str(folder),frames=len(recent),
                              captured_at=dt.datetime.now().astimezone().isoformat())
                (folder/'capture_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
                from sac_table_tilt_from_photo import analyze
                result = analyze(folder)
                print(f'Saved: {folder}',flush=True)
                print(f'ID40 plane/table angle: {result["marker_table_angle_deg"]:.1f} deg',flush=True)
                print('No motor commands sent.',flush=True)
                return 0
    finally:
        if stream is not None:
            stream.release()
        cv.destroyAllWindows()


if __name__=='__main__':
    raise SystemExit(main())
