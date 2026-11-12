"""Motor-free framing preview: rigid ID41 plus two cube markers."""

import datetime
import json
import sys
import time
from pathlib import Path

import cv2 as cv
import numpy as np

from camera_stream import CameraStream


ROOT = Path(__file__).resolve().parent


def main():
    report = {'motor_port_opened': False, 'motor_commands_sent': 0,
              'required_anchor': 41, 'required_cube_markers': 2}
    stream = None
    title = 'LEAP two-face preview | ID41 + 2 cube markers | S saves | Q quits'
    detector = cv.aruco.ArucoDetector(
        cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
        cv.aruco.DetectorParameters())
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    with np.load(ROOT / 'camera1_720.npz') as calibration:
        K, dist = calibration['K'].copy(), calibration['dist'].copy()
    sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'vision' / 'rubik_center'))
    import cube_board
    import track_cube
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text(encoding='utf-8'))['face_rot'])
    points, ids = cube_board.object_points(cube_mm=54.5, marker_mm=10.6,
                                            layout='centers', face_rot=rotations)
    board = cv.aruco.Board(points, dictionary, ids)
    try:
        stream = CameraStream(report)
        print('NO MOTOR PORT. Keep motor power OFF. Hold cube near its start position.', flush=True)
        print('Aim camera so ID41 and two adjacent cube-face IDs are visible. S saves; Q exits.', flush=True)
        cv.namedWindow(title, cv.WINDOW_NORMAL)
        cv.resizeWindow(title, 1100, 620)
        last_frame = time.monotonic()
        latest = None
        latest_stamp = 0.0
        latest_ids = []
        latest_reproj = None
        ready_streak = 0
        last_prompt = 0.0
        while True:
            packet = stream.read_packet()
            if packet is not None:
                latest_stamp, latest = packet
                if time.monotonic() - latest_stamp <= .2:
                    last_frame = time.monotonic()
                    corners, ids, _ = detector.detectMarkers(cv.cvtColor(latest, cv.COLOR_BGR2GRAY))
                    latest_ids = [] if ids is None else [int(i) for i in ids.ravel()]
                    cube_ids = [i for i in latest_ids if 0 <= i <= 5]
                    pose = (track_cube.step(latest, board, detector, dictionary, K, dist)
                            if 41 in latest_ids and len(set(cube_ids)) >= 2 else None)
                    latest_reproj = None if pose is None else pose.get('reproj')
                    good_pose = (pose is not None and pose.get('n_cube_markers', 0) >= 2
                                 and latest_reproj is not None and np.isfinite(latest_reproj)
                                 and latest_reproj <= 3.0)
                    ready_streak = ready_streak + 1 if good_pose else 0
                    ready = ready_streak >= 3
                    preview = latest.copy()
                    if ids is not None:
                        cv.aruco.drawDetectedMarkers(preview, corners, ids)
                    label = 'READY: S saves' if ready else 'Show ID41 + TWO cube face markers'
                    error_text = 'pose n/a' if latest_reproj is None else f'pose {latest_reproj:.1f}px'
                    cv.putText(preview, f'{label} | IDs {latest_ids} | {error_text} | Q quits', (12, 32),
                               cv.FONT_HERSHEY_SIMPLEX, .63,
                               (0, 255, 0) if ready else (0, 255, 255), 2)
                    cv.imshow(title, preview)
                    if time.monotonic() - last_prompt >= 2:
                        print(f'IDs {latest_ids}; {error_text}; '
                              f'{"READY" if ready else "adjust camera/cube"}', flush=True)
                        last_prompt = time.monotonic()
            if time.monotonic() - last_frame > 3:
                raise RuntimeError('No fresh webcam frame for 3s')
            key = cv.waitKey(20) & 255
            if key in (ord('q'), ord('Q'), 27):
                print('Preview closed; no motor access.', flush=True)
                return 0
            if key in (ord('s'), ord('S')):
                if latest is None or time.monotonic() - latest_stamp > .2 or ready_streak < 3:
                    print(f'Not saved: need fresh ID41 + 2 cube markers; found {latest_ids}', flush=True)
                    continue
                tag = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                path = ROOT / f'leap_two_face_preview_{tag}.png'
                ok, encoded = cv.imencode('.png', latest)
                if not ok:
                    raise RuntimeError('PNG encoding failed')
                path.write_bytes(encoded.tobytes())
                report.update(saved_image=str(path), detected_ids=latest_ids,
                              cube_reprojection_px=float(latest_reproj),
                              captured_at=datetime.datetime.now().astimezone().isoformat())
                (ROOT / 'leap_two_face_preview_result.json').write_text(
                    json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
                print(f'Saved: {path}', flush=True)
                return 0
    finally:
        if stream is not None:
            stream.release()
        cv.destroyAllWindows()


if __name__ == '__main__':
    raise SystemExit(main())
