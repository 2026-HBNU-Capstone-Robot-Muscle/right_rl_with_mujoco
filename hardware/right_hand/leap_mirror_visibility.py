"""Camera-only scouting for a second view reflected into the same webcam.

The program does not estimate 3D pose and never opens the motor port.  A
mirror ROI is selected by dragging a rectangle in the preview.  ArUco IDs in
that ROI are detected after horizontal reversal; direct detections from the
same ROI are ignored to avoid counting a reflection twice.
"""

import datetime
import json
import time
from pathlib import Path

import cv2 as cv
import numpy as np

from camera_stream import CameraStream


ROOT = Path(__file__).resolve().parent
TITLE = 'LEAP one-webcam mirror scout | drag MIRROR box | SPACE 20s | Q quit'


def detector():
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    return cv.aruco.ArucoDetector(dictionary, cv.aruco.DetectorParameters())


def cube_ids(frame, det):
    found, ids, _ = det.detectMarkers(cv.cvtColor(frame, cv.COLOR_BGR2GRAY))
    if ids is None:
        return []
    return [(int(i), np.asarray(c).reshape(4, 2).mean(axis=0))
            for i, c in zip(ids.ravel(), found) if 0 <= int(i) <= 5]


def visible_ids(frame, roi, det):
    x, y, w, h = roi
    direct = [i for i, center in cube_ids(frame, det)
              if not (x <= center[0] < x + w and y <= center[1] < y + h)]
    reflection = cube_ids(cv.flip(frame[y:y + h, x:x + w], 1), det)
    reflected = [i for i, _ in reflection]
    return sorted(set(direct)), sorted(set(reflected))


def main():
    report = dict(status='starting', motor_port_opened=False, motor_commands_sent=0,
                  purpose='visibility only; no 3D pose or policy authorization')
    result_file = ROOT / 'leap_mirror_visibility_result.json'
    stream = None
    selection = {'start': None, 'drag': None, 'roi': None}

    def mouse(event, x, y, _flags, _userdata):
        if event == cv.EVENT_LBUTTONDOWN:
            selection['start'] = (x, y)
            selection['drag'] = (x, y)
        elif event == cv.EVENT_MOUSEMOVE and selection['start'] is not None:
            selection['drag'] = (x, y)
        elif event == cv.EVENT_LBUTTONUP and selection['start'] is not None:
            x0, y0 = selection['start']
            xx, yy = min(x0, x), min(y0, y)
            w, h = abs(x - x0), abs(y - y0)
            if w >= 80 and h >= 80:
                selection['roi'] = (xx, yy, w, h)
                print(f'Mirror region: {selection["roi"]}', flush=True)
            selection['start'] = selection['drag'] = None

    try:
        stream = CameraStream(report)
        det = detector()
        # Keep displayed pixels 1:1 so mouse ROI coordinates match 1280x720 frames.
        cv.namedWindow(TITLE, cv.WINDOW_AUTOSIZE)
        cv.setMouseCallback(TITLE, mouse)
        print('CAMERA ONLY. Put mirror OUTSIDE all finger paths. Show cube directly and in mirror.', flush=True)
        print('Drag a box around the MIRROR IMAGE, then SPACE records 20s. Q exits.', flush=True)
        started = None
        last_frame = time.monotonic()
        last_print = 0.0
        samples = []
        snapshots = []
        while True:
            packet = stream.read_packet()
            if packet is not None:
                stamp, frame = packet
                if time.monotonic() - stamp <= .2:
                    last_frame = time.monotonic()
                    roi = selection['roi']
                    direct, reflected = (visible_ids(frame, roi, det) if roi else ([], []))
                    preview = frame.copy()
                    if roi:
                        x, y, w, h = roi
                        cv.rectangle(preview, (x, y), (x + w, y + h), (255, 0, 255), 2)
                    label = (f'direct {direct} | mirror {reflected} | distinct {len(set(direct + reflected))}'
                             if roi else 'DRAG BOX around mirror image')
                    cv.putText(preview, label, (12, 32), cv.FONT_HERSHEY_SIMPLEX,
                               .67, (0, 255, 0) if len(set(direct + reflected)) >= 2 else (0, 255, 255), 2)
                    if started is not None:
                        samples.append(dict(time_s=stamp - started, direct=direct,
                                            mirror=reflected, total_distinct=len(set(direct + reflected))))
                        if len(snapshots) < 3 and len(samples) in (1, 50, 120):
                            snapshots.append(frame.copy())
                    cv.imshow(TITLE, preview)
                    if time.monotonic() - last_print > 2:
                        print(label, flush=True)
                        last_print = time.monotonic()
            if time.monotonic() - last_frame > 3:
                raise RuntimeError('No fresh webcam frame for 3s')
            key = cv.waitKey(20) & 255
            if key in (ord('q'), ord('Q'), 27):
                report['status'] = 'cancelled'
                break
            if key == 32 and started is None:
                if selection['roi'] is None:
                    print('Drag a mirror box before SPACE.', flush=True)
                    continue
                started = time.monotonic()
                print('Recording 20s. Move only the cube, slowly; no motor program.', flush=True)
            if started is not None and time.monotonic() - started >= 20:
                counts = [row['total_distinct'] for row in samples]
                report.update(status='complete', roi_px=selection['roi'], samples=len(samples),
                              two_center_frames=sum(n >= 2 for n in counts),
                              two_center_percent=(100 * sum(n >= 2 for n in counts) / len(counts)
                                                  if counts else 0),
                              direct_ids_seen=sorted({i for row in samples for i in row['direct']}),
                              mirrored_ids_seen=sorted({i for row in samples for i in row['mirror']}),
                              frames=samples)
                tag = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                for index, image in enumerate(snapshots):
                    ok, png = cv.imencode('.png', image)
                    if ok:
                        (ROOT / f'leap_mirror_scout_{tag}_{index}.png').write_bytes(png.tobytes())
                break
    except BaseException as exc:
        report.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        print(f'Mirror scout failed: {report["error"]}', flush=True)
    finally:
        if stream is not None:
            stream.release()
        cv.destroyAllWindows()
        result_file.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(f'Saved: {result_file}', flush=True)
    return 0 if report['status'] in ('complete', 'cancelled') else 1


if __name__ == '__main__':
    raise SystemExit(main())
