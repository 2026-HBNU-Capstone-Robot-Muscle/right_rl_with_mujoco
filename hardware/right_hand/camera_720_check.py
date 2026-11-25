"""720p marker visibility only. No pose calibration or motor access."""
import argparse
import json
import statistics
import time
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def open_hd(cv, index, require_mjpg=False, *, backends=None):
    attempts = []
    if backends is None:
        backends = ((cv.CAP_DSHOW, "DSHOW"), (cv.CAP_MSMF, "MSMF"))
    for backend, name in backends:
        for fourcc in (("MJPG",) if require_mjpg else ("MJPG", "YUY2")):
            cap = cv.VideoCapture(index, backend)
            accepted = False
            try:
                if not cap.isOpened():
                    attempts.append({"backend": name, "requested_format": fourcc, "error": "not opened"})
                    continue
                if not require_mjpg:
                    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*fourcc))
                cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
                cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
                cap.set(cv.CAP_PROP_FPS, 30)
                # OpenCV 4.12 DSHOW may re-negotiate without the previous codec
                # on size/FPS changes. Apply FOURCC LAST, then verify it.
                if require_mjpg:
                    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*fourcc))
                frame = None
                for _ in range(10):
                    ok, candidate = cap.read()
                    if ok and candidate is not None:
                        frame = candidate
                        break
                actual = None if frame is None else [frame.shape[1], frame.shape[0]]
                code = int(cap.get(cv.CAP_PROP_FOURCC))
                meta = {"backend": name, "requested_format": fourcc,
                        "actual_size": actual,
                        "actual_format": "".join(chr((code >> (8 * i)) & 255) for i in range(4)),
                        "reported_fps": cap.get(cv.CAP_PROP_FPS)}
                attempts.append(meta)
                print(meta, flush=True)
                if actual == [1280, 720] and (not require_mjpg or meta['actual_format'] == 'MJPG'):
                    accepted = True
                    return cap, meta, attempts
            finally:
                if not accepted:
                    cap.release()
    requirement = ' with MJPG (YUY2 fallback rejected)' if require_mjpg else ''
    raise RuntimeError("No actual 1280x720 frame" + requirement + ". Attempts: " + json.dumps(attempts))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=1)
    args = parser.parse_args()
    import cv2 as cv
    output = ROOT / "camera_720_check_result.json"
    cap, meta, attempts = open_hd(cv, args.camera, require_mjpg=True)
    detector = cv.aruco.ArucoDetector(
        cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50), cv.aruco.DetectorParameters())
    title = "720p markers ONLY - no motors"
    cv.namedWindow(title, cv.WINDOW_NORMAL)
    cv.resizeWindow(title, 960, 540)
    started = None
    total = failed = hand = cube = both = two = 0
    counts = {str(i): 0 for i in range(6)}
    edges = {str(i): [] for i in [0, 1, 2, 3, 4, 5, 40]}
    print("Cover the EXTERNAL webcam lens to verify this camera; then uncover.", flush=True)
    print("Keep hand/cube still. SPACE: 10-second test; S: raw photo; Q: exit.", flush=True)
    print("640x480 calibration is NOT loaded. No 3D pose or motor commands.", flush=True)
    completed = False
    recent = deque()
    try:
        while True:
            if started is not None and time.monotonic() - started >= 10:
                completed = True
                break
            ok, frame = cap.read()
            if not ok or frame is None:
                failed += int(started is not None)
                if cv.waitKey(10) & 255 in (ord('q'), 27):
                    break
                continue
            if frame.shape[:2] != (720, 1280):
                raise RuntimeError("Frame size changed; stopping test.")
            corners, ids, _ = detector.detectMarkers(cv.cvtColor(frame, cv.COLOR_BGR2GRAY))
            visible = set() if ids is None else set(map(int, ids.ravel()))
            cube_ids = visible.intersection(range(6))
            has_hand = 40 in visible
            good = has_hand and len(cube_ids) >= 2
            now = time.monotonic()
            recent.append((now, good, visible))
            while recent and recent[0][0] < now - 3.0:
                recent.popleft()
            recent_percent = 100.0 * sum(row[1] for row in recent) / len(recent)
            recent_ids = {str(i): round(100.0 * sum(i in row[2] for row in recent) / len(recent), 1)
                          for i in (0, 1, 2, 3, 4, 5, 40)}
            if started is not None:
                total += 1
                hand += has_hand
                cube += bool(cube_ids)
                both += has_hand and bool(cube_ids)
                two += good
                for i in cube_ids:
                    counts[str(i)] += 1
                if ids is not None:
                    for c, marker_id in zip(corners, ids.ravel()):
                        key = str(int(marker_id))
                        if key in edges:
                            pts = c.reshape(4, 2)
                            sides = [float(((pts[j] - pts[(j + 1) % 4]) ** 2).sum()) ** .5 for j in range(4)]
                            edges[key].append(statistics.mean(sides))
            display = frame.copy()
            if ids is not None:
                cv.aruco.drawDetectedMarkers(display, corners, ids)
            state = "SPACE starts" if started is None else f"{max(0, 10 - (time.monotonic()-started)):.1f}s left"
            cv.putText(display, f"1280x720 IDs:{sorted(visible)} hand+2:{good} {state}",
                       (15, 35), cv.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0) if good else (0, 0, 255), 2)
            warming = ' WARMUP' if recent[-1][0] - recent[0][0] < 2.8 else ''
            cv.putText(display, f"MJPG | hand+2 last 3s: {recent_percent:.1f}%{warming} | S save / Q exit",
                       (15, 65), cv.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
            cv.putText(display, 'Last 3s IDs%: ' + ' '.join(f'{i}:{p:.0f}' for i, p in recent_ids.items()),
                       (15, 95), cv.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
            cv.imshow(title, display)
            key = cv.waitKey(1) & 255
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                success, encoded = cv.imencode('.png', frame)
                if success:
                    (ROOT / 'camera_720_photo.png').write_bytes(encoded.tobytes())
                    print('Saved camera_720_photo.png', flush=True)
                    snapshot = {'capture': meta, 'visible_ids': sorted(visible),
                                'recent_hand_plus_two_percent': round(recent_percent, 1),
                                'recent_id_percent': recent_ids, 'recent_frames': len(recent),
                                'recent_span_seconds': round(recent[-1][0] - recent[0][0], 2),
                                'note': 'Live placement guidance only; not a hardware readiness test.'}
                    (ROOT / 'camera_720_photo_status.json').write_text(json.dumps(snapshot, indent=2), encoding='utf-8')
                    success, encoded = cv.imencode('.png', display)
                    if success:
                        (ROOT / 'camera_720_photo_overlay.png').write_bytes(encoded.tobytes())
                    print(f'Live hand+2: {recent_percent:.1f}%; IDs: {sorted(visible)}', flush=True)
            if key == 32 and started is None:
                started = time.monotonic()
                print('Measuring 10 seconds...', flush=True)
    finally:
        cap.release()
        cv.destroyAllWindows()
    if started is None:
        print('No measurement started; previous result unchanged.')
        return 0
    elapsed = time.monotonic() - started
    report = {"status": "complete" if completed else "interrupted", "camera": args.camera,
              "capture": meta, "attempts": attempts, "frames": total, "read_failures": failed,
              "elapsed_seconds": round(elapsed, 2), "processed_fps": round(total / elapsed, 1),
              "percentages": {k: round(100 * v / total, 1) if total else None
                              for k, v in dict(hand=hand, cube=cube, both=both, two_cube=two).items()},
              "cube_id_frames": counts,
              "detected_marker_median_edge_px": {k: round(statistics.median(v), 1) if v else None for k, v in edges.items()},
              "pose_calibrated": False, "motors_accessed": False}
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)
    print(f'Saved: {output}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
