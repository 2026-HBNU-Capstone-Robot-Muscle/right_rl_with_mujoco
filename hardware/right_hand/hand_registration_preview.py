"""Project tentative rigid motor-axis landmarks. No policy or motor access."""
import json
import datetime
import time
from pathlib import Path
import cv2 as cv
import numpy as np
from camera_stream import CameraStream
from hand_registration_geometry import candidate

ROOT = Path(__file__).resolve().parent


def save_view(report, meta, frame, raw, projected, labels, rv, tv, view):
    prefix = 'hand_registration' if view == 'preview' else 'hand_alignment_'+view
    stamp = datetime.datetime.now().astimezone().isoformat()
    for suffix, img in [('preview', frame), ('raw', raw)]:
        success, encoded = cv.imencode('.png', img)
        if not success: raise RuntimeError('PNG encoding failed: '+suffix)
        (ROOT / (prefix+'_'+suffix+'.png')).write_bytes(encoded.tobytes())
    snapshot = dict(report, timestamp=stamp, view=view, capture=meta,
        marker_pose_available=projected is not None,
        projected_landmarks_px=None if projected is None else dict(zip(labels, projected.tolist())),
        marker_rvec_camera=None if rv is None else rv.ravel().tolist(),
        marker_tvec_camera_m=None if tv is None else tv.ravel().tolist(),
        interpretation='Projected candidate only; physical axis alignment must be checked independently')
    (ROOT / (prefix+'_preview_result.json')).write_text(json.dumps(snapshot, indent=2), encoding='utf-8')
    print('Saved '+prefix+'_preview.png at '+stamp+' (NOT calibration approval)', flush=True)


def main():
    report = candidate()
    calibration = np.load(ROOT / 'camera1_720.npz')
    if (int(calibration['height']), int(calibration['width'])) != (720, 1280):
        raise SystemExit('Expected 1280x720 calibration')
    K, dist = calibration['K'], calibration['dist']
    detector = cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
                                     cv.aruco.DetectorParameters())
    s = 28.2 / 1000
    marker = np.array([[-s/2, s/2, 0], [s/2, s/2, 0], [s/2, -s/2, 0], [-s/2, -s/2, 0]], np.float32)
    labels = ['5', '9', '13', '1', 'G']
    points = np.array([report['motor_axis_landmarks_in_marker_m'][i] for i in labels[:-1]] +
                      [report['grasp_site_in_marker_m_candidate']], np.float64)
    cap = CameraStream(report)
    meta = cap.meta
    title = 'HAND FRAME CANDIDATE - NO MOTORS'
    cv.namedWindow(title, cv.WINDOW_NORMAL)
    cv.resizeWindow(title, 960, 540)
    print('Remove cube; keep hand fixed and ID40 visible. NO MOTORS.', flush=True)
    print('Circles 5/9/13/1 are tentative BASE ROTATION AXES, not motor casing centers.', flush=True)
    print('G is the predicted grasp reference, NOT a measured physical point.', flush=True)
    print('Keep HAND fixed; move only camera for two views. Keep ID40 and base joints visible.', flush=True)
    print('1 saves TOP view; 2 saves SIDE view; S saves regular check; Q exits.', flush=True)
    print('Use two visibly different angles; do not change lens zoom or marker mounting.', flush=True)
    last_frame_time = time.monotonic()
    save_requested = None
    view_keys = {ord('1'): 'top', ord('2'): 'side', ord('s'): 'preview', ord('S'): 'preview'}
    saved_views = set()
    try:
        while True:
            packet = cap.read_packet()
            if packet is None or time.monotonic()-packet[0]>.2:
                if time.monotonic() - last_frame_time > 5:
                    raise RuntimeError('No camera frames for 5 seconds')
                key = cv.waitKey(1)&255
                if key in (ord('q'), ord('Q'), 27): break
                if key in view_keys: save_requested = view_keys[key]
                time.sleep(.01)
                continue
            _, frame = packet
            last_frame_time = time.monotonic()
            if frame.shape[:2] != (720, 1280):
                raise RuntimeError('Resolution changed')
            raw = frame.copy()
            corners, ids, _ = detector.detectMarkers(cv.cvtColor(frame, cv.COLOR_BGR2GRAY))
            found = [] if ids is None else [c.reshape(4, 2) for c, i in zip(corners, ids.ravel()) if int(i) == 40]
            projected = None
            rv = tv = None
            if len(found) == 1:
                pose_ok, rv, tv = cv.solvePnP(marker, found[0], K, dist)
                if pose_ok:
                    camera_points = (cv.Rodrigues(rv)[0] @ points.T).T + tv.ravel()
                    if np.all(camera_points[:, 2] > 0):
                        projected = cv.projectPoints(points, rv, tv, K, dist)[0].reshape(-1, 2)
                        if not np.isfinite(projected).all():
                            projected = None
                if projected is not None:
                    for label, p in zip(labels, projected):
                        if not (-2000 < p[0] < 4000 and -2000 < p[1] < 4000):
                            continue
                        xy = tuple(np.rint(p).astype(int))
                        color = (0, 0, 255) if label == 'G' else ((0, 255, 255) if label == '9' else (255, 255, 0))
                        cv.circle(frame, xy, 9, color, 2)
                        cv.drawMarker(frame, xy, color, cv.MARKER_CROSS, 16, 2)
                        cv.putText(frame, 'grasp G' if label == 'G' else 'axis '+label,
                                   (xy[0]+12, xy[1]), cv.FONT_HERSHEY_SIMPLEX, .65, color, 2)
            cv.putText(frame, 'NO MOTORS | 1 TOP | 2 SIDE | S regular save | Q quit',
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
            cv.putText(frame, 'Saved this session: '+(', '.join(sorted(saved_views)) or 'none'),
                       (10, 85), cv.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 2)
            if projected is None:
                cv.putText(frame, 'ID40 pose unavailable', (10, 60), cv.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
            cv.imshow(title, frame)
            key = cv.waitKey(1) & 255
            if key in (ord('q'), ord('Q'), 27):
                break
            requested = view_keys.get(key, save_requested)
            if requested is not None:
                save_requested = None
                if projected is None:
                    print('Not saved: show ID40 clearly, then press the save key again.', flush=True)
                else:
                    save_view(report, meta, frame, raw, projected, labels, rv, tv, requested)
                    saved_views.add(requested)
    finally:
        cap.release()
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()
