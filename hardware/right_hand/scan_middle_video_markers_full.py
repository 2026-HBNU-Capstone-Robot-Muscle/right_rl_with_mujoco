"""Scan an existing video only; no camera, serial, or motor access."""
from pathlib import Path
import json
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector

root = Path(__file__).resolve().parent
out = root / 'middle_pose_review_20261017/phone_video'
meta = json.loads((out / 'metadata.json').read_text())
cap = cv.VideoCapture(meta['source'])
if not cap.isOpened():
    raise RuntimeError('Video cannot be opened')
detector = MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50), [41], hand_id=40, required_cube_count=1)
rows = []
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        index = len(rows)
        corners, ids, _ = detector.detectMarkers(cv.cvtColor(frame, cv.COLOR_BGR2GRAY))
        markers = {} if ids is None else {str(int(i)): np.asarray(c).reshape(4, 2).tolist() for i, c in zip(ids.ravel(), corners)}
        paired = '40' in markers and '41' in markers
        rows.append(dict(frame=index, time_s=index/meta['fps'], markers=markers))
        if paired:
            cv.imencode('.jpg', frame)[1].tofile(str(out / f'paired_{index:04d}.jpg'))
        if len(rows) % 100 == 0:
            print(f'Scanned {len(rows)} frames; paired {sum("40" in r["markers"] and "41" in r["markers"] for r in rows)}', flush=True)
finally:
    cap.release()
summary = dict(frames=len(rows), paired_frames=sum('40' in r['markers'] and '41' in r['markers'] for r in rows),
               id40_frames=sum('40' in r['markers'] for r in rows), id41_frames=sum('41' in r['markers'] for r in rows),
               source_sha256=meta['sha256'], camera_calibration_verified=False)
(out / 'markers_full.json').write_text(json.dumps(dict(summary=summary, rows=rows), indent=2))
print(json.dumps(summary), flush=True)
