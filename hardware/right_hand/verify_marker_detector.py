"""Compare detectors on saved images only. Never opens hardware."""
import json
import time
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector


def main():
    root=Path(__file__).resolve().parent
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    baseline=cv.aruco.ArucoDetector(dictionary,cv.aruco.DetectorParameters())
    improved=MarkerDetector(dictionary,range(6))
    rows=[]
    for name in ('camera_720_photo.png','hand_registration_raw.png','id4_webcam.png'):
        path=root/name
        if not path.exists(): continue
        frame=cv.imdecode(np.fromfile(path,dtype=np.uint8),cv.IMREAD_GRAYSCALE)
        if frame is None: continue
        began=time.monotonic(); _,ids,_=baseline.detectMarkers(frame)
        baseline_ms=(time.monotonic()-began)*1000
        old=[] if ids is None else sorted(int(i) for i in ids.ravel() if i<6 or i==40)
        improved.detectMarkers(frame)
        row=dict(file=name,baseline_ids=old,baseline_ms=baseline_ms,improved=improved.stats.copy())
        rows.append(row)
        print(json.dumps(row))
    (root/'marker_detector_offline_result.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')


if __name__=='__main__': main()
