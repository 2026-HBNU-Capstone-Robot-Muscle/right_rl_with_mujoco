"""Offline marker evidence from existing phone frames; no camera/motor access."""
from pathlib import Path
import json
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'middle_pose_review_20261017/phone_video'
detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[41],hand_id=40,required_cube_count=1)
rows=[]
for p in sorted(OUT.glob('frame_*.jpg')):
    im=cv.imdecode(np.fromfile(p,dtype=np.uint8),cv.IMREAD_COLOR)
    corners,ids,_=detector.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
    found={} if ids is None else {str(int(i)):np.asarray(c).reshape(4,2).tolist() for i,c in zip(ids.ravel(),corners)}
    rows.append(dict(image=p.name,shape=list(im.shape),markers=found,detector=dict(detector.stats)))
    print(p.name,list(found),flush=True)
(OUT/'markers.json').write_text(json.dumps(dict(rows=rows,paired_frames=sum(len(r['markers'])==2 for r in rows)),indent=2))
