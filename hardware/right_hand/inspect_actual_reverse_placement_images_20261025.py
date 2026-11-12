"""Marker visibility in the saved new placement, without assuming registration."""
import json,hashlib
from pathlib import Path
import numpy as np
import cv2 as cv
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'reverse_start_placement_20261025_075405_164917'
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_placement_review'
params=cv.aruco.DetectorParameters();params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_SUBPIX
detector=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),params)
rows=[]
for name in ('reverse_start_empty.jpg','cube_placed.jpg','cube_fixed_hold.jpg'):
    p=SOURCE/name;blob=p.read_bytes();im=cv.imdecode(np.frombuffer(blob,np.uint8),cv.IMREAD_COLOR)
    corners,ids,_=detector.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
    seen={} if ids is None else {int(i):c.reshape(4,2).tolist() for i,c in zip(ids.ravel(),corners)}
    rows.append(dict(image=name,sha256=hashlib.sha256(blob).hexdigest(),size=list(im.shape[:2]),markers=seen))
    print(name,json.dumps(seen))
(OUT/'marker_visibility.json').write_text(json.dumps(dict(images=rows,registration_verified=False),indent=2),encoding='utf-8')
