"""Search existing recording for simultaneous rigid references; no devices."""
import json
import time
from collections import Counter
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
folder=ROOT/'cube_supported_rotation_trial_20261023_034640_750784'
frames=json.loads((folder/'camera/frames.json').read_bytes())['frames']
phases={'preflight','stable_off_readings','screen_current_pose','stage_2_empty_open',
        'open_before_cube','place_cube','remove_cube','post_off_reference'}
selected=[f for i,f in enumerate(frames) if f['phase'] in phases and i%3==0]
p=cv.aruco.DetectorParameters();p.cornerRefinementMethod=cv.aruco.CORNER_REFINE_CONTOUR
p.adaptiveThreshWinSizeMax=73;p.adaptiveThreshWinSizeStep=4
det=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),p)
clahe=cv.createCLAHE(2.,(8,8))
rows=[];counts=Counter();began=time.monotonic()
for i,f in enumerate(selected):
    im=cv.imdecode(np.frombuffer((folder/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_GRAYSCALE)
    best=None
    for gray in (im,clahe.apply(im)):
        corners,ids,_=det.detectMarkers(gray)
        ids=[] if ids is None else list(map(int,ids.ravel()))
        counts.update(j for j in ids if j in (40,41))
        if 40 in ids and 41 in ids:
            points={str(j):np.asarray(corners[ids.index(j)]).reshape(4,2).tolist() for j in (40,41)}
            rows.append(dict(**f,corners=points));break
    if i%100==0:print('SCANNED',i,'/',len(selected),'PAIRS',len(rows),flush=True)
out=dict(source=folder.name,scanned_frames=len(selected),reference_detections=dict(counts),
         paired_frames=len(rows),elapsed_seconds=time.monotonic()-began,rows=rows)
(folder/'same_run_reference_pair_scan.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in out.items() if k!='rows'},indent=2),flush=True)
