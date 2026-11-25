import json
from pathlib import Path
import cv2 as cv,numpy as np
from trace_adaptive_loaded_grid_20261029 import fit,det,unit
R=Path(__file__).resolve().parent
raw=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),cv.aruco.DetectorParameters())
for folder in ('single_forward_trial_20261030_052636_068402','single_forward_trial_20261030_053131_966030'):
 for name in ('cube_before_grasp.jpg','cube_before_horizontal_turn.jpg','cube_horizontal_turn_hold.jpg'):
  p=R/folder/name;im=cv.imdecode(np.frombuffer(p.read_bytes(),np.uint8),1);cs,ids,_=raw.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY));found={} if ids is None else {int(i):np.array(c).reshape(4,2) for i,c in zip(ids.ravel(),cs)};v=fit(p)
  if 5 in found:
   q=found[5].astype(np.float32);H=cv.getPerspectiveTransform(q,unit);mask=cv.inRange(cv.cvtColor(im,cv.COLOR_BGR2HSV),np.array([35,55,40]),np.array([100,255,255]));n,lab,stats,cent=cv.connectedComponentsWithStats(mask);points=[]
   for i in range(1,n):
    if 150<stats[i,4]<20000:
     c=cv.perspectiveTransform(np.float32(cent[i])[None,None],H)[0,0]
     if max(abs(c))<5:points.append(dict(center=cent[i].tolist(),canonical=c.tolist(),area=int(stats[i,4])))
   v.update(raw_ID5_edge_px=np.linalg.norm(q-np.roll(q,1,axis=0),axis=1).tolist(),green_components_near_marker=points)
  print(json.dumps(dict(folder=folder,file=name,raw_ids=list(found),analysis=v)),flush=True)
