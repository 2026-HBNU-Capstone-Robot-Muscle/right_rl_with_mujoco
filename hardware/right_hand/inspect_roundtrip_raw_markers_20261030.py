import cv2 as cv,numpy as np
from pathlib import Path
R=Path(__file__).resolve().parent;F=R/'yaw_roundtrip_trial_20261030_011416_269699'
dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50);params=cv.aruco.DetectorParameters();params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_CONTOUR
d=cv.aruco.ArucoDetector(dictionary,params)
for name in ('cube_before_grasp.jpg','cube_before_horizontal_turn.jpg','cube_forward_turn_hold.jpg','cube_inverse_turn_hold.jpg'):
 im=cv.imdecode(np.frombuffer((F/name).read_bytes(),np.uint8),cv.IMREAD_GRAYSCALE)
 for scale in (1,2):
  frame=im if scale==1 else cv.resize(im,None,fx=scale,fy=scale,interpolation=cv.INTER_CUBIC)
  c,ids,_=d.detectMarkers(frame)
  print(name,scale,[(int(i),np.asarray(p).reshape(4,2).mean(0).tolist()) for i,p in zip([] if ids is None else ids.ravel(),c)])
