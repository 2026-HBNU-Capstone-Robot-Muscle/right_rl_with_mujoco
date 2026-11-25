import cv2 as cv,numpy as np
from pathlib import Path
from measure_inverse_side_marker_20261029 import det,poses
R=Path(__file__).resolve().parent
for folder,name in (('adaptive_yaw_loaded_20261029_160451_416563','cube_horizontal_turn_hold.jpg'),('gapped_inverse_trial_20261029_184252_272166','cube_before_horizontal_turn.jpg')):
 im=cv.imdecode(np.frombuffer((R/folder/name).read_bytes(),np.uint8),cv.IMREAD_COLOR)
 c,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY));print(folder,'IDs',ids)
 if ids is not None:
  for p,i in zip(c,ids.ravel()):
   if i in(0,40,41):print(int(i),np.asarray(p).reshape(4,2).tolist(),[(s['rmse_px'],s['translation'].tolist()) for s in poses(np.asarray(p).reshape(4,2),.0107 if i==0 else .027)])
