"""Stored cube rim crop with pixel coordinates, not a generated image."""
from pathlib import Path
import cv2 as cv
import numpy as np
p=Path(__file__).resolve().parent/'cube_supported_rotation_trial_20261023_034640_750784'
im=cv.imdecode(np.frombuffer((p/'camera/frame_01560.jpg').read_bytes(),np.uint8),cv.IMREAD_COLOR)
a=im[100:280,520:900].copy()
a=cv.resize(a,None,fx=2,fy=2)
for x in range(520,901,20):
    xx=(x-520)*2;cv.line(a,(xx,0),(xx,a.shape[0]-1),(180,180,180),1)
    cv.putText(a,str(x),(xx+2,15),cv.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),2)
    cv.putText(a,str(x),(xx+2,15),cv.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
for y in range(100,281,20):
    yy=(y-100)*2;cv.line(a,(0,yy),(a.shape[1]-1,yy),(180,180,180),1)
    cv.putText(a,str(y),(2,max(30,yy-3)),cv.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),2)
    cv.putText(a,str(y),(2,max(30,yy-3)),cv.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
cv.imencode('.jpg',a)[1].tofile(str(p/'cube_rim_pixel_grid.jpg'))
