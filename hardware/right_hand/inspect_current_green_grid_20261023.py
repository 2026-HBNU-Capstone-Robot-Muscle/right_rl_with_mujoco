from pathlib import Path
import json
import cv2 as cv
import numpy as np

root=Path(__file__).resolve().parent
folder=root/'cube_pose_readonly_20261023_171233_601943'
source=json.loads((folder/'result.json').read_bytes());pair=source['pairs'][5]
image=cv.imdecode(np.frombuffer((folder/pair['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
marker=np.array(pair['markers']['corners']['5']);center=marker.mean(0)
scale=np.mean(np.linalg.norm(marker-np.roll(marker,1,axis=0),axis=1))
hsv=cv.cvtColor(image,cv.COLOR_BGR2HSV)
print('marker scale',scale,'center',center)
for saturation in (55,80,100,120,140,160):
    mask=cv.inRange(hsv,np.array([35,saturation,35]),np.array([100,255,255]))
    _,labels,stats,centers=cv.connectedComponentsWithStats(mask)
    rows=[]
    for stat,p in zip(stats[1:],centers[1:]):
        x,y,w,h,area=stat
        if 100<area and np.linalg.norm(p-center)<160:
            rows.append(dict(area=int(area),center=p.tolist(),size=[int(w),int(h)]))
    print(saturation,rows)
