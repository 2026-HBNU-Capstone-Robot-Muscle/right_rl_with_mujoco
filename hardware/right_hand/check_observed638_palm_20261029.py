"""Fixed housing image features; this does not certify a robot coordinate frame."""
import hashlib,json
from pathlib import Path
import cv2 as cv,numpy as np
ROOT=Path(__file__).resolve().parent;F=ROOT/'adaptive_yaw_loaded_20261029_160451_416563';O=F/'offline_review'
def load(name):return cv.imdecode(np.frombuffer((F/name).read_bytes(),np.uint8),cv.IMREAD_GRAYSCALE)
a=load('cube_before_grasp.jpg');mask=np.zeros(a.shape,np.uint8);mask[400:570,205:380]=255
sift=cv.SIFT_create(nfeatures=1500);ka,da=sift.detectAndCompute(a,mask);rows=[]
for name in ('cube_before_horizontal_turn.jpg','cube_horizontal_turn_hold.jpg','cube_after_open.jpg'):
    b=load(name);kb,db=sift.detectAndCompute(b,mask);matches=[m for m,n in cv.BFMatcher().knnMatch(da,db,k=2) if m.distance<.7*n.distance]
    row=dict(image=name,ratio_matches=len(matches),accepted=False)
    if len(matches)>=10:
        p=np.float32([ka[m.queryIdx].pt for m in matches]);q=np.float32([kb[m.trainIdx].pt for m in matches]);h,keep=cv.findHomography(p,q,cv.RANSAC,1.5)
        if h is not None:
            selected=keep.ravel().astype(bool);count=int(selected.sum())
            if count>=10:
                row.update(accepted=True,inliers=count,shift_px_median_p95=np.percentile(np.linalg.norm(p[selected]-q[selected],axis=1),[50,95]).tolist(),homography=h.tolist())
    rows.append(row)
r=dict(source=F.name,ROI_px=[205,400,380,570],ROI='fixed palm cover and ID40; excludes cube, moving fingers and visible human hand',
    rows=rows,certified_robot_registration=False,source_images_modified=False,motor_writes=0)
(O/'fixed_palm_image_check.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');print(json.dumps(r))
