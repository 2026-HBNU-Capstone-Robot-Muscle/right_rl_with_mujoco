"""Compact saved-model diagnostics; no device APIs."""
import json
from pathlib import Path
import numpy as np
import cv2 as cv
from grasp_place_plan import endpoints
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/book20_current_review'
r=json.loads((OUT/'result.json').read_bytes());raw=np.array(r['raw_ticks']);opened,closed=endpoints()
distance=np.minimum(abs(raw-opened),abs(raw-closed))
print('ADMISSION',json.dumps([dict(motor_id=int(i+1),actual=int(raw[i]),opened=int(opened[i]),closed=int(closed[i]),
    nearest_difference_ticks=int(distance[i])) for i in np.flatnonzero(distance>512)]))
planes=json.loads((OUT/'fixed_palm_side_planes.json').read_bytes())
for p in planes:
    pts=np.array(p['vertices_palm_mm'],np.float32)[:,[0,2]]
    hull=cv.convexHull(pts).reshape(-1,2)
    print('PLANE',json.dumps(dict(y_mm=p['plane_y_mm'],area_mm2=p['area_mm2'],bounds=p['bounds_palm_mm'],
        hull_xz_mm=np.round(hull,3).tolist())))
# List the contact that persists when the hypothetical book ends behind the
# palm's front. This prevents blaming the book alone for every model contact.
for row in r['records']:
    if row['book_front_mm_hypothesis']==-20 and row['book_yhi_mm_hypothesis']==25:
        print('REMAINING',json.dumps(dict(projection=row['support_projection_mm_hypothesis'],
            support_pairs=row['poses'][0]['support_pairs_mm'],self_pairs=row['poses'][0]['self_pairs_mm'])))
