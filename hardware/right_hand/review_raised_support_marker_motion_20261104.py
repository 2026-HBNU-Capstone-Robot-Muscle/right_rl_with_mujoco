"""Image-plane comparison only; no metric 3D registration or devices."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np

R=Path(__file__).resolve().parent
O=R/'right_hand_rotation_20261104/raised_support_capture/actual_review'
params=cv.aruco.DetectorParameters();params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_SUBPIX
det=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),params)
rows=[]
for folder in sorted(R.glob('raised_support_capture_20261104_*')):
    photos=[]
    for name in ('cube_on_support.jpg','cube_after_static_hold.jpg'):
        p=folder/name
        if not p.exists():continue
        img=cv.imdecode(np.frombuffer(p.read_bytes(),np.uint8),cv.IMREAD_COLOR)
        corners,ids,rejected=det.detectMarkers(img)
        markers={} if ids is None else {int(i):c.reshape(4,2).astype(float) for i,c in zip(ids.ravel(),corners)}
        row=dict(file=p.relative_to(R).as_posix(),IDs=sorted(markers),corners={str(k):v.tolist() for k,v in markers.items()})
        if 40 in markers:
            H=cv.getPerspectiveTransform(markers[40].astype(np.float32),np.array([[0,0],[1,0],[1,1],[0,1]],np.float32))
            row['cube_markers_in_palm_marker_plane_units']={str(k):cv.perspectiveTransform(v.astype(np.float32).reshape(1,4,2),H).reshape(4,2).mean(0).tolist() for k,v in markers.items() if k<6}
        photos.append(row)
    comparisons=[]
    if len(photos)==2:
        a,b=photos
        for ident in set(a.get('cube_markers_in_palm_marker_plane_units',{}))&set(b.get('cube_markers_in_palm_marker_plane_units',{})):
            pa=np.array(a['cube_markers_in_palm_marker_plane_units'][ident]);pb=np.array(b['cube_markers_in_palm_marker_plane_units'][ident])
            comparisons.append(dict(marker_ID=int(ident),normalized_image_plane_center_change=float(np.linalg.norm(pb-pa)),unit='palm-marker widths in planar projection; not cube displacement in mm'))
    rows.append(dict(folder=folder.name,photos=photos,comparisons=comparisons))
report=dict(status='saved_photo_image_plane_marker_comparison',runs=rows,camera_to_robot_registration_verified=False,metric_displacement_verified=False,physical_rotation_verified=False,devices_opened=False)
(O/'image_plane_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps([{k:v for k,v in row.items() if k!='photos'}|{'photo_IDs':[p['IDs'] for p in row['photos']]} for row in rows],ensure_ascii=False,indent=2))
