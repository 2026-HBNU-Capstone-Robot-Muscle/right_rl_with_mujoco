"""Inspect actual saved markers and local calibration names, no devices."""
from pathlib import Path
import json,numpy as np,cv2 as cv
root=Path(__file__).resolve().parent;folder=root/'loaded_quarter_trial_20261025_171607_019752'
d=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),cv.aruco.DetectorParameters())
for name in ('cube_before_turn.jpg','cube_after_turn.jpg'):
    im=cv.imdecode(np.frombuffer((folder/name).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    corners,ids,rejected=d.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
    print(name,json.dumps({int(i):c.reshape(4,2).tolist() for i,c in zip(ids.ravel(),corners)} if ids is not None else {}))
print('CALIBRATION_FILES',[p.name for p in root.glob('*calib*') if p.is_file()][:35])
for p in sorted(root.glob('*.py')):
    if any(s in p.name for s in ('cube_pose_readonly','marker_geometry','cube_closeup','pose_readonly')):print('POSE_SOURCE',p.name)
for p in (root.parent/'work/Leap_Test/vision/rubik_center/track_cube.py',):
    if p.exists():print('TRACK_SOURCE',str(p));print('\n'.join(p.read_text(encoding='utf-8').splitlines()[:90]))
