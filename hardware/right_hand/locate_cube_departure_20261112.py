"""Locate cube visibility change in saved frames, then verify source images."""
import bisect
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np
ROOT=Path(__file__).resolve().parent
RUN=ROOT/'nominal_two_turn_trial_20261112_060746_992877'
OUT=ROOT/'right_hand_rotation_20261112/nominal_cycle_actual_review/cube_departure';OUT.mkdir(exist_ok=True)
r=json.loads((RUN/'result.json').read_bytes());ss=r['samples'];enabled=r['enabled_at']
begin=next(s['t'] for s in ss if s['phase']=='stage_13_cycle_first_rotation')
hold=next(s['t'] for s in ss if s['phase']=='stage_14_cycle_first_hold')
frames=json.loads((RUN/'camera/frames.json').read_bytes())['frames']
detector=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),cv.aruco.DetectorParameters())
observations=[];selected=[f for f in frames if hold-105<=f['camera_read_monotonic']-enabled<=hold+2]
for f in selected[::4]:
    path=RUN/'camera'/f['image'];im=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)
    corners,ids,_=detector.detectMarkers(im);found=[]
    if ids is not None:
        for ident,points in zip(ids.ravel(),corners):
            points=points.reshape(4,2);area=abs(cv.contourArea(points))
            if int(ident) in range(6) and area>=150:
                found.append({'id':int(ident),'center_px':points.mean(0).tolist(),'area_px':area})
    observations.append({'image':f['image'],'t':f['camera_read_monotonic']-enabled,'cube_marker_candidates':found})
visible=[v for v in observations if v['cube_marker_candidates']]
result={'status':'saved_marker_visibility_scan_not_a_fall_or_contact_certificate','source_result_sha256':hashlib.sha256((RUN/'result.json').read_bytes()).hexdigest(),
        'first_loaded_t':begin,'first_hold_t':hold,'scanned_frames':len(observations),'marker_visible_frames':len(visible),
        'last_visible':visible[-6:],'observations':observations,'device_accesses':0}
(OUT/'coarse_visibility.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
if visible:
    last=visible[-1]['t'];times=[f['camera_read_monotonic']-enabled for f in frames]
    picks=[]
    for t in (last-3,last-1,last,last+.5,last+1,last+2):
        i=min(range(len(times)),key=lambda k:abs(times[k]-t));f=frames[i]
        picks.append({'image':(RUN/'camera'/f['image']).relative_to(ROOT).as_posix(),'t':times[i]})
    result['source_images_to_review']=picks
    (OUT/'selected_images.json').write_text(json.dumps(picks,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('observations','source_result_sha256')},ensure_ascii=False))
