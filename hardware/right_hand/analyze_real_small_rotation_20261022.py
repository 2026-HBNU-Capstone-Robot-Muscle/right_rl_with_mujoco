"""Stored marker/encoder review; no camera connection or hardware writes."""
from collections import Counter
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_small_rotation_trial_20261022_151241_962114'


def main():
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    detector=MarkerDetector(dictionary,range(6),hand_id=40,required_cube_count=1)
    frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    selected=[f for i,f in enumerate(frames) if (f['phase'] in
        ('stage_4_small_rotation','stage_2_fixed_hold','small_rotation_observation')
        or f['phase']=='stage_3_loaded_approach' and i%8==0
        or f['phase']=='place_cube' and i%8==0)]
    rows=[];counts=Counter()
    for f in selected:
        frame=cv.imdecode(np.frombuffer((FOLDER/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
        corners,ids,_=detector.detectMarkers(cv.cvtColor(frame,cv.COLOR_BGR2GRAY))
        ids=[] if ids is None else list(map(int,ids.ravel()));counts.update(ids)
        points={str(i):np.asarray(c).reshape(4,2).tolist() for i,c in zip(ids,corners)}
        row=dict(**f,ids=ids,corners=points)
        if '5' in points:
            a=np.asarray(points['5']);edge=((a[1]-a[0])+(a[2]-a[3]))/2
            row.update(cube_marker_center_px=a.mean(axis=0).tolist(),
                       cube_marker_image_edge_deg=float(np.degrees(np.arctan2(edge[1],edge[0]))))
        rows.append(row)
    rotation=[r for r in rows if r['phase']=='stage_4_small_rotation' and '5' in r['corners']]
    result=dict(status='stored_short_yaw_image_review',source_trial=FOLDER.name,
        analyzed_frames=len(rows),marker_counts=dict(counts),
        hand_cube_pairs=sum(40 in r['ids'] and 5 in r['ids'] for r in rows),
        rotation_frames_with_top_marker=len(rotation),user_reports_cube_unchanged=True,
        physical_desired_rotation_verified=False,
        note='Image-edge angle is a 2D diagnostic, not a calibrated world yaw. No full-sequence before/after conflation.',rows=rows)
    if len(rotation)>=6:
        first=rotation[:3];last=rotation[-3:]
        result['image_edge_change_deg']=float(np.median([r['cube_marker_image_edge_deg'] for r in last])-
                                                np.median([r['cube_marker_image_edge_deg'] for r in first]))
        result['image_center_shift_px']=(np.median([r['cube_marker_center_px'] for r in last],axis=0)-
                                          np.median([r['cube_marker_center_px'] for r in first],axis=0)).tolist()
    (FOLDER/'marker_rotation_review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
    print('PAIRS',[(r['image'],r['phase'],r['ids']) for r in rows if 40 in r['ids'] and 5 in r['ids']][:5])
    print('ANGLES',[(r['image'],round(r['cube_marker_image_edge_deg'],2),r['corners']['5']) for r in rotation])


if __name__=='__main__':main()
