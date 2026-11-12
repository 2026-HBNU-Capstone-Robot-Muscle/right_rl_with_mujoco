"""Detect markers in stored frames only. No webcam or motor access."""
from collections import Counter
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_grasp_trial_20261022_141534_666300'
OUT=ROOT/'right_hand_rotation_20261022'

if __name__=='__main__':
    OUT.mkdir(exist_ok=True)
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    detector=MarkerDetector(dictionary,range(6),hand_id=40,required_cube_count=1)
    allframes=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    frames=[dict(r,path=FOLDER/'camera'/r['image']) for i,r in enumerate(allframes)
            if i%3==0 and r['phase'] in ('stage_3_loaded_approach','stage_2_fixed_hold','grasp_observation','remove_cube')]
    frames+=[dict(image=name,path=FOLDER/name,phase=name) for name in
              ('ring_released.jpg','open_before_cube.jpg','cube_before_approach.jpg','cube_after_approach.jpg')]
    rows=[];counts=Counter()
    for row in frames:
        frame=cv.imdecode(np.frombuffer(row['path'].read_bytes(),np.uint8),cv.IMREAD_COLOR)
        corners,ids,_=detector.detectMarkers(cv.cvtColor(frame,cv.COLOR_BGR2GRAY))
        ids=[] if ids is None else list(map(int,ids.ravel()))
        counts.update(ids)
        rows.append(dict(image=str(row['path'].relative_to(ROOT)),phase=row['phase'],ids=ids,
            corners={str(i):np.asarray(c).reshape(4,2).tolist() for i,c in zip(ids,corners)}))
    report=dict(status='stored_marker_inventory',frames_analyzed=len(rows),counts=dict(counts),
        simultaneous_hand_cube=sum(40 in r['ids'] and any(i<6 for i in r['ids']) for r in rows),
        physical_rotation_verified=False,rows=rows)
    (OUT/'marker_inventory.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
    print('NAMED',[(r['phase'],r['ids']) for r in rows[-4:]])
    print('PAIRS',[(r['image'],r['ids']) for r in rows if 40 in r['ids'] and any(i<6 for i in r['ids'])][:8])
