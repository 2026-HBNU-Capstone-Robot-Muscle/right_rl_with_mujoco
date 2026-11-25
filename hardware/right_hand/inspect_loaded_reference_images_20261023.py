"""Read stored image IDs and metadata only."""
import json
from collections import Counter
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector

ROOT=Path(__file__).resolve().parent
folder=ROOT/'cube_supported_rotation_trial_20261023_034640_750784'
frames=json.loads((folder/'camera/frames.json').read_bytes())['frames']
phase_counts=Counter(f['phase'] for f in frames)
print('PHASE_COUNTS',dict(phase_counts))
detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(40),hand_id=41,required_cube_count=1)
chosen=[]
for phase in ('release_contact_confirmation','stage_2_empty_open','open_before_cube','place_cube',
              'stage_3_loaded_approach','stage_4_supported_cube_rotation'):
    group=[f for f in frames if f['phase']==phase]
    if group:
        chosen += [group[i] for i in sorted(set([0,len(group)//2,len(group)-1]))]
records=[]
for f in chosen:
    a=cv.imdecode(np.frombuffer((folder/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    corners,ids,_=detector.detectMarkers(cv.cvtColor(a,cv.COLOR_BGR2GRAY))
    ids=[] if ids is None else list(map(int,ids.ravel()))
    points={str(i):np.asarray(c).reshape(4,2).tolist() for i,c in zip(ids,corners)}
    row=dict(**f,ids=ids,corners=points);records.append(row)
    print(f['phase'],f['image'],ids,detector.stats['min_edge_px_by_id'])
(folder/'reference_image_detections.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
