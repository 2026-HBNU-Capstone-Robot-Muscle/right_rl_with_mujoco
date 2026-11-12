import collections
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from cube_marker_readiness import make_detector,inspect_frame
root=Path(__file__).resolve().parent/'slow_grasp_demo_20261023_182105_573825'
r=json.loads((root/'on_paired_pose_review.json').read_bytes())
print('REASONS',collections.Counter(x['reason'] for x in r['rejected']))
print('IDS',collections.Counter(str(x['decoded_ids']) for x in r['rejected']))
detector=make_detector()
for name in ('cube_before_approach.jpg','cube_after_approach.jpg',r['rejected'][len(r['rejected'])//2]['image']):
    path=root/name
    if not path.exists():path=root/'camera'/name
    frame=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)
    print(name,inspect_frame(frame,detector))
