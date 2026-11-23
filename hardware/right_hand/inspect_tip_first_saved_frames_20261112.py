import json
from pathlib import Path
root=Path(__file__).resolve().parent
for name in ['tip_first_trial_20261112_173936_071900','tip_first_trial_20261112_173745_907847','tip_first_trial_20261112_173345_707995']:
    folder=root/name
    pictures=sorted((folder/'camera').glob('*.jpg'))
    print(name,'pictures',len(pictures))
    for i in sorted(set([len(pictures)//4,3*len(pictures)//4])):
        if pictures:print(str(pictures[i]))
    print('camera_metadata',[(p.name,p.stat().st_size) for p in (folder/'camera').glob('*.json*')])
    r=json.loads((folder/'empty_recovery_screen/review.json').read_bytes())
    pairs={}
    for c in r['static_cases']:
        for pair,depth in c.get('worst_self_pairs_mm',{}).items():pairs[pair]=max(pairs.get(pair,0),depth)
    print('static_passed',sum(c['passed'] for c in r['static_cases']),'dynamic_passed',sum(c['passed'] for c in r['dynamic_cases']),'pairs',pairs)
print('CV2_AVAILABLE')
try:
    import cv2
    print(cv2.__version__)
except ImportError:print(False)
