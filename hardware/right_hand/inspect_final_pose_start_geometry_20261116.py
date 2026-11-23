import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'final_grip_pose_20261116_152024_215914'
r=json.loads((folder/'result.json').read_bytes())
for key in ('camera_recording','camera_startup','empty_start_marker_screen','preflight_state'):
    if key=='preflight_state':
        s=r[key];print('ROBOT',json.dumps({k:s[k] for k in ('position','current','voltage','temperature','torque') if k in s}))
    else: print(key,json.dumps(r.get(key),ensure_ascii=False)[:1800])
images=list(folder.rglob('*.jpg'))
print('IMAGE_COUNT',len(images));print('LAST_IMAGES',[str(p) for p in images[-4:]])
path=ROOT/'right_hand_rotation_20261116/final_grip_pose/passive_roots_validation/empty_recovery_screen/review.json'
if path.exists():
    s=json.loads(path.read_bytes());print('STATIC_FAILURES',json.dumps(s['static_cases'][:2]));print('DYNAMIC',json.dumps(s['dynamic_cases']))
