"""Saved manual-tip evidence and local source only; no device access."""
import json,hashlib
from pathlib import Path
import cv2 as cv
import numpy as np
from leap_pose_pair_readonly import validate_pair
root=Path(__file__).resolve().parent
source=root/'pose_pair_readonly_20261018_072023_698689/result.json'
r=json.loads(source.read_text(encoding='utf-8'));assert r['status']=='two_pose_pairs_saved'
out=root/'middle_direction_review_20261018';out.mkdir(exist_ok=True)
poses=[]
for pair in r['direction_pairs']:
    check=validate_pair(*pair['snapshots']);assert check['stationary_endpoints']
    pose=dict(phase=pair['phase'],raw_ticks=[m['position_ticks_signed'] for m in sorted(pair['snapshots'][0]['motors'],key=lambda m:m['id'])],pair_check=check)
    poses.append(pose)
ims=[cv.imdecode(np.fromfile(source.parent/p['relative_folder']/p['image'],np.uint8),cv.IMREAD_COLOR) for p in r['direction_pairs']]
# Full evidence preserved. These are analysis crops, not generated imagery.
crops=[im[20:430,595:825].copy() for im in ims]
sheet=np.concatenate(crops,axis=1)
cv.imencode('.png',sheet)[1].tofile(str(out/'tip_before_after.png'))
report=dict(source=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),poses=poses,
            saved_direction_check=r['direction_check'],
            visual_observation='Distal pad changes tilt relative to adjacent housing; proximal housings appear much less changed. Test reversed within-finger order before offsets.',
            mapping_verified=False,hardware_commands_sent=0)
(out/'evidence.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
attachment=Path('C:/Users/김준서/.codex/attachments/ef2d0fcc-2c5f-4744-b993-75951a2e54fb/Pasted text.txt')
if attachment.exists():print('ATTACHED LOG\n'+attachment.read_text(encoding='utf-8-sig')[-4500:])
repo=Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python')
print('PYTHON REPO FILES', [p.name for p in repo.glob('*.py')])
for p in repo.glob('*.py'):
    lines=p.read_text(encoding='utf-8-sig',errors='replace').splitlines()
    found=[dict(line=i,text=t) for i,t in enumerate(lines,1) if any(s in t.lower() for s in ('역순','reverse','motor_ids','motors =','motors='))]
    if found:print(p.name,json.dumps(found[:20],ensure_ascii=True))
print(json.dumps(dict(poses=poses,deltas=r['direction_check']['delta_degrees_by_motor']),indent=2))
