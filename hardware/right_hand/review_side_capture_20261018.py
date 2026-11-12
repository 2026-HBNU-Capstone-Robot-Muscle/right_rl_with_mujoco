"""Summarize saved side evidence without device access."""
import json,hashlib
from pathlib import Path
from leap_pose_pair_readonly import validate_pair
root=Path(__file__).resolve().parent
folder=root/'pose_pair_readonly_20261018_032605_137975'
source=folder/'result.json';r=json.loads(source.read_text(encoding='utf-8'))
assert r['status']=='photo_and_readings_saved'
check=validate_pair(*r['snapshots']);assert check['stationary_endpoints']
positions={x['id']:x['position_ticks_signed'] for x in r['snapshots'][0]['motors']}
comparisons={}
for name in ('pose_pair_readonly_20261017_061429_504276','pose_pair_readonly_20261017_212057_778684'):
    prior=json.loads((root/name/'result.json').read_text(encoding='utf-8'))
    old={x['id']:x['position_ticks_signed'] for x in prior['snapshots'][0]['motors']}
    comparisons[name]={str(i):round((positions[i]-old[i])*360/4096,4) for i in range(1,17)}
report=dict(source=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),status=r['status'],
    pair_check=check,raw_ticks=[positions[i] for i in range(1,17)],
    middle_converted_deg=[(positions[i]-2048)*360/4096 for i in range(9,13)],
    changes_deg=comparisons,burst_count=len(r['photo_burst']['samples']),selected_index=r['photo_burst']['selected_index'],
    visual_review='Image is substantially clearer than prior blurred side image. Visible housing edges suitable for qualitative comparison, not certified 3D calibration.',
    calibration_selected=False,hardware_commands_sent=0)
out=root/'middle_side_review_20261018';out.mkdir(exist_ok=True)
(out/'capture_review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
