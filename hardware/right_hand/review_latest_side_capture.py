"""Inspect saved evidence only; no devices opened."""
import json
from pathlib import Path
import numpy as np
import cv2 as cv
root=Path(__file__).resolve().parent
folders=sorted(root.glob('pose_pair_readonly_20261017_21*'))
old=json.loads((root/'pose_pair_readonly_20261017_061429_504276/result.json').read_text(encoding='utf-8'))
oldpos={r['id']:r['position_ticks_signed'] for r in old['snapshots'][0]['motors']}
rows=[]
for folder in folders:
    r=json.loads((folder/'result.json').read_text(encoding='utf-8'))
    row=dict(folder=folder.name,status=r['status'],error=r.get('error'),images=[p.name for p in folder.glob('*.png')])
    if r['status']=='photo_and_readings_saved':
        motors=r['snapshots'][0]['motors']
        row.update(pair_check=r['pair_check'],change_from_morning_deg={str(x['id']):(x['position_ticks_signed']-oldpos[x['id']])*360/4096 for x in motors},
                   middle_deg=[(x['position_ticks_signed']-2048)*360/4096 for x in motors if 9<=x['id']<=12])
    rows.append(row)
scores={}
for folder in [root/'pose_pair_readonly_20261017_061429_504276',folders[-1]]:
    im=cv.imdecode(np.fromfile(folder/'paired_photo.png',np.uint8),cv.IMREAD_GRAYSCALE)
    h,w=im.shape;roi=im[h//6:5*h//6,w//4:3*w//4]
    scores[folder.name]=dict(central_laplacian_variance=float(cv.Laplacian(roi,cv.CV_64F).var()),
                            note='Relative image detail only; camera/view/content differ, not a calibrated acceptance threshold.')
report=dict(runs=rows,detail_scores=scores,side_image_visual_review='Strong motion blur; individual joint edges cannot be measured reliably.',geometry_calibration_verified=False)
out=root/'middle_reference_review_20261017/side_capture_review.json'
out.write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
