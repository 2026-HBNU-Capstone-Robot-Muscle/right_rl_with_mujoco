import json
from pathlib import Path
import numpy as np
from box_clearance import clearance
root=Path(__file__).resolve().parent;out=root/'paired_pose_audit_20261017'
data=json.loads((out/'geometry.json').read_text())
def box(row):
    v=np.array(row['vertices_marker']);c=v.mean(0)
    axes=np.array([v[4]-v[0],v[2]-v[0],v[1]-v[0]]).T
    size=np.linalg.norm(axes,axis=0)/2;return c,axes/(2*size),size
thumb=[b for b in data['boxes'] if b['body'] in ('th_px','th_ds') and b['name']!='th_tip']
report=[]
for prefix in ('if_','mf_','rf_'):
    rows=[]
    for a in data['boxes']:
        if not a['body'].startswith(prefix):continue
        for b in thumb:
            gap,hit=clearance(*box(a),*box(b))
            rows.append(dict(pair=[a['name'],b['name']],gap_mm=gap*1000,overlap=hit))
    report.append(dict(finger=prefix,minimum=min(rows,key=lambda x:x['gap_mm'])))
(out/'all_finger_thumb_gaps.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
