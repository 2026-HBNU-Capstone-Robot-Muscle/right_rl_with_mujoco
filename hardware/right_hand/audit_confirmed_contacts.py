"""Check two user-confirmed contacts in the same paired encoder reconstruction."""
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
groups={
    'middle_tip_to_thumb_housing':(['mf_tip'],[b['name'] for b in data['boxes'] if b['body'] in ('th_px','th_ds') and b['name']!='th_tip']),
    'ring_tip_to_palm':(['rf_tip'],[b['name'] for b in data['boxes'] if b['body']=='palm']),
    'ring_distal_link_to_palm':([b['name'] for b in data['boxes'] if b['body']=='rf_ds'],[b['name'] for b in data['boxes'] if b['body']=='palm'])}
byname={b['name']:b for b in data['boxes']}
results={}
for name,(aa,bb) in groups.items():
    pairs=[]
    for a in aa:
        for b in bb:
            gap,hit=clearance(*box(byname[a]),*box(byname[b]))
            pairs.append(dict(pair=[a,b],gap_mm=gap*1000,overlap=hit))
    results[name]=min(pairs,key=lambda r:r['gap_mm'])
report=dict(user_confirmed=dict(ID12='touches thumb end-motor housing',ID16='touches palm; exact tip/housing surface unspecified'),
            results=results,mapping_changed=False,led_identification_request_cancelled=True,
            caveat='Model collision boxes, not measured physical distances. Neither contact supplies an exact 3D point or normal.')
(out/'confirmed_contacts.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
