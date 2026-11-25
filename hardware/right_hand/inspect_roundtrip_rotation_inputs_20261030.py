import json,collections
from pathlib import Path
R=Path(__file__).resolve().parent
for F in sorted(R.glob('yaw_roundtrip_trial_*'))[-2:]:
 r=json.loads((F/'offline_review/physical_and_angle_review.json').read_bytes());rows=r['rows']
 print(F.name,'3+5grid',sum(x.get('accepted',False) and 3 in x.get('ids',[]) for x in rows),'0+3',sum(0 in x.get('ids',[]) and 3 in x.get('ids',[]) for x in rows))
 for group in ('placed','grasp','grasp_hold','forward_yaw','forward_hold','inverse_yaw','inverse_hold'):
  a=[x for x in rows if x['leg_label']==group]
  print(group,'accepted',sum(x.get('accepted',False) for x in a),'id3',sum(3 in x.get('ids',[]) for x in a),'id0',sum(0 in x.get('ids',[]) for x in a),'palm',sum(40 in x.get('ids',[]) for x in a))
