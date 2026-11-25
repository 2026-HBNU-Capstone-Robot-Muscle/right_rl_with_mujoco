import json
from pathlib import Path

root=Path(__file__).resolve().parent
folder=root/'right_hand_rotation_20261024/middle_ring_side_review'
data=json.loads((folder/'paired_native_geometry.json').read_bytes())
for c in data['captures']:
    print('POSE',c['folder'])
    print('CONTACTS',json.dumps(c['contacts']))
    print('CLOSEST_MR',json.dumps(c['closest_middle_ring_hull_pairs'][:1]))
