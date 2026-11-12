import json
from pathlib import Path
p=Path(__file__).resolve().parent/'right_hand_rotation_20261026/yaw_after_flip_v1/thumb_opposed_pinch_endpoints.json'
rows=[v for v in json.loads(p.read_bytes())['pairs'] if v['endpoint_promising']]
for i,r in enumerate(rows):
    th,f=r['thumb'],r['finger_candidate']
    print(i,r['finger'],'thumb h/offset',th['height_mm'],th['tangent_offset_mm'],'finger h/offset',f['height_mm'],f['tangent_offset_mm'],'max overlap',round(r['cube_overlap_mm'],3))
