import json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent
c=json.loads((root/'right_hand_rotation_20261023/same_grasp_small_yaw_candidate.json').read_bytes())
print('CANDIDATE',{k:v for k,v in c.items() if k!='goal_ticks' and k!='model_manifest'})
for filename in ('hand_registration_geometry.py','review_successful_slow_grasp_20261023.py','overlay_on_grasp_geometry_20261023.py'):
    print('SOURCE',filename)
    print((root/filename).read_text(encoding='utf-8')[:12000])
