import json,numpy as np
from pathlib import Path
p=Path('right_hand_rotation_20261024/corrected_policy_reference/projected_policy_reference.json')
d=json.loads(p.read_bytes());print('bad_original_reviews',[(i,r['maximum_overlap_mm'],r['maximum_reference_deviation_rad']) for i,r in enumerate(d['reviews']) if r['maximum_overlap_mm']>1e-5])
q=np.array(d['projected_goals_sim_rad']);t=np.array(d['sparse_source_parameters']);delta=np.max(abs(np.diff(q,axis=0)),axis=1)
for i in np.argsort(delta)[-8:]:print('largest_sparse_chord',i,t[i:i+2].tolist(),delta[i],np.argmax(abs(q[i+1]-q[i])))
