"""Check hand-state provenance before using a saved cube seed."""
from pathlib import Path
import json
import numpy as np
from calibrated_quarter_plan_20261028 import LOADED_PATH

ROOT=Path(__file__).resolve().parent
source=np.load(LOADED_PATH)
endpoint=np.load(ROOT/'right_hand_rotation_20261029/reverse_cube_scout/forward_endpoint/loaded_path.npz')
candidate=np.load(ROOT/'right_hand_rotation_20261029/prepositioned_reverse_cube/candidate.npz')
for phase in ('grasp','yaw','release'):
    a=source['goals'][source['phases']==phase]
    b=endpoint['goals'][endpoint['phases']==phase]
    print(json.dumps(dict(phase=phase,source_goals=len(a),endpoint_goals=len(b),
        same_shape=a.shape==b.shape,equal=np.array_equal(a,b),
        source_last=a[-1].tolist(),endpoint_last=b[-1].tolist(),
        endpoint_minus_source_last=(b[-1]-a[-1]).tolist())))
print('candidate_start',candidate['start'].tolist())
print('MODEL SOURCE',str(LOADED_PATH))
