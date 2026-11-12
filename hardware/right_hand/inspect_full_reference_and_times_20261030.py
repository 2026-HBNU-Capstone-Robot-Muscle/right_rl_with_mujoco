import json
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent
F=R/'yaw_roundtrip_trial_20261030_011416_269699';r=json.loads((F/'result.json').read_bytes())
s=next(s for s in r['samples'] if s['phase']=='stage_7_horizontal_yaw');print('SAMPLE_KEYS',list(s));print('SCALARS',{k:v for k,v in s.items() if not isinstance(v,(list,dict))})
for f in ('right_hand_rotation_20261027/yaw90_clearance_t8_early_root16/candidate.npz','right_hand_rotation_20261027/yaw90_clearance_t8_early_root16/full_loaded_27/loaded_path.npz'):
 z=np.load(R/f);print(f,[(k,z[k].shape) for k in z.files])
 if 'phases' in z:print(dict(zip(*[v.tolist() for v in np.unique(z['phases'],return_counts=True)])))
