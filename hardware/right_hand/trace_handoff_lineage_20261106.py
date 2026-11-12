"""Read source identities and goal-array sizes for the successful checkpoint."""
import json
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip'
folders=('torque_cap_sensitivity/cap0.15','near_quarter_full_replay','postturn_middle_clearance','postturn_thumb_clearance','index_countergrip_workspace/p12_seeds6','countergrip_level/seed0_source1_lift2_normal1.5_n850','countergrip_level/seed0_source1_lift0_normal2.5_n900_resume_lower_staged_freebase','temporary_handoff_load','supported_handoff_first_full','supported_handoff_search_mf_active_second')
for folder in folders:
    path=O/folder/'review.json';d=json.loads(path.read_bytes());refs={k:v for k,v in d.items() if any(word in k for word in ('source','candidate')) and isinstance(v,str) and 'sha256' not in k};print(json.dumps(dict(folder=folder,keys=list(d),refs=refs)),flush=True)
    for p in sorted((O/folder).glob('*.npz')):
        z=np.load(p)
        if 'goals' in z:print(json.dumps(dict(goalfile=p.relative_to(R).as_posix(),goals=len(z['goals']),start=z['start'].tolist(),last=z['goals'][-1].tolist())),flush=True)
    if folder.endswith('active_second'):
        for p in sorted((O/folder).glob('verified*/candidate.npz')):print('VERIFIED',p.relative_to(R).as_posix())
