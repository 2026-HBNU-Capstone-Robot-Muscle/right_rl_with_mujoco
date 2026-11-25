"""Check actual simulated contacts, separately from IK support-point proxies."""
from pathlib import Path
import json,sys
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import mujoco,numpy as np
from research_rotation_body_hold_20261104 import restore

out=ROOT/'right_hand_rotation_20261116/opposed_lower_grip'
model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()})
initial=dict(np.load(out/'independent_initial.npz',allow_pickle=False));rows=[]
for path in sorted(out.glob('three_finger_trials*/middle_*/static_proposal.npz')):
    z=dict(initial);z.update(dict(np.load(path,allow_pickle=False)));d=restore(m,z)
    cube=m.geom('cube').id;B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube];contacts=[]
    for c in d.contact:
        if cube not in (int(c.geom1),int(c.geom2)):continue
        other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
        contacts.append(dict(geom=m.geom(other).name,gap_mm=float(c.dist*1000),
            local_mm=((c.pos-C)@B*1000).tolist()))
    row=dict(case=path.parent.relative_to(out).as_posix(),contacts=contacts);rows.append(row)
    print(json.dumps(row),flush=True)
(out/'actual_contact_review.json').write_text(json.dumps(rows,indent=2)+'\n')
