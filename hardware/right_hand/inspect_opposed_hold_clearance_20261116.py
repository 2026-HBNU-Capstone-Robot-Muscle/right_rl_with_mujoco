"""Inspect saved simulated hold geometry; no physical device access."""
from pathlib import Path
import json,sys
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,inspect_for
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

out=ROOT/'right_hand_rotation_20261116/opposed_lower_grip'
model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()})
hand,_,_=inspect_for(m);geometry=precompute(m,hand);rows=[]
for name in ('independent_initial.npz','hold_end.npz'):
    z=np.load(out/name,allow_pickle=False);d=restore(m,z);cube=m.geom('cube').id
    clearance=sorted(gaps(m,d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3),geometry,True),key=lambda x:x[1])
    rows.append(dict(state=name,lowest_clearance_mm=clearance[:8],center=d.geom_xpos[cube].tolist()))
review=json.loads((out/'review.json').read_text());hold=review['hold']
print(json.dumps(dict(states=rows,tilt=hold['final_up_tilt_deg'],contacts=hold['final_contact_N'],
    initial_orient_change=hold['regrasp_yaw_change_deg'],whole_hold=hold['full_fixed_hold']),ensure_ascii=False),flush=True)
(out/'clearance_diagnostic.json').write_text(json.dumps(rows,indent=2)+'\n')
