"""Inspect native contacts/filters for user-confirmed contacts. Offline only."""
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
root=Path(__file__).resolve().parent;out=root/'paired_pose_audit_20261017'
geo=json.loads((out/'geometry.json').read_text())
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m=env.mj_model;d=mujoco.MjData(m)
d.qpos[:16]=np.array(geo['raw_ticks'])[list(range(4,16))+list(range(4))]*2*np.pi/4096-np.pi
d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
def contacts():
    return [dict(pair=[m.geom(int(c.geom1)).name,m.geom(int(c.geom2)).name],distance_mm=float(c.dist*1000)) for c in d.contact]
original=contacts()
names=('rf_tip','palm_collision_4','mf_tip','th_ds_collision_1')
geoms={name:dict(contype=int(m.geom_contype[m.geom(name).id]),conaffinity=int(m.geom_conaffinity[m.geom(name).id]),
                 body=m.body(int(m.geom_bodyid[m.geom(name).id])).name) for name in names}
explicit=[dict(pair=[m.geom(int(a)).name,m.geom(int(b)).name]) for a,b in zip(m.pair_geom1,m.pair_geom2)]
# Local in-memory diagnostic only: enable just the named ring/palm pair with a new bit.
used=int(np.bitwise_or.reduce(np.r_[m.geom_contype,m.geom_conaffinity]))
bit=1 << used.bit_length()
for name in ('rf_tip','palm_collision_4'):
    g=m.geom(name).id;m.geom_contype[g]|=bit;m.geom_conaffinity[g]|=bit
mujoco.mj_forward(m,d)
diagnostic=contacts()
report=dict(model=str(env.xml_path),geoms=geoms,explicit_pairs=explicit,native_contacts_original=original,
            ring_palm_filter_probe=diagnostic,probe_bit=bit,
            deployed_model_changed=False,hardware_commands_sent=0,
            caveat='Static contact probe, not a validated dynamic trajectory or correction of middle-finger kinematics.')
(out/'collision_filters.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2),flush=True)
