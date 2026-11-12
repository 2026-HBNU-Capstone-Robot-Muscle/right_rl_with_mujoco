import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env);d=mujoco.MjData(m)
r=json.loads((ROOT/'sac_support_path_audit_result.json').read_text())['poses']['Offline candidate roots >= 0.65 rad']
for label,q in [('initial',r['initial_qpos']),('held',r['hold_2s']['final_qpos'])]:
    d.qpos[:]=q;mujoco.mj_forward(m,d)
    print(label,[(m.geom(c.geom1).name,m.geom(c.geom2).name,round(1000*c.dist,2)) for c in d.contact if c.dist < -.002])
for i in range(m.ngeom):
    n=m.geom(i).name
    if n.startswith(('th_','palm_','leap_mount')):
        print(n,'body',m.body(int(m.geom_bodyid[i])).name,'type',m.geom_type[i],'group',m.geom_group[i],'size',m.geom_size[i].tolist(),'mask',m.geom_contype[i],m.geom_conaffinity[i])
for key,val in env._model_assets.items():
    if str(key).endswith('leap_rh_mjx.xml'):
        (ROOT/'sac_support_source_hand.xml').write_bytes(val if isinstance(val,bytes) else val.encode())
print('ranges',m.jnt_range[:16].tolist())
