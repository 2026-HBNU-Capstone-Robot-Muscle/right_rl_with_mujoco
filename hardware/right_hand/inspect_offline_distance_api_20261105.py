"""Read-only API/model diagnostics for offline geometry queries."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json
import numpy as np,mujoco
from pathlib import Path
from research_rotation_body_hold_20261104 import restore
R=Path(__file__).resolve().parent;CP=R/'right_hand_rotation_20261105/preintegration/reference_end'
m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));d=restore(m,np.load(CP/'end_state.npz'));cube=m.geom('cube').id
report=dict(version=mujoco.__version__,enable_flags=int(m.opt.enableflags),disable_flags=int(m.opt.disableflags),enable_bits={n:int(v) for n,v in mujoco.mjtEnableBit.__members__.items()},disable_bits={n:int(v) for n,v in mujoco.mjtDisableBit.__members__.items()},queries=[])
for name in ('if_bs_collision_1','mf_tip','th_tip'):
    for dist in (.002,.01,.15):
        line=np.zeros(6);value=mujoco.mj_geomDistance(m,d,cube,m.geom(name).id,dist,line);report['queries'].append(dict(geom=name,distmax=dist,value=value,fromto=line.tolist()))
print(json.dumps(report),flush=True)
