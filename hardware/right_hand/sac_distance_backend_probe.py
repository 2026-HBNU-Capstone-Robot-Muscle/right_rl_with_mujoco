import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
model,_=build(env);data=mujoco.MjData(model)
q=json.loads((ROOT/'sac_stage0_reset_audit_result.json').read_text())['joint_angles_sim_rad'];data.qpos[:16]=q
data.qpos[-4:]=[1,0,0,0];data.qpos[-7:-4]=[0,0,2]
print(mujoco.__version__,{name:int(v) for name,v in mujoco.mjtDisableBit.__members__.items() if int(v)&model.opt.disableflags})
for flags in (int(model.opt.disableflags),0):
    model.opt.disableflags=flags
    for root in (.55,.60,.65,.75,.85):
        data.qpos[0]=root;data.qpos[1]=-1.047;mujoco.mj_forward(model,data)
        print(flags,root,[(g,o,mujoco.mj_geomDistance(model,data,model.geom(g).id,model.geom(o).id,.2,np.zeros(6)))
                         for g in ('if_bs_visual_support_proxy','if_px_visual_support_proxy') for o in ('support_glass','support_wood')])
