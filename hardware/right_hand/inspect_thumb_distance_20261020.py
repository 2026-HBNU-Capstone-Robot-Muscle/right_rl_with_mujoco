"""Check distance-query support for the selected collision geometries."""
import os
os.environ.setdefault("MUJOCO_GL","egl")
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from hand_motor_mapping import ticks_to_sim
from offline_housing_proxy_model import build_with_middle_thumb_hulls
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config("LeapCubeFaceUp");cfg.impl="jax"
m,_=build_with_middle_thumb_hulls(registry.load("LeapCubeFaceUp",config=cfg));d=mujoco.MjData(m)
report=json.loads((ROOT/"right_hand_root_success_20261020/next_preparation_screen.json").read_text())
raw=report["thumb_tip_target_alternatives"]["-64"]["final_raw_ticks"]
d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
for name in ("th_tip","palm_collision_4","palm_collision_8"):
    g=m.geom(name).id
    print(name,"type",int(m.geom_type[g]),"size",m.geom_size[g].tolist(),"position",d.geom_xpos[g].tolist())
a=m.geom("th_tip").id
for name in ("palm_collision_4","palm_collision_8"):
    b=m.geom(name).id
    ray=np.empty(6)
    print(name,"distance",mujoco.mj_geomDistance(m,d,a,b,1.,ray),"closest",ray.tolist())
    original=d.geom_xpos[a].copy()
    d.geom_xpos[a]+=np.array([1.,0,0])
    print("separation control",mujoco.mj_geomDistance(m,d,a,b,2.,None))
    d.geom_xpos[a]=original
print(mujoco.mj_geomDistance.__doc__)
