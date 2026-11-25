"""Saved-state native forward-kinematics diagnostics only."""
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'prepared_cube_review_20261022'
obs=json.loads((OUT/'placement_review.json').read_text(encoding='utf-8'))
reset=json.loads((OUT/'support_settling_scan.json').read_bytes())['dynamic_holds'][0]['qpos']
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.100)
d=mujoco.MjData(m);base=ticks_to_sim(obs['placement_goal_ticks'])
d.qpos[:16]=base;d.qpos[-7:]=reset[-7:];mujoco.mj_forward(m,d)
rows=dict(cube_qpos=d.qpos[-7:].tolist(),cube_geom_center=d.geom_xpos[m.geom('cube').id].tolist(),
    cube_geom_local_position=m.geom_pos[m.geom('cube').id].tolist(),
    cube_body=m.body(int(m.geom_bodyid[m.geom('cube').id])).name,
    cube_geom_rotation=d.geom_xmat[m.geom('cube').id].reshape(3,3).tolist(),
    tips={n:dict(size=m.geom_size[m.geom(n).id].tolist(),center=d.geom_xpos[m.geom(n).id].tolist()) for n in ('if_tip','mf_tip','th_tip')},cases=[])
for root in (.65,1.0,1.4):
    for pip in (.05,.7,1.4):
        for dip in (.3,1.2,2.0):
            q=base.copy();q[[0,2,3]]=[root,pip,dip];d.qpos[:16]=q;mujoco.mj_forward(m,d)
            rows['cases'].append(dict(root=root,pip=pip,dip=dip,tip_center=d.geom_xpos[m.geom('if_tip').id].tolist()))
(OUT/'finger_fk_diagnostic.json').write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
