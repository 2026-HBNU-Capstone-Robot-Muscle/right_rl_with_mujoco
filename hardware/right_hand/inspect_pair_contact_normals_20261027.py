import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2'
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,OUT/'contact_inspect_scene.xml');set_cube_physics(m,.055,.1);d=mujoco.MjData(m);z=np.load(OUT/'nominal_grasp_state.npz');d.qpos[:]=z['qpos'];d.qvel[:]=0;d.ctrl[:]=z['ctrl'];mujoco.mj_forward(m,d);cube=m.geom('cube').id;B=d.xmat[m.body('cube').id].reshape(3,3);C=d.qpos[-7:-4]
for name in ('th_tip','mf_tip'):
    g=m.geom(name).id;segment=np.zeros(6);distance=mujoco.mj_geomDistance(m,d,cube,g,1.,segment);print(json.dumps(dict(name=name,type=int(m.geom_type[g]),size=m.geom_size[g].tolist(),distance_mm=distance*1000,segment=segment.tolist(),cube_local_nearest_point=(B.T@(segment[:3]-C)).tolist(),centre=(B.T@(d.geom_xpos[g]-C)).tolist())))
print('CONTACTS',[(m.geom(int(c.geom1)).name,m.geom(int(c.geom2)).name,float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))])
report=json.loads((OUT/'free_cube_review.json').read_bytes());print('YAW_HEIGHTS',[(v['gain_scale'],v['friction'],v['trace'][-1]) for v in report['cases']])
