"""Offline fingertip inverse-kinematics seeds, not certified motor paths."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261022'
r = json.loads((OUT/'small_yaw_refined_result.json').read_bytes())
cfg = registry.get_default_config('LeapCubeFaceUp'); cfg.impl = 'jax'
env = registry.load('LeapCubeFaceUp', config=cfg)
m, _ = build_with_middle_thumb_hulls(env)
set_cube_physics(m, side_m=.055, mass_kg=.100)
d = mujoco.MjData(m)
base = np.asarray(r['base_goal_ticks'])
d.qpos[:] = r['validation'][0]['initial_qpos']
center = d.qpos[-7:-4].copy()
d.qpos[:16] = ticks_to_sim(base); mujoco.mj_forward(m,d)
tips = [m.geom(n).id for n in ('if_tip','mf_tip','th_tip')]
initial = d.geom_xpos[tips].copy()
up = -np.asarray(m.opt.gravity); up /= np.linalg.norm(up)
joints = m.actuator_trnid[:,0]
lo = np.maximum(m.jnt_range[joints,0], m.actuator_ctrlrange[:,0])
hi = np.minimum(m.jnt_range[joints,1], m.actuator_ctrlrange[:,1])
motor_lo = np.empty(16); motor_hi = np.empty(16)
for i, motor in enumerate(SIM_TO_MOTOR_IDS):
    motor_lo[motor-1] = np.ceil((lo[i]+np.pi)*4096/(2*np.pi))
    motor_hi[motor-1] = np.floor((hi[i]+np.pi)*4096/(2*np.pi))
seeds = []
cases = [(yaw,0.,0.) for yaw in (-8.,-10.,-12.,-16.,-20.)]
cases += [(yaw,lift,pinch) for yaw in (-10.,-15.) for lift in (3.,6.) for pinch in (1.,2.)]
for yaw,lift,pinch in cases:
    delta = []
    for fraction, limit in ((.5,128),(1.,192)):
        vectors = initial-center
        lengths = np.linalg.norm(vectors,axis=1,keepdims=True)
        vectors = vectors * (1-pinch/1000/lengths)
        target = center + Rotation.from_rotvec(up*np.radians(yaw*fraction)).apply(vectors) + up*lift/1000
        lower = np.maximum(-limit,motor_lo[:12]-base[:12])
        upper = np.minimum(limit,motor_hi[:12]-base[:12])
        lower[[7,11]] = np.maximum(lower[[7,11]],0)
        def loss(x):
            raw = base.astype(float).copy(); raw[:12] += x
            d.qpos[:16] = ticks_to_sim(raw); mujoco.mj_forward(m,d)
            position_error = ((d.geom_xpos[tips]-target)*1000).ravel()
            return np.r_[position_error, x/1000]
        fit = least_squares(loss,np.clip(np.zeros(12)+1e-6,lower+1e-8,upper-1e-8),
                            bounds=(lower,upper),diff_step=.005,max_nfev=180)
        delta.append(np.rint(fit.x).astype(int).tolist())
    seeds.append(dict(target_yaw_deg=yaw,lift_mm=lift,pinch_mm=pinch,delta=delta))
path = OUT/'rigid_yaw_lift_ik_seeds.json'
path.write_text(json.dumps(dict(algorithm='offline_IK_seeds_only',hardware_execution_allowed=False,
                               source_sha256=r['source_sha256'],seeds=seeds),indent=2),encoding='utf-8')
print(json.dumps(seeds,indent=2)); print(path)
